# main.py
import cv2, time
from camera_capture import Camera
from lidar import TFLuna
from detection_yolo8 import Detector
from tracker import build_tracker
from vision import (
    draw_track,
    draw_center_dot,
    draw_guidance,
    draw_system_status,
    draw_sensor_info,
    draw_command_info,
)
from control import SimpleGuidance, GuidanceConfig
from offboard_adapter import VelocityController, MapConfig, SafetyConfig
from udp_stream import create_udp_writer
from px4_mavsdk_telemetry import Px4TelemetryClient


DETECT_EVERY = 5  # YOLO cadence

def main():
    cv2.setUseOptimized(True)

    cam = Camera()
    lidar = TFLuna()
    det = Detector(model_path="yolov8s.pt", target_class="person", conf_thres=0.4, imgsz=480)
# -------------------------------------------------------------------
# Create the velocity controller (sits between vision+lidar and Pixhawk)
# -------------------------------------------------------------------
    controller = VelocityController(
        map_cfg=MapConfig(
            # ---------------- General tuning ----------------
            deadband_px=20,          # ignore small dx/dy jitter (pixels)
            desired_m=3.0,           # target hold distance (m)
            K_yaw=0.0045,            # yaw-rate gain per pixel offset (dx)
            K_vz=0.0025,             # vertical speed gain per pixel offset (dy)
            K_vx=0.6,                # forward/backward speed per meter distance error
            max_vx=1.2,              # clamp for forward/backward speed (m/s)
            max_vz=0.6,              # clamp for vertical speed (m/s)
            max_yawrate=0.8,         # clamp for yaw-rate (rad/s)
            tol_enter_m=0.25,        # hysteresis: start moving if error > this
            tol_exit_m=0.15,         # hysteresis: stop when within this band

            # ---------------- Alignment gating ----------------
            align_px=60,             # allow full vx if |dx| <= 60 px
            hard_gate_px=120,        # freeze vx if |dx| >= 120 px (object too far to side)

            # ---------------- Lateral control (optional) ----------------
            enable_vy=False,         # keep False for now (use yaw centering instead)
            K_vy=0.0035,             # small lateral velocity gain if enabled
            max_vy=0.5,              # cap for lateral speed (m/s)

            # ---------------- Robustness ----------------
            lost_timeout_s=0.4       # freeze vx (& vy) if target unseen for >0.4 s
        ),

        safety=SafetyConfig(
            ENABLE_FLIGHT=False,     # DRYRUN mode (safe). True = sends to Pixhawk.
            SEND_HZ=10.0,            # update/send rate (Hz)
            link_timeout_s=2.0,      # reserved (not used yet)
            serial="/dev/ttyTHS1",   # Jetson UART device linked to Pixhawk TELEM2
            baud=57600,              # baud rate (matches Pixhawk TELEM2)
            arm_when_enabled=False   # keep False; arm manually via RC or Mission Planner
        ),

        backend="print"          # "print" | "ardupilot" | "px4" (future)
    )


    # PX4 MAVSDK telemetry client (for HUD)
    tele = Px4TelemetryClient(serial="/dev/ttyTHS1", baud=57600)
    tele.start()
    # Start the controller
    controller.start()
    # Fast tracker by default; switch prefer to "csrt" for tighter but slower
    tracker = build_tracker(
        prefer="mosse",
        detect_every=DETECT_EVERY,
        reinit_iou_thresh=0.35,
        smooth=0.35,
        refine_mode="blend",
        refine_alpha=0.6,
        inflate_scale=1.03,
        max_scale_step=1.25,
    )

    # Debug guidance (print only)
    guide = SimpleGuidance(
        GuidanceConfig(
            deadband_px=20,
            hold_ms_on_missing=400,
            desired_m=3.0,
            tol_enter_m=0.25,
            tol_exit_m=0.15,
            rate_limit_hz=10.0,
            combine_axes=True
        )
    )

    cam.start()
    # UDP stream: smaller, 640x360, 30 FPS
    udp_width, udp_height, udp_fps = 640, 360, 30
    udp_out = create_udp_writer(
        host="192.168.1.107",   # change if needed
        port=5600,
        width=udp_width,
        height=udp_height,
        fps=udp_fps,
    )
    last_dets = []
    fidx = 0

    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue
            fidx += 1

            # Sparse detection
            if fidx % DETECT_EVERY == 0:
                last_dets, _, _ = det.infer(frame)

            # Tracker update
            tracks = tracker.update(frame, last_dets)
            track = tracks[0] if tracks else None

            # Center dot in the frame
            frame_center, frame = draw_center_dot(frame)
            fx, fy = frame_center

            # Track overlay + compute offset
            dx = dy = None
            if track is not None:
                frame = draw_track(frame, track)
                x1, y1, x2, y2 = track.bbox
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                dx = cx - fx
                dy = cy - fy
                track_center = (cx, cy)
                has_target = True
            else:
                track_center = None
                has_target = False

            # Guidance visualization (capture ring + line)
            frame = draw_guidance(frame, frame_center, track_center)

            # LiDAR distance
            dist_m = lidar.read_distance_m()

            # Debug guidance (console only)
            guide.update(track_center=track_center, frame_center=(fx, fy), lidar_m=dist_m)

            # Compute/setpoint and send to backend
            sp = controller.compute_from_tracking(
                track_center=track_center,
                frame_center=(fx, fy),
                lidar_m=dist_m,
            )
            controller.send(sp)  # DRYRUN prints unless ENABLE_FLIGHT=True

            # Get latest PX4 telemetry for HUD
            t_state = tele.get_state()
            mode_str = t_state.mode
            sats_str = str(t_state.sats)
            alt_str = f"{t_state.alt_m:.1f}"
            bat_str = f"{t_state.bat_pct}%" if t_state.bat_pct >= 0 else "??%"
            armed_flag = t_state.armed

            frame = draw_system_status(
                frame,
                flight_on=controller.safety.ENABLE_FLIGHT,
                mode=mode_str,
                sats=sats_str,
                alt_m=alt_str,
                bat_str=bat_str,
                armed=armed_flag,
            )

            frame = draw_sensor_info(frame, has_target=has_target, dist_m=dist_m)
            frame = draw_command_info(frame, sp=sp, dx=dx, dy=dy, has_target=has_target)
            # Resize for UDP stream (frame is already flipped by Camera())
            stream_frame = cv2.resize(frame, (udp_width, udp_height))
            udp_out.write(stream_frame)
            # Display
            cv2.imshow("Drone Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        lidar.close()
        cam.stop()
        udp_out.release()
        tele.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
