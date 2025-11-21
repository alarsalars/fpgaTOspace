# vision.py
import cv2
import math

FONT = cv2.FONT_HERSHEY_SIMPLEX
def put_text_outline(frame, text, org, scale=0.6,
                     color=(255, 255, 255), thickness=1):
    """
    Draws text with a thin black outline for readability.
    """
    x, y = org
    # shadow/outline
    cv2.putText(frame, text, (x, y), FONT, scale,
                (0, 0, 0), thickness + 2, cv2.LINE_AA)
    # main text
    cv2.putText(frame, text, (x, y), FONT, scale,
                color, thickness, cv2.LINE_AA)
    return frame

def draw_track(frame, track):
    if track is None:
        return frame
    x1,y1,x2,y2 = track.bbox
    cx=(x1+x2)//2; cy=(y1+y2)//2
    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,215,255),2)
    cv2.circle(frame,(cx,cy),4,(255,255,255),-1)
    label=f"ID {track.track_id}"
    cv2.putText(frame,label,(x1,max(0,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,215,255),2)
    return frame

def draw_center_dot(frame):
    h,w = frame.shape[:2]
    center=(w//2,h//2)
    cv2.circle(frame, center, 5, (0,0,255), -1)
    return center, frame

def draw_guidance(frame, frame_center, target_center,
                  inner_radius: int = 30,
                  outer_radius: int = 120):
    """
    Visualize the relationship between:
      - frame_center (red dot in the middle of the image)
      - target_center (white dot in the bbox)

    - Draws a thin 'capture ring' around the frame center.
    - Draws a guidance line from frame center -> target center
      when the target is outside the ring.
    - Line color encodes how far the target is:
        inside inner_radius: no line (good alignment)
        between inner_radius and outer_radius: yellow
        beyond outer_radius: red
    """
    h, w = frame.shape[:2]
    fx, fy = frame_center

    # Subtle capture ring around the central red dot
    ring_color = (180, 180, 180)  # light gray
    cv2.circle(frame, (fx, fy), inner_radius, ring_color, 1, lineType=cv2.LINE_AA)

    # No target -> nothing else to draw
    if target_center is None:
        return frame

    tx, ty = target_center
    dx = tx - fx
    dy = ty - fy
    dist = math.hypot(dx, dy)

    # If target is inside the ring, consider it "aligned": no line needed
    if dist < inner_radius:
        return frame

    # Choose line color based on distance
    if dist < outer_radius:
        line_color = (0, 255, 255)   # yellow: small error
    else:
        line_color = (0, 0, 255)     # red: large error

    # Draw guidance line from frame center to target center
    cv2.line(frame, (fx, fy), (tx, ty), line_color, 2, lineType=cv2.LINE_AA)

    return frame

def draw_system_status(frame,
                       flight_on: bool,
                       mode: str,
                       sats: str,
                       alt_m: str,
                       bat_str: str,
                       armed: bool):
    """
    Top-left: PX4 + mode + sats + alt + battery + armed.
    For now mode/sats/alt/bat may be placeholders, but layout is ready for real telemetry.
    """
    status = "LIVE" if flight_on else "DRYRUN"
    arm_text = "ARMED" if armed else "DISARMED"

    line1 = f"PX4 | {status} | {arm_text}"
    line2 = f"MODE: {mode} | SATS: {sats} | ALT: {alt_m} m | BAT: {bat_str}"

    frame = put_text_outline(frame, line1, (10, 22), scale=0.6,
                             color=(0, 255, 255), thickness=1)
    frame = put_text_outline(frame, line2, (10, 42), scale=0.6,
                             color=(255, 255, 255), thickness=1)
    return frame

def draw_sensor_info(frame, has_target: bool, dist_m: float | None):
    """
    Top-right: target status + range in a single, non-flickering block.
    Example:
      TARGET LOCKED | 2.34 m
      NO TARGET     | --.- m
    """
    h, w = frame.shape[:2]

    if dist_m is not None and dist_m > 0:
        range_str = f"{dist_m:.2f} m"
    else:
        range_str = "--.- m"

    if has_target:
        text = f"TARGET LOCKED | {range_str}"
        color = (0, 255, 0)
    else:
        text = f"NO TARGET | {range_str}"
        color = (0, 0, 255)

    (tw, th), _ = cv2.getTextSize(text, FONT, 0.6, 1)
    org = (w - tw - 10, 22)

    return put_text_outline(frame, text, org, scale=0.6,
                            color=color, thickness=1)



def draw_command_info(frame, sp, dx, dy, has_target: bool):
    """
    Bottom-left: commanded velocities + offset.
    """
    h, w = frame.shape[:2]

    # Line 1: commanded body velocities
    line1 = f"CMD vx={sp.vx:+.2f} vy={sp.vy:+.2f} vz={sp.vz:+.2f} yaw={sp.yaw_rate:+.2f}"

    # Line 2: pixel offset or placeholder
    if has_target and dx is not None and dy is not None:
        line2 = f"OFFSET: ({dx:+d}, {dy:+d})"
    else:
        line2 = "OFFSET: (--, --)"

    frame = put_text_outline(frame, line1, (10, h - 40), scale=0.6,
                             color=(0, 255, 255), thickness=1)
    frame = put_text_outline(frame, line2, (10, h - 18), scale=0.6,
                             color=(255, 255, 0), thickness=1)
    return frame


