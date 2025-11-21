# offboard_adapter.py
"""
Purpose
-------
Translate vision guidance (dx, dy, range_m) into safe drone motion commands
(vx, vy, vz, yaw_rate) and send them to a chosen backend.

Backends
--------
- PrintBackend: DRYRUN — only logs setpoints (safe on-bench).
- ArduPilotBackend: Sends MAVLink SET_POSITION_TARGET_LOCAL_NED (body-frame velocities).
- Px4MavsdkBackend: Stub for future PX4 Offboard (prints only).

Safety & Behavior
-----------------
- Deadbands: ignore tiny pixel jitter near image center.
- Clamps: cap max speeds & yaw-rate.
- Distance hold (vx): move forward/back to maintain desired range with hysteresis.
- Alignment gate: scale/freeze vx if the target is off-axis (|dx| too big).
- Lost-target timeout: freeze vx (and vy) if target not seen for a short time.
- Master switch: ENABLE_FLIGHT must be True before anything is actually sent.

Sign conventions
----------------
- Frame: MAV_FRAME_BODY_NED for velocity commands.
- +vx: forward, +vy: right, +vz: DOWN (NED).
- yaw_rate: rad/s (right/clockwise positive as in MAVLink yaw rate field).
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple
import time
import math

# =========================== Config structures ===========================

@dataclass
class MapConfig:
    """
    Mapping/tuning parameters from (dx, dy, range_m) to (vx, vy, vz, yaw_rate).
    NOTE: In NED / body-NED, +vz is DOWN (so "go up" means negative vz).

    deadband_px: pixels around center where we ignore dx/dy (reduce jitter)
    K_yaw:       rad/s per pixel for yaw centering from dx
    K_vz:        m/s per pixel for vertical centering from dy  (+vz is DOWN)
    desired_m:   target distance to hold (meters)
    K_vx:        m/s per meter of distance error (forward/back)
    max_*:       safety clamps for motion
    tol_*:       hysteresis around desired distance (prevents oscillation)

    NEW (robustness):
    align_px:    within ±align_px → allow full vx
    hard_gate_px:≥ this |dx| → force vx=0 (don’t surge when target is off-axis)
    enable_vy:   allow small lateral strafing based on dx
    K_vy:        m/s per pixel for vy (keep small)
    max_vy:      clamp for vy
    lost_timeout_s: if target missing for this long → freeze vx (& vy)
    """
    # Existing tuning
    deadband_px: int = 20
    K_yaw: float = 0.0045
    K_vz: float = 0.0025
    desired_m: float = 3.0
    K_vx: float = 0.6
    max_vx: float = 1.2
    max_vy: float = 0.0        # default 0; can enable later
    max_vz: float = 0.6
    max_yawrate: float = 0.8
    tol_enter_m: float = 0.25
    tol_exit_m: float = 0.15

    # NEW: alignment gating & lateral
    align_px: int = 60
    hard_gate_px: int = 120
    enable_vy: bool = False
    K_vy: float = 0.0035
    lost_timeout_s: float = 0.4


@dataclass
class SafetyConfig:
    """
    Safety + link parameters.

    ENABLE_FLIGHT: False keeps everything DRYRUN, even on the Ardu backend.
    SEND_HZ:       rate limit for sending/printing commands.
    serial/baud:   ArduPilot UART device & baud (e.g. /dev/ttyTHS1 @ 57600).
    arm_when_enabled: optional auto-arm when ENABLE_FLIGHT=True (prefer RC arming).
    """
    ENABLE_FLIGHT: bool = False
    SEND_HZ: float = 10.0
    link_timeout_s: float = 2.0
    serial: str = "/dev/ttyTHS1"
    baud: int = 57600
    arm_when_enabled: bool = False


@dataclass
class Setpoint:
    """
    Body-NED motion command with timestamp.

    vx, vy, vz: m/s  (+vx forward, +vy right, +vz DOWN)
    yaw_rate:   rad/s (right/clockwise positive)
    """
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    stamp: float


# ============================= Core mapper ==============================

class GuidanceMapper:
    """
    Turn (dx, dy, lidar_m) into a bounded Setpoint using:
      - yaw control from dx (with deadband)
      - vertical control from dy  (+vz is DOWN)
      - forward/back distance hold from lidar around desired_m (hysteresis)
      - alignment gating for vx based on |dx|
      - lost-target timeout to freeze vx (and vy) if target not seen
      - optional lateral vy from dx (disabled by default for safety)
    """

    def __init__(self, map_cfg: MapConfig):
        self.cfg = map_cfg
        self._want_forward = False
        self._want_backward = False
        # Bookkeeping for robustness
        self._last_seen_t = 0.0
        self._last_dx = 0

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def compute(
        self,
        dx_px: Optional[int],
        dy_px: Optional[int],
        lidar_m: Optional[float],
    ) -> Setpoint:
        """
        Inputs
        ------
        dx_px: horizontal pixel error (cx - fx). Right of center => +dx.
        dy_px: vertical pixel error (cy - fy). Below center    => +dy.
        lidar_m: measured distance to target (m).

        Output
        ------
        Setpoint(vx, vy, vz, yaw_rate, stamp)
        """
        now = time.time()
        yaw_rate = 0.0
        vz = 0.0
        vx = 0.0
        vy = 0.0

        # ---- Update last-seen info ----
        if dx_px is not None:
            self._last_seen_t = now
            self._last_dx = dx_px

        # ---- Yaw from dx (deadband + clamp) ----
        if dx_px is not None and abs(dx_px) > self.cfg.deadband_px:
            yaw_rate = self._clamp(self.cfg.K_yaw * dx_px, -self.cfg.max_yawrate, self.cfg.max_yawrate)

        # ---- Vertical from dy (deadband + clamp) ----
        if dy_px is not None and abs(dy_px) > self.cfg.deadband_px:
            vz_cmd = self.cfg.K_vz * dy_px  # dy>0 (below center) => +vz (DOWN)
            vz = self._clamp(vz_cmd, -self.cfg.max_vz, self.cfg.max_vz)

        # ---- Optional lateral vy from dx ----
        if self.cfg.enable_vy and dx_px is not None and abs(dx_px) > self.cfg.deadband_px:
            vy_cmd = self.cfg.K_vy * dx_px  # dx>0 => strafe right
            vy = self._clamp(vy_cmd, -self.cfg.max_vy, self.cfg.max_vy)

        # ---- Lost-target freeze for vx (& vy) ----
        if (now - self._last_seen_t) > self.cfg.lost_timeout_s:
            # Target stale -> hold position forward/strafe; keep yaw/vz corrections allowed.
            return Setpoint(
                vx=0.0,
                vy=0.0 if self.cfg.enable_vy else vy,
                vz=vz,
                yaw_rate=yaw_rate,
                stamp=now,
            )

        # ---- Distance hold for vx (with hysteresis) ----
        if lidar_m is not None and lidar_m > 0.0:
            err = lidar_m - self.cfg.desired_m

            # Enter hysteresis regions
            if err > self.cfg.tol_enter_m:
                self._want_forward, self._want_backward = True, False
            elif err < -self.cfg.tol_enter_m:
                self._want_forward, self._want_backward = False, True

            # Exit when inside tight band
            if abs(err) <= self.cfg.tol_exit_m:
                self._want_forward = self._want_backward = False

            raw_vx = 0.0
            if self._want_forward:
                raw_vx = self._clamp(self.cfg.K_vx * err, 0.0, self.cfg.max_vx)
            elif self._want_backward:
                raw_vx = self._clamp(self.cfg.K_vx * err, -self.cfg.max_vx, 0.0)

            # Alignment gating/scale for vx based on |dx|
            dx_for_gate = dx_px if dx_px is not None else self._last_dx
            adx = abs(dx_for_gate)

            if adx >= self.cfg.hard_gate_px:
                # Way off-axis → don't drive forward/back blindly
                vx = 0.0
            else:
                if adx <= self.cfg.align_px:
                    scale = 1.0
                else:
                    # Smooth cosine falloff from align_px .. hard_gate_px
                    r = (adx - self.cfg.align_px) / float(self.cfg.hard_gate_px - self.cfg.align_px)
                    r = max(0.0, min(1.0, r))
                    scale = 0.5 * (1.0 + math.cos(math.pi * r))  # 1 → 0 smoothly
                vx = raw_vx * scale

        return Setpoint(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, stamp=now)


# ============================ Backends (IO) =============================

class PrintBackend:
    """
    DRYRUN backend: does not talk to the flight controller.
    Rate-limits prints to ~SEND_HZ for readability.
    """
    def __init__(self, safety: SafetyConfig):
        self.cfg = safety
        self._last_send = 0.0

    def start(self) -> None:
        print("[Adapter] Print backend ready. (DRYRUN)")

    def stop(self) -> None:
        print("[Adapter] Print backend stopped.")

    def send(self, sp: Setpoint) -> None:
        now = time.time()
        min_dt = 1.0 / max(1e-6, self.cfg.SEND_HZ)
        if now - self._last_send < min_dt:
            return
        self._last_send = now
        print(
            f"[DRYRUN] vx={sp.vx:+.2f} m/s  vy={sp.vy:+.2f}  "
            f"vz={sp.vz:+.2f}  yaw_rate={sp.yaw_rate:+.2f} rad/s"
        )


class ArduPilotBackend:
    """
    ArduPilot flight backend.
    Sends SET_POSITION_TARGET_LOCAL_NED with MAV_FRAME_BODY_NED
    velocity components and yaw_rate.

    Requires: pip install pymavlink
    """
    def __init__(self, safety: SafetyConfig):
        from pymavlink import mavutil  # lazy import so module stays optional
        self.mavutil = mavutil
        self.cfg = safety
        self._m = None
        self._last_send = 0.0
        self._t0 = time.time()

    def start(self) -> None:
        print(f"[Adapter] Connecting ArduPilot at {self.cfg.serial}@{self.cfg.baud} …")
        self._m = self.mavutil.mavlink_connection(self.cfg.serial, baud=self.cfg.baud)
        self._m.wait_heartbeat(timeout=5)
        print(f"[Adapter] Heartbeat from system {self._m.target_system} component {self._m.target_component}")
        self._m.set_mode_apm("GUIDED")
        if self.cfg.ENABLE_FLIGHT and self.cfg.arm_when_enabled:
            self._m.arducopter_arm()
        print("[Adapter] GUIDED ready.")

    def stop(self) -> None:
        if self._m:
            try:
                self._send_raw(Setpoint(0.0, 0.0, 0.0, 0.0, time.time()))
            except Exception:
                pass
            try:
                self._m.close()
            except Exception:
                pass
        print("[Adapter] ArduPilot backend stopped.")

    def _send_raw(self, sp: Setpoint) -> None:
        """
        Low-level MAVLink send: BODY_NED velocity + yaw_rate.
        We ignore position and acceleration fields.
        """
        type_mask = (
            1 << 0 | 1 << 1 | 1 << 2 |  # ignore position x,y,z
            1 << 6 | 1 << 7 | 1 << 8 |  # ignore acceleration
            1 << 11                      # ignore yaw (we use yaw_rate)
        )
        # Enable velocity (bits 3,4,5) and yaw_rate (bit 10) by clearing their ignore flags
        type_mask &= ~((1 << 3) | (1 << 4) | (1 << 5) | (1 << 10))
        t_ms = int((time.time() - self._t0) * 1000) & 0xFFFFFFFF
        self._m.mav.set_position_target_local_ned_send(
            t_ms,                   # time_boot_ms
            self._m.target_system,
            self._m.target_component,
            self.mavutil.mavlink.MAV_FRAME_BODY_NED,  # velocities in body frame
            type_mask,
            0, 0, 0,                                  # position (ignored)
            sp.vx, sp.vy, sp.vz,                      # velocity (m/s)
            0, 0, 0,                                  # acceleration (ignored)
            0,                                        # yaw (ignored)
            sp.yaw_rate                               # yaw_rate (rad/s)
        )

    def send(self, sp: Setpoint) -> None:
        """
        Respect ENABLE_FLIGHT:
        - False: print like DRYRUN (visible but harmless).
        - True: actually send at ~SEND_HZ.
        """
        if not self.cfg.ENABLE_FLIGHT:
            print(
                f"[DRYRUN][Ardu] vx={sp.vx:+.2f} vy={sp.vy:+.2f} "
                f"vz={sp.vz:+.2f} yaw_rate={sp.yaw_rate:+.2f}"
            )
            return

        now = time.time()
        min_dt = 1.0 / max(1e-6, self.cfg.SEND_HZ)
        if now - self._last_send < min_dt:
            return
        self._last_send = now
        self._send_raw(sp)


class Px4MavlinkBackend:
    """
    PX4 flight backend using pymavlink.
    Sends SET_POSITION_TARGET_LOCAL_NED with MAV_FRAME_BODY_NED
    velocity components and yaw_rate.

    - You select mode (e.g. OFFBOARD / POSCTL) from RC/QGC.
    - ENABLE_FLIGHT=False → DRYRUN print only.
    """

    def __init__(self, safety: SafetyConfig):
        from pymavlink import mavutil  # lazy import
        self.mavutil = mavutil
        self.cfg = safety
        self._m = None
        self._last_send = 0.0
        self._t0 = time.time()

    def start(self) -> None:
        print(f"[Adapter] Connecting PX4 at {self.cfg.serial}@{self.cfg.baud} …")
        self._m = self.mavutil.mavlink_connection(self.cfg.serial, baud=self.cfg.baud)
        self._m.wait_heartbeat(timeout=5)
        print(f"[Adapter] Heartbeat from system {self._m.target_system} component {self._m.target_component}")
        print("[Adapter] PX4 backend ready. Select OFFBOARD / POSCTL via RC/QGC when you want to use velocities.")

    def stop(self) -> None:
        if self._m:
            try:
                self._send_raw(Setpoint(0.0, 0.0, 0.0, 0.0, time.time()))
            except Exception:
                pass
            try:
                self._m.close()
            except Exception:
                pass
        print("[Adapter] PX4 backend stopped.")

    def _send_raw(self, sp: Setpoint) -> None:
        """
        Low-level MAVLink send: BODY_NED velocity + yaw_rate.
        Works on PX4 when in a suitable mode (typically OFFBOARD).
        """
        type_mask = (
            1 << 0 | 1 << 1 | 1 << 2 |  # ignore position x,y,z
            1 << 6 | 1 << 7 | 1 << 8 |  # ignore acceleration
            1 << 11                      # ignore yaw
        )
        # Enable velocity (3,4,5) and yaw_rate (10)
        type_mask &= ~((1 << 3) | (1 << 4) | (1 << 5) | (1 << 10))

        t_ms = int((time.time() - self._t0) * 1000) & 0xFFFFFFFF
        print(f"[PX4_SEND] vx={sp.vx:+.2f}, vy={sp.vy:+.2f}, vz={sp.vz:+.2f}, yaw={sp.yaw_rate:+.2f}")

        self._m.mav.set_position_target_local_ned_send(
            t_ms,                                   # time_boot_ms (uint32)
            self._m.target_system,
            self._m.target_component,
            self.mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,                                # position (ignored)
            sp.vx, sp.vy, sp.vz,                    # velocity (m/s)
            0, 0, 0,                                # acceleration (ignored)
            0,                                      # yaw (ignored)
            sp.yaw_rate                             # yaw_rate (rad/s)
        )

    def send(self, sp: Setpoint) -> None:
        """
        ENABLE_FLIGHT:
          - False: DRYRUN print only (no MAVLink output).
          - True: actually send velocity setpoints at ~SEND_HZ.
        """
        if not self.cfg.ENABLE_FLIGHT:
            print(
                f"[DRYRUN][PX4] vx={sp.vx:+.2f} vy={sp.vy:+.2f} "
                f"vz={sp.vz:+.2f} yaw_rate={sp.yaw_rate:+.2f}"
            )
            return

        now = time.time()
        min_dt = 1.0 / max(1e-6, self.cfg.SEND_HZ)
        if now - self._last_send < min_dt:
            return
        self._last_send = now
        self._send_raw(sp)


# =============================== Facade ================================

class VelocityController:
    """
    Small facade you use from main.py:

      controller = VelocityController(..., backend="ardupilot")
      controller.start()

      sp = controller.compute_from_tracking(track_center, frame_center, lidar_m)
      controller.send(sp)  # DRYRUN unless ENABLE_FLIGHT=True

      controller.stop()

    - Computes setpoints from vision+LiDAR
    - Sends them to the configured backend
    - Keeps last setpoint so you can call send() even if compute() wasn’t run
    """

    def __init__(
        self,
        map_cfg: MapConfig = MapConfig(),
        safety: SafetyConfig = SafetyConfig(),
        backend: str = "print",
    ):
        self.mapper = GuidanceMapper(map_cfg)
        self.safety = safety

        b = backend.lower()
        if b == "ardupilot":
            self.backend = ArduPilotBackend(safety)
        elif b == "px4":
            self.backend = Px4MavlinkBackend(safety)
        else:
            self.backend = PrintBackend(safety)

        self._last: Optional[Setpoint] = None

    def start(self) -> None:
        """Initialize backend (open link, set GUIDED for Ardu, etc.)."""
        self.backend.start()

    def stop(self) -> None:
        """Cleanly stop backend (send zero once, close link)."""
        self.backend.stop()

    def compute_from_tracking(
        self,
        track_center: Optional[Tuple[int, int]],
        frame_center: Tuple[int, int],
        lidar_m: Optional[float],
    ) -> Setpoint:
        """
        Convert tracker outputs into dx/dy and compute a setpoint.
        - track_center: (cx, cy) or None if not visible
        - frame_center: (fx, fy)
        - lidar_m: distance in meters (or None)
        """
        if track_center is None:
            dx = dy = None
        else:
            (cx, cy), (fx, fy) = track_center, frame_center
            dx, dy = cx - fx, cy - fy

        sp = self.mapper.compute(dx, dy, lidar_m)
        self._last = sp
        return sp

    def send(self, sp: Optional[Setpoint] = None) -> None:
        """
        Send a setpoint to the backend (or the last one if not provided).
        If nothing exists yet, send a zero (harmless).
        """
        if sp is None:
            sp = self._last or Setpoint(0.0, 0.0, 0.0, 0.0, time.time())
        self.backend.send(sp)

