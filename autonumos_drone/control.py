# control.py — simple debug guidance printer (no real flight commands)
from dataclasses import dataclass, field
import time
from typing import List, Optional, Tuple

@dataclass
class GuidanceConfig:
    deadband_px: int = 20          # center deadband in pixels for yaw/altitude
    hold_ms_on_missing: int = 400  # don't print anything if track disappears for less than this
    desired_m: float = 3.0         # target distance
    tol_enter_m: float = 0.25      # start moving when error exceeds this
    tol_exit_m: float = 0.15       # stop moving when back within this (hysteresis)
    rate_limit_hz: float = 10.0    # max print frequency
    combine_axes: bool = True      # print one combined line instead of many

@dataclass
class SimpleGuidance:
    cfg: GuidanceConfig = field(default_factory=GuidanceConfig)
    _last_seen_ts: float = field(default_factory=lambda: 0.0)
    _last_print_ts: float = field(default_factory=lambda: 0.0)
    _last_cmd: str = ""

    # internal hysteresis states
    _want_forward: bool = False
    _want_backward: bool = False

    def _rate_limited(self) -> bool:
        now = time.time()
        min_dt = 1.0 / max(1e-6, self.cfg.rate_limit_hz)
        return (now - self._last_print_ts) < min_dt

    def _print(self, text: str):
        # rate-limit and change-filter
        if self._rate_limited() and text == self._last_cmd:
            return
        print(text)
        self._last_cmd = text
        self._last_print_ts = time.time()

    def update(
        self,
        track_center: Optional[Tuple[int, int]],
        frame_center: Tuple[int, int],
        lidar_m: Optional[float]
    ):
        """
        Decide debug commands to center the object and hold ~3.0 m.
        Only prints (no real commands).
        Suppresses prints during brief (< hold_ms_on_missing) detection gaps.
        """
        now = time.time()
        cmds: List[str] = []

        # Handle brief missing target
        if track_center is None:
            # If we've seen something recently, suppress printing for a short time
            if (now - self._last_seen_ts) * 1000 < self.cfg.hold_ms_on_missing:
                return
            # Otherwise, emit a single "No target" line at low rate
            self._print("No target")
            return

        # We have a target
        self._last_seen_ts = now
        cx, cy = track_center
        fx, fy = frame_center
        dx = cx - fx
        dy = cy - fy

        # Yaw (horizontal) and Altitude (vertical) guidance with deadband
        if dx > self.cfg.deadband_px:
            cmds.append("Yaw Right")
        elif dx < -self.cfg.deadband_px:
            cmds.append("Yaw Left")

        if dy > self.cfg.deadband_px:
            cmds.append("Down")
        elif dy < -self.cfg.deadband_px:
            cmds.append("Up")

        # Distance hold (3.0 m) with hysteresis
        if lidar_m is not None and lidar_m > 0:
            err = lidar_m - self.cfg.desired_m
            # enter conditions
            if err > self.cfg.tol_enter_m:
                self._want_forward = True
                self._want_backward = False
            elif err < -self.cfg.tol_enter_m:
                self._want_backward = True
                self._want_forward = False
            # exit conditions (hysteresis)
            if abs(err) <= self.cfg.tol_exit_m:
                self._want_forward = False
                self._want_backward = False

            if self._want_forward:
                cmds.append("Forward")   # object too far → get closer
            elif self._want_backward:
                cmds.append("Backward")  # object too close → back off

        # Print combined or per-axis
        if not cmds:
            # If centered and in distance band, don't spam; tiny heartbeat
            self._print("Hold")
        else:
            if self.cfg.combine_axes:
                self._print(" | ".join(cmds))
            else:
                for c in cmds:
                    self._print(c)
