# px4_mavsdk_telemetry.py
import asyncio
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from mavsdk import System


@dataclass
class TelemetryState:
    mode: str = "N/A"
    sats: int = 0
    alt_m: float = 0.0
    bat_pct: int = -1
    armed: bool = False
    last_update: float = field(default_factory=lambda: 0.0)


class Px4TelemetryClient:
    """
    Background MAVSDK telemetry client:
      - Connects over serial (e.g. /dev/ttyTHS1 @ 57600)
      - Subscribes to flight_mode, gps_info, position, battery, armed
      - Exposes a simple TelemetryState via get_state()
    """

    def __init__(self, serial: str = "/dev/ttyTHS1", baud: int = 57600):
        self._serial = serial
        self._baud = baud

        self._state = TelemetryState()
        self._lock = threading.Lock()

        self._thread: Optional[threading.Thread] = None
        self._stop_flag = False

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._system: Optional[System] = None
        self._offboard_started: bool = False

    # ---------------------- public API ----------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_flag = False
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print("[MAVSDK-TEL] Telemetry client thread started.")

    def stop(self) -> None:
        self._stop_flag = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        print("[MAVSDK-TEL] Telemetry client stopped.")

    def get_state(self) -> TelemetryState:
        with self._lock:
            # Return a shallow copy so caller can't corrupt internal state
            return TelemetryState(
                mode=self._state.mode,
                sats=self._state.sats,
                alt_m=self._state.alt_m,
                bat_pct=self._state.bat_pct,
                armed=self._state.armed,
                last_update=self._state.last_update,
            )

    # ---------------------- Offboard control (public API) ----------------------

    def ensure_offboard_started(self):
        """
        Thread-safe wrapper to ensure Offboard is started.
        Can be called from main thread.
        """
        if self._loop is None:
            print("[MAVSDK-TEL] Offboard start requested but loop is None.")
            return

        asyncio.run_coroutine_threadsafe(
            self._ensure_offboard_started(),
            self._loop,
        )

    def send_offboard_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """
        Thread-safe wrapper to send a body-frame velocity setpoint.
        Can be called from main thread.
        """
        if self._loop is None:
            # Not ready yet; ignore
            return

        asyncio.run_coroutine_threadsafe(
            self._offboard_set_velocity(vx, vy, vz, yaw_rate),
            self._loop,
        )

    def stop_offboard(self):
        """
        Thread-safe wrapper to stop Offboard mode.
        """
        if self._loop is None:
            return

        asyncio.run_coroutine_threadsafe(
            self._offboard_stop(),
            self._loop,
        )

    # ---------------------- internal async loop ----------------------

    def _run_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop   # <-- add this
        try:
            loop.run_until_complete(self._runner())
        except Exception as e:
            print("[MAVSDK-TEL] ERROR in telemetry loop:", e)
        finally:
            loop.run_until_complete(self._shutdown(loop))
            loop.close()

    async def _shutdown(self, loop: asyncio.AbstractEventLoop):
        # Nothing special for now; placeholder if we need cleanup
        await asyncio.sleep(0)

    async def _runner(self):
        system = System()
        url = f"serial:///{self._serial}:{self._baud}"
        print(f"[MAVSDK-TEL] Connecting: {url}")
        await system.connect(system_address=url)

        # Wait until connected
        async for state in system.core.connection_state():
            if state.is_connected:
                print("[MAVSDK-TEL] Connected to PX4.")
                break
        self._system = system

        # Start separate tasks for different telemetry streams
        tasks = [
            asyncio.create_task(self._flight_mode_loop(system)),
            asyncio.create_task(self._gps_loop(system)),
            asyncio.create_task(self._position_loop(system)),
            asyncio.create_task(self._battery_loop(system)),
            asyncio.create_task(self._armed_loop(system)),
        ]

        # Run until stop_flag set
        while not self._stop_flag:
            await asyncio.sleep(0.1)

        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    # ---------------------- individual streams ----------------------

    async def _flight_mode_loop(self, system: System):
        async for fm in system.telemetry.flight_mode():
            with self._lock:
                self._state.mode = fm.name
                self._state.last_update = time.time()
            if self._stop_flag:
                return

    async def _gps_loop(self, system: System):
        async for gi in system.telemetry.gps_info():
            with self._lock:
                self._state.sats = gi.num_satellites
                self._state.last_update = time.time()
            if self._stop_flag:
                return

    async def _position_loop(self, system: System):
        async for pos in system.telemetry.position():
            # Use relative altitude if available; otherwise use absolute
            alt = pos.relative_altitude_m if pos.relative_altitude_m is not None else pos.absolute_altitude_m
            with self._lock:
                self._state.alt_m = float(alt) if alt is not None else 0.0
                self._state.last_update = time.time()
            if self._stop_flag:
                return

    async def _battery_loop(self, system: System):
        async for bat in system.telemetry.battery():
            pct = int(bat.remaining_percent * 100.0) if bat.remaining_percent is not None else -1
            with self._lock:
                self._state.bat_pct = pct
                self._state.last_update = time.time()
            if self._stop_flag:
                return

    async def _armed_loop(self, system: System):
        async for armed in system.telemetry.armed():
            with self._lock:
                self._state.armed = bool(armed)
                self._state.last_update = time.time()
            if self._stop_flag:
                return
    # ---------------------- Offboard control (async helpers) ----------------------

    async def _ensure_offboard_started(self):
        """
        Ensure Offboard mode is started on PX4.
        Sends a few zero-velocity setpoints first (required by PX4),
        then calls offboard.start().
        """
        if self._system is None:
            print("[MAVSDK-TEL] Cannot start offboard: system is None")
            return

        if self._offboard_started:
            return

        print("[MAVSDK-TEL] Preparing Offboard: sending zero-velocity setpoints...")
        # PX4 requires a stream of setpoints before starting offboard
        try:
            for _ in range(5):
                await self._system.offboard.set_velocity_body(
                    vx_m_s=0.0,
                    vy_m_s=0.0,
                    vz_m_s=0.0,
                    yaw_rate_rad_s=0.0,
                )
                await asyncio.sleep(0.1)

            print("[MAVSDK-TEL] Starting Offboard mode...")
            await self._system.offboard.start()
            self._offboard_started = True
            print("[MAVSDK-TEL] Offboard started.")
        except Exception as e:
            print("[MAVSDK-TEL] Failed to start Offboard:", e)

    async def _offboard_set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """
        Send a single body-frame velocity setpoint.
        """
        if self._system is None:
            # No connection yet
            return

        try:
            await self._system.offboard.set_velocity_body(
                vx_m_s=vx,
                vy_m_s=vy,
                vz_m_s=vz,
                yaw_rate_rad_s=yaw_rate,
            )
        except Exception as e:
            print("[MAVSDK-TEL] Error sending offboard velocity:", e)

    async def _offboard_stop(self):
        """
        Stop Offboard mode on PX4.
        """
        if self._system is None:
            return

        if not self._offboard_started:
            return

        try:
            print("[MAVSDK-TEL] Stopping Offboard...")
            await self._system.offboard.stop()
        except Exception as e:
            print("[MAVSDK-TEL] Error stopping Offboard:", e)
        finally:
            self._offboard_started = False
