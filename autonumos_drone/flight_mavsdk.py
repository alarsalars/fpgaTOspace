# flight_mavsdk.py
import asyncio
from mavsdk import System
from mavsdk.telemetry import Health

DEFAULT_SERIAL = "/dev/ttyTHS1"
DEFAULT_BAUD = 57600

async def connect_mavsdk(serial_port: str = DEFAULT_SERIAL, baud: int = DEFAULT_BAUD,
                         timeout_s: float = 12.0) -> System:
    """
    Connect to autopilot via MAVSDK over serial and wait until basic telemetry is flowing.
    Tolerates missing/slow Info plugin responses on ArduPilot by not hard-requiring get_version().
    """
    system = System()
    system_address = f"serial:///{serial_port}:{baud}"
    print(f"[MAVSDK] Connecting: {system_address}")
    await system.connect(system_address=system_address)

    # Wait for is_connected True
    async for state in system.core.connection_state():
        if state.is_connected:
            print("[MAVSDK] Connected (is_connected=True).")
            break

    # Don’t block on Info plugin; instead try, but continue on failure.
    try:
        await asyncio.wait_for(_ensure_basic_telemetry(system), timeout=timeout_s)
    except asyncio.TimeoutError:
        print("[MAVSDK] Warning: basic telemetry not confirmed within timeout; continuing anyway.")

    return system

async def _ensure_basic_telemetry(system: System):
    """
    Try to read a few telemetry items; all are best-effort.
    """
    # Best-effort version (may fail on ArduPilot immediately after connect)
    try:
        version = await system.info.get_version()
        print(f"[MAVSDK] FW: {version.flight_sw_major}.{version.flight_sw_minor}.{version.flight_sw_patch}")
    except Exception as e:
        print(f"[MAVSDK] Info.get_version not ready yet ({e}); skipping.")

    # Health snapshot; will succeed once streams are up
    try:
        health: Health = await system.telemetry.health()
        print(f"[MAVSDK] Health:"
              f" gyro={health.gyrometer_calibration_ok}, accel={health.accelerometer_calibration_ok},"
              f" mag={health.magnetometer_calibration_ok}, local_pos={health.local_position_ok},"
              f" global_pos={health.global_position_ok}")
    except Exception as e:
        print(f"[MAVSDK] Telemetry.health not ready yet ({e}); skipping.")

    # Flight mode snapshot
    try:
        fm = await system.telemetry.flight_mode()
        print(f"[MAVSDK] Flight mode: {fm.name}")
    except Exception as e:
        print(f"[MAVSDK] Telemetry.flight_mode not ready yet ({e}); skipping.")

async def close_mavsdk(system: System):
    try:
        await system.close()
    except Exception:
        pass
