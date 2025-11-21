# telemetry_mavsdk.py
import asyncio
from flight_mavsdk import connect_mavsdk, close_mavsdk, DEFAULT_SERIAL, DEFAULT_BAUD


async def telemetry_loop(sys):
    """
    Subscribe to telemetry streams and periodically print a summary line.
    """

    state = {
        "mode": None,
        "pos": None,
        "gps": None,
        "bat": None,
        "armed": None,
    }

    async def sub_mode():
        async for fm in sys.telemetry.flight_mode():
            state["mode"] = fm

    async def sub_pos():
        async for p in sys.telemetry.position():
            state["pos"] = p

    async def sub_gps():
        async for g in sys.telemetry.gps_info():
            state["gps"] = g

    async def sub_bat():
        async for b in sys.telemetry.battery():
            state["bat"] = b

    async def sub_armed():
        async for a in sys.telemetry.armed():
            state["armed"] = a

    tasks = [
        asyncio.create_task(sub_mode()),
        asyncio.create_task(sub_pos()),
        asyncio.create_task(sub_gps()),
        asyncio.create_task(sub_bat()),
        asyncio.create_task(sub_armed()),
    ]

    try:
        while True:
            mode = state["mode"]
            pos = state["pos"]
            gps = state["gps"]
            bat = state["bat"]
            armed = state["armed"]

            mode_str = mode.name if mode is not None else "UNKNOWN"
            sats_str = f"{gps.num_satellites:2d}" if gps is not None else "??"
            alt_str = f"{pos.relative_altitude_m:5.1f}" if pos is not None else "  ?.?"
            bat_str = f"{bat.remaining_percent*100:3.0f}%" if bat is not None else " ??%"
            armed_str = str(armed) if armed is not None else "None"

            line = (
                f"MODE={mode_str:>10} | "
                f"SATS={sats_str} | "
                f"ALT={alt_str} m | "
                f"BAT={bat_str} | "
                f"ARMED={armed_str}"
            )
            print(line)

            await asyncio.sleep(0.5)
    finally:
        for t in tasks:
            t.cancel()
        await close_mavsdk(sys)


async def main():
    sys = await connect_mavsdk(DEFAULT_SERIAL, DEFAULT_BAUD, timeout_s=12.0)
    print("[Telemetry] Link OK, starting telemetry loop…")
    await telemetry_loop(sys)


if __name__ == "__main__":
    asyncio.run(main())
