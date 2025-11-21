# mavsdk_connect_test.py
import asyncio
from flight_mavsdk import connect_mavsdk, close_mavsdk, DEFAULT_SERIAL, DEFAULT_BAUD

async def main():
    sys = await connect_mavsdk(DEFAULT_SERIAL, DEFAULT_BAUD, timeout_s=12.0)
    print("[TEST] Link OK. No commands sent.")
    await close_mavsdk(sys)

if __name__ == "__main__":
    asyncio.run(main())
