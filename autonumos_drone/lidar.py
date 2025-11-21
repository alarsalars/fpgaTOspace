from smbus2 import SMBus
import time

class TFLuna:
    """
    TF-Luna LiDAR reader using I2C (register-polling mode only).
    Returns only distance in meters.
    """

    def __init__(self, bus_id=7, addr=0x10, delay=0.03):
        self.bus_id = bus_id
        self.addr = addr
        self.delay = delay
        self.nbytes = 9
        self.bus = SMBus(bus_id)

    def _read_frame(self):
        """Reads 9-byte I2C frame from TF-Luna."""
        try:
            return self.bus.read_i2c_block_data(self.addr, 0x00, self.nbytes)
        except Exception as e:
            print("[TF-Luna] I2C read error:", e)
            return None

    def _parse_distance(self, buf):
        """Parses distance in meters from 9-byte frame."""
        if len(buf) != 9:
            return None
        checksum = sum(buf[:8]) & 0xFF
        if buf[8] != 0 and checksum != buf[8]:
            return None

        dist_cm = (buf[1] << 8) | buf[0]
        return dist_cm / 100.0  # meters

    def read_distance_m(self):
        """Returns distance in meters (float), or None on failure."""
        buf = self._read_frame()
        if not buf:
            return None
        return self._parse_distance(buf)

    def close(self):
        self.bus.close()

    def run_test(self):
        """Simple test loop to print distance."""
        try:
            count = 0
            while True:
                d = self.read_distance_m()
                if d is not None:
                    count += 1
                    print(f"[{count:05d}] Distance: {d:.2f} m")
                else:
                    print("TF-Luna read failed")
                time.sleep(self.delay)
        except KeyboardInterrupt:
            print("\n[TF-Luna] Test stopped.")
        finally:
            self.close()


if __name__ == "__main__":
    lidar = TFLuna()
    lidar.run_test()
