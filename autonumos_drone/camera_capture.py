import cv2
import time

class Camera:
    """
    Camera class to interface with a CSI camera using OpenCV + GStreamer.
    """
    def __init__(self, sensor_id=0, width=1280, height=720, fps=60, flip=2):
        self.sensor_id = sensor_id
        self.width = width
        self.height = height
        self.fps = fps
        self.flip = flip
        self.cap = None
        self._frame_count = 0
        self._start_time = None

    def _gstreamer_pipeline(self):
        return (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! "
            f"video/x-raw(memory:NVMM),width={self.width},height={self.height},format=NV12,framerate={self.fps}/1 ! "
            f"nvvidconv flip-method={self.flip} ! "
            f"video/x-raw,format=BGRx ! videoconvert ! "
            f"video/x-raw,format=BGR ! appsink drop=true sync=false"
        )

    def start(self):
        pipe = self._gstreamer_pipeline()
        self.cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("Failed to open CSI camera via GStreamer")

        self._start_time = time.time()
        self._frame_count = 0
        print("[Camera] Started successfully.")

    def read(self):
        if not self.cap:
            raise RuntimeError("Camera not started. Call start() first.")

        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("Camera frame read failed")

        self._frame_count += 1
        return frame

    def stop(self):
        if self.cap:
            self.cap.release()
            cv2.destroyAllWindows()
            print("[Camera] Stopped.")

            # Optional: Print average FPS
            dt = time.time() - self._start_time
            if dt > 0:
                fps = self._frame_count / dt
                print(f"[Camera] Average FPS: {fps:.2f}")

    def is_opened(self):
        return self.cap.isOpened() if self.cap else False
