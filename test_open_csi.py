import cv2, time

def gst_pipeline(sensor_id=0, w=1280, h=720, fps=30, flip=2):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={w},height={h},format=NV12,framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw,format=BGRx ! videoconvert ! "
        f"video/x-raw,format=BGR ! appsink drop=true sync=false"
    )

pipe = gst_pipeline()
cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
print("Opened:", cap.isOpened())
if not cap.isOpened():
    raise SystemExit("Failed to open CSI via GStreamer")

t0, frames = time.time(), 0
while True:
    ok, frame = cap.read()
    if not ok:
        print("No frame"); break
    frames += 1
    cv2.imshow("IMX219 (OpenCV+GStreamer)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

dt = time.time() - t0
print(f"Frames: {frames}, ~{frames/max(dt,1e-6):.1f} FPS")
cap.release()
cv2.destroyAllWindows()
