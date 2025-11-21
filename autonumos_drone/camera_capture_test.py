from camera_capture import Camera
import cv2

cam = Camera()
cam.start()

try:
    while True:
        frame = cam.read()
        cv2.imshow("CSI Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cam.stop()
