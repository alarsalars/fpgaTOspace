# udp_stream.py
import cv2

def create_udp_writer(
    host="192.168.1.107",
    port=5600,
    width=640,
    height=360,
    fps=30,
):
    gst = (
        f"appsrc ! "
        f"video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
        f"videoconvert ! "
        f"video/x-raw,format=I420 ! "
        f"x264enc tune=zerolatency speed-preset=superfast bitrate=4000 "
        f"key-int-max=30 bframes=0 threads=4 byte-stream=true ! "
        f"h264parse config-interval=-1 ! "
        f"mpegtsmux ! "
        f"udpsink host={host} port={port} sync=false async=false"
    )

    fourcc = cv2.VideoWriter_fourcc(*"H264")  # OpenCV + GStreamer
    out = cv2.VideoWriter(gst, fourcc, fps, (width, height), isColor=True)

    if not out.isOpened():
        raise RuntimeError("Failed to open UDP GStreamer writer")

    print(f"[UDP] Streaming to udp://{host}:{port} at {width}x{height}@{fps}")
    return out
