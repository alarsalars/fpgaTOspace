# detection_yolo8.py
import cv2
import torch
from ultralytics import YOLO

class Detector:
    def __init__(self, model_path="yolov8s.pt", target_class="person", conf_thres=0.4, imgsz=480):
        self.model = YOLO(model_path)
        self.target_class = target_class
        self.conf_thres = conf_thres
        self.imgsz = imgsz
        self.use_gpu = torch.cuda.is_available()
        # NOTE: keep the call light; we'll invoke it every N frames from main

    def infer(self, frame):
        # run a single forward with filtering; return top few detections
        res = self.model(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_thres,
            verbose=False,
            device=0 if self.use_gpu else "cpu",
            half=self.use_gpu,         # FP16 on GPU
            agnostic_nms=False
        )[0]

        detections = []
        h, w, _ = frame.shape
        names = res.names

        for box in res.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            if conf < self.conf_thres:
                continue

            name = names.get(cls_id, str(cls_id))
            # class filter (either name or id matches target_class)
            if self.target_class is not None and str(self.target_class) != name and str(self.target_class) != str(cls_id):
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x1 = max(0, min(x1, w-1)); y1 = max(0, min(y1, h-1))
            x2 = max(0, min(x2, w-1)); y2 = max(0, min(y2, h-1))
            if x2 <= x1 or y2 <= y1:
                continue
            cx = (x1+x2)//2; cy=(y1+y2)//2

            detections.append({
                'bbox': (x1,y1,x2,y2),
                'center': (cx,cy),
                'conf': conf,
                'cls': cls_id
            })

        detections.sort(key=lambda d: d['conf'], reverse=True)
        return detections[:5], 0, 0
