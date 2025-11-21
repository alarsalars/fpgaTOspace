# tracker.py
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import cv2
import numpy as np
import math
import time

@dataclass
class Track:
    track_id: int
    bbox: Tuple[int, int, int, int]   # x1,y1,x2,y2
    score: float
    cls: Optional[int] = None

# ---------------- utils ----------------
def _try_ctor(name: str):
    ctor_legacy = getattr(getattr(cv2, "legacy", object()), f"Tracker{name}_create", None)
    if callable(ctor_legacy):
        return ctor_legacy()
    ctor = getattr(cv2, f"Tracker{name}_create", None)
    if callable(ctor):
        return ctor()
    return None

def _create_tracker(prefer="MOSSE"):
    # Prefer user's choice then fallbacks
    order = [prefer.upper()] + [a for a in ("CSRT","KCF","MOSSE") if a.upper()!=prefer.upper()]
    for algo in order:
        t = _try_ctor(algo)
        if t is not None:
            return algo, t
    raise RuntimeError("OpenCV built without tracking APIs. Install opencv-contrib build.")

def _xyxy_to_xywh(b):
    x1,y1,x2,y2 = map(int, b)
    return (x1, y1, max(1, x2-x1), max(1, y2-y1))

def _xywh_to_xyxy(b):
    x,y,w,h = b
    return (int(x), int(y), int(x+w), int(y+h))

def _clip_box(x1,y1,x2,y2,w,h):
    x1 = max(0, min(x1, w-1)); y1 = max(0, min(y1, h-1))
    x2 = max(0, min(x2, w-1)); y2 = max(0, min(y2, h-1))
    if x2 <= x1: x2 = min(w-1, x1+2)
    if y2 <= y1: y2 = min(h-1, y1+2)
    return (x1,y1,x2,y2)

def _iou(a,b)->float:
    ax1,ay1,ax2,ay2=a; bx1,by1,bx2,by2=b
    ix1=max(ax1,bx1); iy1=max(ay1,by1)
    ix2=min(ax2,bx2); iy2=min(ay2,by2)
    iw=max(0,ix2-ix1); ih=max(0,iy2-iy1)
    inter=iw*ih
    if inter<=0: return 0.0
    areaA=max(0,ax2-ax1)*max(0,ay2-ay1)
    areaB=max(0,bx2-bx1)*max(0,by2-by1)
    return inter/(areaA+areaB-inter+1e-6)

def _inflate(b, scale=1.08, w=None, h=None):
    x1,y1,x2,y2 = b
    cx = (x1+x2)/2; cy=(y1+y2)/2
    bw = (x2-x1)*scale; bh=(y2-y1)*scale
    nx1 = int(cx - bw/2); ny1=int(cy - bh/2)
    nx2 = int(cx + bw/2); ny2=int(cy + bh/2)
    if w is not None and h is not None:
        return _clip_box(nx1,ny1,nx2,ny2,w,h)
    return (nx1,ny1,nx2,ny2)
def _blend_boxes(a, b, alpha=0.6):
    ax1,ay1,ax2,ay2 = a; bx1,by1,bx2,by2 = b
    w = 1.0 - alpha
    return (int(ax1*w + bx1*alpha),
            int(ay1*w + by1*alpha),
            int(ax2*w + bx2*alpha),
            int(ay2*w + by2*alpha))

def _limit_scale_step(prev, new, max_step=1.25):
    # prevent sudden growth/shrink > max_step
    px1,py1,px2,py2 = prev; nx1,ny1,nx2,ny2 = new
    pw = max(1, px2-px1); ph = max(1, py2-py1)
    nw = max(1, nx2-nx1); nh = max(1, ny2-ny1)
    sw = min(max(nw/pw, 1.0/max_step), max_step)
    sh = min(max(nh/ph, 1.0/max_step), max_step)
    # scale around previous center
    cx = (px1+px2)/2; cy=(py1+py2)/2
    tw = int(pw*sw); th=int(ph*sh)
    tx1 = int(cx - tw/2); ty1=int(cy - th/2)
    tx2 = tx1 + tw; ty2 = ty1 + th
    return (tx1,ty1,tx2,ty2)

class _SingleTracker:
    """
    Very fast single-object tracker with:
    - default MOSSE (switchable to CSRT/KCF)
    - periodic re-associate to YOLO
    - strict edge-leave drop to avoid frozen boxes
    - EMA smoothing
    """
    def __init__(
        self,
        prefer_algo: str = "MOSSE",
        prefer_class: Optional[int|str] = None,
        reinit_iou_thresh: float = 0.35,
        detect_every: int = 5,
        smooth: float = 0.35,
        min_area: int = 200,
        edge_margin: int = 18,
        stale_edge_frames: int = 6,
        refine_mode: str = "blend",    # "snap" | "blend" | "none"
        refine_alpha: float = 0.6,     # how strongly to pull toward detector box (for blend)
        inflate_scale: float = 1.03,   # slight inflate on init to avoid too-tight crops
        max_scale_step: float = 1.25   # limit sudden size jumps per frame
        ):
        self.prefer_class = None if prefer_class is None else str(prefer_class)
        self.reinit_iou = reinit_iou_thresh
        self.detect_every = max(1, int(detect_every))
        self.smooth = float(smooth)
        self.min_area = int(min_area)
        self.edge_margin = int(edge_margin)
        self.stale_edge_frames = int(stale_edge_frames)
        self.refine_mode = refine_mode.lower()
        self.refine_alpha = float(refine_alpha)
        self.inflate_scale = float(inflate_scale)
        self.max_scale_step = float(max_scale_step)
        self.algo, self.cv = _create_tracker(prefer_algo)
        self.active=False
        self.tid=1
        self.bbox=None   # xyxy
        self.score=0.0
        self.cls=None
        self.fidx=0
        self.last_center=None
        self.edge_stale_count=0

    def _best_det(self, dets: List[Dict]) -> Optional[Dict]:
        if not dets: return None
        pool = dets
        if self.prefer_class is not None:
            pool = [d for d in dets if str(d.get("cls"))==self.prefer_class] or dets
        return max(pool, key=lambda d: float(d.get("conf",0.0)))

    def _init(self, frame, det):
        h,w = frame.shape[:2]
        x1,y1,x2,y2 = det["bbox"]
        x1,y1,x2,y2 = _clip_box(int(x1),int(y1),int(x2),int(y2),w,h)
        if (x2-x1)*(y2-y1) < self.min_area:
            pad = 12
            x1=max(0,x1-pad); y1=max(0,y1-pad); x2=min(w-1,x2+pad); y2=min(h-1,y2+pad)
        # small inflate so the box covers the object better
        x1,y1,x2,y2 = _inflate((x1,y1,x2,y2), self.inflate_scale, w, h)

        self.algo, self.cv = _create_tracker(self.algo)  # re-create with same algo
        self.cv.init(frame, _xyxy_to_xywh((x1,y1,x2,y2)))
        self.bbox=(x1,y1,x2,y2)
        self.score=float(det.get("conf",0.0))
        self.cls = int(det["cls"]) if det.get("cls") is not None else None
        self.active=True
        self.last_center=((x1+x2)//2, (y1+y2)//2)
        self.edge_stale_count=0

    def _ema(self, old, new):
        if old is None: return new
        a=self.smooth
        ox1,oy1,ox2,oy2=old; nx1,ny1,nx2,ny2=new
        return (int(ox1*(1-a)+nx1*a),
                int(oy1*(1-a)+ny1*a),
                int(ox2*(1-a)+nx2*a),
                int(oy2*(1-a)+ny2*a))

    def _near_edge(self, b, w, h):
        x1,y1,x2,y2=b
        return (x1<=self.edge_margin or y1<=self.edge_margin or
                x2>=w-1-self.edge_margin or y2>=h-1-self.edge_margin)

    def _center(self, b):
        x1,y1,x2,y2=b
        return ((x1+x2)//2, (y1+y2)//2)

    def update(self, frame, detections: List[Dict]) -> List[Track]:
        self.fidx += 1
        h,w = frame.shape[:2]

        if not self.active:
            det = self._best_det(detections)
            if det is not None:
                self._init(frame, det)
            else:
                return []

        ok, box = self.cv.update(frame)
        if not ok:
            # lost -> drop immediately; reinit if fresh det available
            self.active=False
            self.bbox=None
            det = self._best_det(detections)
            if det is not None:
                self._init(frame, det)
                return [Track(self.tid, self.bbox, self.score, self.cls)]
            return []

        # clamp & size check
        new_xyxy = _clip_box(*_xywh_to_xyxy(box), w, h)
        if (new_xyxy[2]-new_xyxy[0])*(new_xyxy[3]-new_xyxy[1]) < self.min_area:
            self.active=False
            self.bbox=None
            return []

        # EMA smoothing
        self.bbox = self._ema(self.bbox, new_xyxy)

        # strict edge-leave handling: if near edge and no good detection association → drop fast
        near_edge = self._near_edge(self.bbox, w, h)
        reassociated = False
        if detections:
            # find best matching detection by IoU
            best = max(detections, key=lambda d: _iou(self.bbox, d['bbox']))
            best_iou = _iou(self.bbox, best['bbox'])
            # if they overlap reasonably, refine size/position
            if best_iou >= 0.15:  # low threshold to allow gentle corrections
                det_box = tuple(map(int, best['bbox']))
                if self.refine_mode == "snap":
                    # snap directly but limit extreme size jumps to avoid flicker
                    snapped = _limit_scale_step(self.bbox, det_box, self.max_scale_step)
                    self.bbox = self._ema(self.bbox, snapped)  # one more EMA to keep it smooth
                elif self.refine_mode == "blend":
                    # blend toward detector box (tightens without harsh jumps)
                    blended = _blend_boxes(self.bbox, det_box, self.refine_alpha)
                    blended = _limit_scale_step(self.bbox, blended, self.max_scale_step)
                    self.bbox = blended
                # "none" => keep tracker-only box

        if near_edge and not reassociated:
            self.edge_stale_count += 1
            if self.edge_stale_count >= self.stale_edge_frames:
                # consider it gone
                self.active=False
                self.bbox=None
                self.edge_stale_count=0
                return []
        else:
            self.edge_stale_count=0

        self.last_center = self._center(self.bbox)
        return [Track(self.tid, self.bbox, self.score, self.cls)]

def build_tracker(prefer: str="mosse", **kwargs):
    return _SingleTracker(
        prefer_algo=prefer,
        prefer_class=kwargs.get("prefer_class"),
        reinit_iou_thresh=kwargs.get("reinit_iou_thresh", 0.35),
        detect_every=kwargs.get("detect_every", 5),
        smooth=kwargs.get("smooth", 0.35),
        min_area=kwargs.get("min_area", 200),
        edge_margin=kwargs.get("edge_margin", 18),
        stale_edge_frames=kwargs.get("stale_edge_frames", 6),
        refine_mode=kwargs.get("refine_mode", "blend"),
        refine_alpha=kwargs.get("refine_alpha", 0.6),
        inflate_scale=kwargs.get("inflate_scale", 1.03),
        max_scale_step=kwargs.get("max_scale_step", 1.25),
    )
