#!/usr/bin/env python3
"""
T265 + YOLO11 灰度模型实时检测 V7
参照 CSDN 文章 V7 脚本风格重写，适配：
- T265 鱼眼 #1 灰度输入 (848×800 Y8)
- 灰度模型 NCHW (1,1,800,864)
- 6 输出: bbox_P3/cls_P3/bbox_P4/cls_P4/bbox_P5/cls_P5
- DFL 解码 + 动态输出头匹配 + 面积过滤
"""
import os, sys, time
import numpy as np
import cv2
import pyrealsense2 as rs

sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages/hobot_dnn')
import pyeasy_dnn

# ── 配置 ──────────────────────────────────────────
MODEL_DIR = "/home/sunrise/inter t265/model"
LABELS_PATH = os.path.join(MODEL_DIR, "labels.names")
INPUT_H, INPUT_W = 800, 864          # 模型输入尺寸
T265_W, T265_H = 848, 800            # T265 鱼眼原始尺寸
CONF_THRESH = 0.25
NMS_THRESH = 0.50
REG_MAX = 16
MIN_AREA = 100                       # 最小检测框面积（过滤噪点）

# ── 自动找模型文件 ──────────────────────────────
MODEL_PATH = None
for f in sorted(os.listdir(MODEL_DIR)):
    if 'my_t265' in f and f.endswith('.bin'):
        MODEL_PATH = os.path.join(MODEL_DIR, f)
        break
if MODEL_PATH is None:
    print("ERROR: 未找到 my_t265 模型文件", file=sys.stderr)
    sys.exit(1)

with open(LABELS_PATH, "r") as f:
    CLASS_NAMES = [line.strip() for line in f if line.strip()]
NUM_CLASSES = len(CLASS_NAMES)
print(f"模型: {MODEL_PATH}")
print(f"类别: {CLASS_NAMES} ({NUM_CLASSES} 类)")
print(f"输入: {INPUT_W}×{INPUT_H} 灰度 NCHW")

# ── 工具函数 ─────────────────────────────────────
def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))

def softmax_last(x):
    e = np.exp(x - np.max(x, axis=-1, keepdims=True))
    return e / np.sum(e, axis=-1, keepdims=True)

# ── 检测器类 ─────────────────────────────────────
class T265YoloDetector:
    def __init__(self):
        models = pyeasy_dnn.load(MODEL_PATH)
        self.model = models[0]
        self.conf_thresh = CONF_THRESH
        self.nms_thresh = NMS_THRESH
        self.reg_max = REG_MAX
        self._parse_heads()

    def _parse_heads(self):
        """按 shape 动态匹配 bbox(64ch) 和 cls(nc ch) 输出"""
        bbox_map, cls_map = {}, {}
        for idx, out in enumerate(self.model.outputs):
            shp = list(out.properties.shape)
            if len(shp) != 4:
                continue
            if shp[1] == 64:                          # NCHW bbox
                bbox_map[(shp[2], shp[3])] = (idx, "NCHW")
            elif shp[1] in (1, NUM_CLASSES):          # NCHW cls
                cls_map[(shp[2], shp[3])] = (idx, "NCHW")

        self.heads = []
        for hw, (b_idx, b_fmt) in sorted(bbox_map.items(), key=lambda x: -x[0][0]):
            if hw not in cls_map:
                continue
            c_idx, c_fmt = cls_map[hw]
            stride = INPUT_H // hw[0]
            H, W = hw
            gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
            grid = np.stack([gx, gy], axis=-1).reshape(-1, 2).astype(np.float32)
            self.heads.append({
                "bbox_idx": b_idx, "cls_idx": c_idx,
                "hw": hw, "stride": stride,
                "grid": grid,
                "bbox_fmt": b_fmt, "cls_fmt": c_fmt,
            })
        print(f"检测头: {[(h['stride'], h['hw']) for h in self.heads]}")

    def _extract_feat(self, outs, idx, fmt, H, W, C):
        """从输出中提取特征并展平为 (H*W, C)"""
        buf = np.array(outs[idx].buffer, copy=False).astype(np.float32)
        if fmt == "NCHW":
            return buf.reshape(1, C, H, W).transpose(0, 2, 3, 1).reshape(-1, C)
        else:
            return buf.reshape(-1, C)

    def _dfl_decode(self, bbox_raw):
        """DFL: (N, 64) → (N, 4)  ltrb 偏移"""
        bbox = bbox_raw.reshape(-1, 4, self.reg_max)
        bbox_exp = np.exp(bbox - np.max(bbox, axis=-1, keepdims=True))
        bbox_sm = bbox_exp / np.sum(bbox_exp, axis=-1, keepdims=True)
        weights = np.arange(self.reg_max, dtype=np.float32).reshape(1, 1, -1)
        return np.sum(bbox_sm * weights, axis=-1)

    def preprocess(self, gray):
        """灰度图 letterbox → NCHW (1,1,800,864) uint8"""
        h, w = gray.shape
        scale = min(INPUT_W / w, INPUT_H / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(gray, (nw, nh), interpolation=cv2.INTER_LINEAR)
        padded = np.full((INPUT_H, INPUT_W), 114, dtype=np.uint8)
        dx, dy = (INPUT_W - nw) // 2, (INPUT_H - nh) // 2
        padded[dy:dy + nh, dx:dx + nw] = resized
        return padded.reshape(1, 1, INPUT_H, INPUT_W), scale, dx, dy

    def detect(self, gray):
        """推理 + 后处理，返回 (boxes, scores, classes)"""
        orig_h, orig_w = gray.shape
        inp, scale, pad_x, pad_y = self.preprocess(gray)
        outs = self.model.forward(inp)

        all_boxes, all_scores, all_cls = [], [], []

        for h in self.heads:
            H, W = h["hw"]
            stride = h["stride"]
            grid = h["grid"]

            bbox_feat = self._extract_feat(outs, h["bbox_idx"], h["bbox_fmt"], H, W, 64)
            cls_feat  = self._extract_feat(outs, h["cls_idx"],  h["cls_fmt"],  H, W, NUM_CLASSES)

            scores = sigmoid(cls_feat)
            max_scores = np.max(scores, axis=1)
            keep = max_scores >= self.conf_thresh
            if not np.any(keep):
                continue

            max_cls   = np.argmax(scores, axis=1)
            bbox_keep = bbox_feat[keep]
            score_keep = max_scores[keep]
            cls_keep   = max_cls[keep]
            grid_keep  = grid[keep]

            ltrb = self._dfl_decode(bbox_keep)

            xc = (grid_keep[:, 0] + 0.5) * stride
            yc = (grid_keep[:, 1] + 0.5) * stride

            x1 = np.clip((xc - ltrb[:, 0] * stride - pad_x) / scale, 0, orig_w)
            y1 = np.clip((yc - ltrb[:, 1] * stride - pad_y) / scale, 0, orig_h)
            x2 = np.clip((xc + ltrb[:, 2] * stride - pad_x) / scale, 0, orig_w)
            y2 = np.clip((yc + ltrb[:, 3] * stride - pad_y) / scale, 0, orig_h)

            # 面积过滤
            areas = (x2 - x1) * (y2 - y1)
            valid = areas > MIN_AREA
            if not np.any(valid):
                continue

            boxes = np.stack([x1[valid], y1[valid], x2[valid], y2[valid]], axis=1)
            all_boxes.append(boxes)
            all_scores.append(score_keep[valid])
            all_cls.append(cls_keep[valid])

        if not all_boxes:
            return np.empty((0,4), np.float32), np.empty(0, np.float32), np.empty(0, np.int32)

        boxes   = np.concatenate(all_boxes)
        scores  = np.concatenate(all_scores)
        classes = np.concatenate(all_cls)

        idxs = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), self.conf_thresh, self.nms_thresh)
        if len(idxs) > 0:
            idxs = np.array(idxs).flatten()
            return boxes[idxs], scores[idxs], classes[idxs].astype(np.int32)
        return np.empty((0,4), np.float32), np.empty(0, np.float32), np.empty(0, np.int32)

    def draw(self, gray, boxes, scores, classes):
        """灰度 → BGR 并绘制检测框"""
        disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for box, score, cls_id in zip(boxes, scores, classes):
            x1, y1, x2, y2 = map(int, box)
            name = CLASS_NAMES[cls_id] if cls_id < NUM_CLASSES else str(cls_id)
            label = f"{name}: {score:.2f}"
            color = (0, 255, 0) if cls_id == 0 else (0, 0, 255)
            cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(disp, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(disp, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return disp

# ── 主循环 ────────────────────────────────────────
def main():
    det = T265YoloDetector()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.fisheye, 1, T265_W, T265_H, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.fisheye, 2, T265_W, T265_H, rs.format.y8, 30)
    pipe.start(cfg)

    fc = 0
    fps_list = []
    print("T265 YOLO V7 实时检测已启动 | 按 q 退出 | +/- 调阈值", flush=True)

    try:
        while True:
            frames = pipe.wait_for_frames()
            gray = np.asanyarray(frames.get_fisheye_frame(1).get_data())

            t0 = time.time()
            boxes, scores, classes = det.detect(gray)
            dt = time.time() - t0

            disp = det.draw(gray, boxes, scores, classes)

            fps_list.append(1.0 / max(dt, 1e-6))
            if len(fps_list) > 30:
                fps_list.pop(0)
            fps = np.mean(fps_list)

            cv2.putText(disp, f"FPS:{fps:.1f} Det:{len(boxes)} Thr:{det.conf_thresh:.2f}",
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("T265 YOLO V7", disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key in (ord('+'), ord('=')):
                det.conf_thresh = min(0.95, det.conf_thresh + 0.05)
                print(f"阈值: {det.conf_thresh:.2f}", flush=True)
            elif key == ord('-'):
                det.conf_thresh = max(0.05, det.conf_thresh - 0.05)
                print(f"阈值: {det.conf_thresh:.2f}", flush=True)

            fc += 1
            if fc % 100 == 0:
                print(f"Frame {fc}: FPS={fps:.1f}, Det={len(boxes)}", flush=True)

    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        print(f"已退出, 共 {fc} 帧", flush=True)

if __name__ == "__main__":
    main()
