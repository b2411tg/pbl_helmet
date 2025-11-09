from rknnlite.api import RKNNLite
import numpy as np
import cv2, time, os
import queue

SIZE = 320
RKNN_MODEL = 'yolo/best_rk3588_320.rknn'
VIDEO_IN   = 'backwards_320.mp4'   # ★動画ファイルのパス

CONF_THRES = 0.8
IOU_THRES  = 0.5
TOPK_PER_CLASS = 300
CLASS_NAMES = ["stop_sign", "stop_road", "intersection", "backwards", "forwards", "school"]

def put_latest(q: queue.Queue, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)

class DetectStopAndBackwards:
    def __init__(self, shared, in_queue, out_queue):
        self.shared = shared
        self.in_queue = in_queue
        self.out_queue = out_queue
        self.detect_stop_flag = False
        
    def _xywh_to_xyxy(self, xywh):
        cx, cy, w, h = xywh.T
        return np.stack([cx - w/2, cy - h/2, cx + w/2, cy + h/2], axis=1)

    def _nms(self, boxes, scores, iou_thres):
        if boxes.size == 0: return []
        x1,y1,x2,y2 = boxes.T
        areas = (x2-x1).clip(0)*(y2-y1).clip(0)
        order = scores.argsort()[::-1]
        keep = []
        while order.size:
            i = order[0]; keep.append(i)
            if order.size == 1: break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = (xx2-xx1).clip(0); h = (yy2-yy1).clip(0)
            inter = w*h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
            order = order[np.where(iou <= iou_thres)[0] + 1]
        return keep

    def _standardize_output(self, outputs):
        a = np.asarray(outputs[0])
        if a.ndim == 3 and a.shape[0] == 1: a = a[0]
        if a.shape[0] < a.shape[1]: a = a.T
        return a

    def _postprocess(self, outputs, conf_thres=0.25, iou_thres=0.45, size_px=320, topk_per_class=300):
        preds = self._standardize_output(outputs).astype(np.float32)
        if preds.ndim != 2 or preds.shape[1] < 5: return []
        xywh = preds[:, :4]
        probs = preds[:, 4:]
        cls_id = np.argmax(probs, axis=1)
        score  = probs[np.arange(probs.shape[0]), cls_id]

        if float(np.max(np.abs(xywh))) <= 2.0:
            xywh *= float(size_px)

        keep = (score >= conf_thres) & (xywh[:,2] >= 2.0) & (xywh[:,3] >= 2.0)
        if not np.any(keep): return []
        xywh, score, cls_id = xywh[keep], score[keep], cls_id[keep]

        results = []
        for c in np.unique(cls_id):
            idx = np.where(cls_id == c)[0]
            if idx.size == 0: continue
            if idx.size > topk_per_class:
                topk_idx = idx[np.argpartition(score[idx], -topk_per_class)[-topk_per_class:]]
            else:
                topk_idx = idx

            boxes_c = self._xywh_to_xyxy(xywh[topk_idx])
            boxes_c[:,[0,2]] = boxes_c[:,[0,2]].clip(0, size_px)
            boxes_c[:,[1,3]] = boxes_c[:,[1,3]].clip(0, size_px)
            scores_c = score[topk_idx]

            kept_local = self._nms(boxes_c, scores_c, iou_thres)
            for k in kept_local:
                i = topk_idx[k]
                results.append((boxes_c[k], float(scores_c[k]), int(c)))
        return results

    def main(self):
        cv2.setNumThreads(1)

        # === RKNN ===
        rknn = RKNNLite(verbose=False)
        assert rknn.load_rknn(RKNN_MODEL) == 0
        init_ok = False
        try:
            core_mask = getattr(RKNNLite, 'NPU_CORE_0_1_2', None)
            if core_mask is not None:
                init_ok = (rknn.init_runtime(core_mask=core_mask) == 0)
        except Exception:
            init_ok = False
        if not init_ok:
            assert rknn.init_runtime() == 0

        first = self.in_queue.get(timeout=1)
        in_h, in_w = first.shape[:2]
        sx, sy = in_w / SIZE, in_h / SIZE

        # 固定バッファ
        inp_buf = np.empty((1, SIZE, SIZE, 3), dtype=np.uint8)
        tmp_resize = np.empty((SIZE, SIZE, 3), dtype=np.uint8)

        t0 = time.perf_counter()
        frames = 0
        font = cv2.FONT_HERSHEY_SIMPLEX

        while True:
            if self.in_queue.empty():
                continue         
            frame_bgr = self.in_queue.get(timeout=1)
            frames += 1

            # 前処理（ゼロコピー風）
            cv2.resize(frame_bgr, (SIZE, SIZE), dst=tmp_resize, interpolation=cv2.INTER_LINEAR)
            cv2.cvtColor(tmp_resize, cv2.COLOR_BGR2RGB, dst=inp_buf[0])

            # 推論
            outs = rknn.inference(inputs=[inp_buf], data_format="nhwc")

            # 後処理
            for box, sc, ci in self._postprocess(outs, CONF_THRES, IOU_THRES, SIZE, TOPK_PER_CLASS):
                x1,y1,x2,y2 = box
                X1, Y1, X2, Y2 = int(x1*sx), int(y1*sy), int(x2*sx), int(y2*sy)
                cv2.rectangle(frame_bgr, (X1, Y1), (X2, Y2), (0,255,0), 1)
                name = CLASS_NAMES[ci] if 0 <= ci < len(CLASS_NAMES) else str(ci)
                cv2.putText(frame_bgr, f'{name}:{sc:.2f}', (X1, max(12, Y1-4)), font, 0.5, (0,255,0), 1, cv2.LINE_AA)

                # 交差点30m以内検知で一時停止を検出した時にﾌﾗｸﾞをｾｯﾄしてGPSﾀｽｸに通知する
                if not self.shared.detect_stop.is_set() and self.shared.detect_intersection_30m.is_set() and (name=="stop_sign" or name=="stop_road"):
                    self.shared.detect_stop.set()
                elif not self.shared.detect_intersection_30m.is_set():
                    self.shared.detect_stop.clear()
                
            # FPS 表示
            dt = time.perf_counter() - t0
            fps_now = frames / dt if dt > 0 else 0.0
            cv2.putText(frame_bgr, f'FPS:{fps_now:.1f}', (8, 24), font, 0.7, (0,255,0), 2)

            put_latest(self.out_queue, frame_bgr)
