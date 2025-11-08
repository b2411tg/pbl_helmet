# infer_video_rknn_seg_ema.py
# ROCK 5C (RK3588) + RKNNLite で STDC2 セグメンテーションをリアル表示
# 変更点：フレーム間のチラつきを抑えるために「時系列EMA（指数移動平均）」のみ適用

from rknnlite.api import RKNNLite
import numpy as np
import cv2
import queue

# ===== 設定 =====
#RKNN_MODEL = "segmentation/seg_STDC_268_cityscapes_192x384_fp32.rknn"   # モデル名
#H, W = 192, 384          # モデル入力（height, width）※32の倍数推奨
RKNN_MODEL = "segmentation/seg_PIDNet_218_cityscapes_288x288_fp32.rknn"
H, W = 288, 288         # モデル入力（height, width）※32の倍数推奨
VIDEO = 'backwards_320.mp4'
ALPHA = 0.9              # オーバーレイ透明度（1.0=マスクのみ / 0.0=元映像のみ）

# EMA（指数移動平均）
USE_EMA = True           # 有効/無効
EMA_ALPHA = 0.6          # 0.5～0.8（大きいほど最新フレームを重視）

# Cityscapes 19クラス簡易パレット（BGR）
PALETTE = np.array([
    [255,255,255],[244, 35,232],[ 70, 70, 70],[102,102,156],[190,153,153],
    [153,153,153],[250,170, 30],[220,220,  0],[107,142, 35],[152,251,152],
    [ 70,130,180],[220, 20, 60],[255,  0,  0],[  0,  0,142],[  0,  0, 70],
    [  0, 60,100],[  0, 80,100],[  0,  0,230],[119, 11, 32]
], dtype=np.uint8)

def put_latest(q: queue.Queue, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)


class DetectSidewalk:
    def __init__(self, in_queue, out_queue):
        self.in_queue = in_queue
        self.out_queue = out_queue

    def _nc1hwc2_to_nchw(self, x):
        """RKNNの NC1HWC2 を NCHW へ"""
        n, c1, h, w, c2 = x.shape
        x = x.transpose(0, 1, 4, 2, 3)   # N, C1, 2, H, W
        return x.reshape(n, c1 * c2, h, w)

    def _ensure_prob(self, scores_nchw: np.ndarray) -> np.ndarray:
        """
        NCHWスコアを確率に（softmax前/後どちらでもOKにする）
        - すでに確率（各画素の和 ≈ 1）っぽければそのまま返す
        - そうでなければ softmax を適用
        """
        scores = scores_nchw.astype(np.float32, copy=False)
        sum_ch = scores.sum(axis=1, keepdims=True)
        if (scores.min() >= -1e-4 and scores.max() <= 1.0001
                and abs(float(sum_ch.mean()) - 1.0) < 5e-3):
            return scores
        # logits -> softmax
        L = scores - scores.max(axis=1, keepdims=True)
        ex = np.exp(L, dtype=np.float32)
        return ex / (ex.sum(axis=1, keepdims=True) + 1e-6)

    def main(self):
        # ===== RKNN 初期化 =====
        rk = RKNNLite()
        assert rk.load_rknn(RKNN_MODEL) == 0, "load_rknn failed"
        assert rk.init_runtime() == 0, "init_runtime failed"
    
        # ===== 入力（動画 or カメラ） =====
        #cap = cv2.VideoCapture(11)  # カメラ
        #cap = cv2.VideoCapture(VIDEO)  # 動画ファイル
    
        prev_probs = None  # EMA用の前フレーム確率
    
        while True:
            frame_bgr = self.in_queue.get(timeout=1)
            
            # ----- 前処理（NHWC / RGB） -----
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_LINEAR)
            inp = np.expand_dims(resized, axis=0)  # [1, H, W, C]（RKNNはNHWCでOK）
    
            # ----- 推論 -----
            outs = rk.inference(inputs=[inp])
            out0 = outs[0]
    
            # ----- 出力形状を NCHW スコアに統一 -----
            if out0.ndim == 5:                             # NC1HWC2
                scores = self._nc1hwc2_to_nchw(out0)             # [1, C, H, W]
            elif out0.ndim == 4:
                if out0.shape[1] in (19, 20, 21):          # [1, C, H, W]
                    scores = out0
                else:                                      # [1, H, W, C] -> [1, C, H, W]
                    scores = out0.transpose(0, 3, 1, 2)
            else:
                raise RuntimeError(f'unexpected output shape: {out0.shape}')
    
            # ----- 確率に変換（softmax後ならそのまま） -----
            probs = self._ensure_prob(scores)  # [1, C, H, W], float32, 各画素合計=1
    
            # ----- 時系列EMA（指数移動平均） -----
            if USE_EMA:
                if prev_probs is None:
                    ema = probs
                else:
                    ema = EMA_ALPHA * probs + (1.0 - EMA_ALPHA) * prev_probs
                    # 数値安定のため再正規化（各画素で合計=1に戻す）
                    ema /= (ema.sum(axis=1, keepdims=True) + 1e-6)
                prev_probs = ema
                probs = ema
    
            # ----- argmaxでクラスID -----
            pred = probs.argmax(axis=1)[0].astype(np.uint8)  # [H, W]
    
            # ----- 元サイズへ最近傍で拡大 -----
            mask = cv2.resize(pred, (frame_bgr.shape[1], frame_bgr.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
    
            # ----- カラー化 & オーバーレイ表示 -----
            color = PALETTE[mask]  # BGR
            if ALPHA >= 1.0:
                overlay = color
            elif ALPHA <= 0.0:
                overlay = frame_bgr
            else:
                overlay = cv2.addWeighted(frame_bgr, 1.0 - ALPHA, color, ALPHA, 0.0)
    
            put_latest(self.out_queue, overlay)
