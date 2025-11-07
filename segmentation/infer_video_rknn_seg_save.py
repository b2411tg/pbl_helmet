# infer_video_rknn_seg_ema.py
# 元スクリプトに「時系列EMA（指数移動平均）」のみ追加した版（出力は動画保存）

from rknnlite.api import RKNNLite
import numpy as np, cv2
import time

RKNN_MODEL = "segmentation/seg_STDC_268_cityscapes_192x384_fp32.rknn"   # モデル名
H, W = 192, 384          # モデル入力（height, width）※32の倍数推奨
VIDEO = 'segmentation/video7_320.mp4'
ALPHA = 0.5            # 透過（cv2.addWeightedを使う場合）

# ===== 追加: EMA 設定 =====
USE_EMA = True         # EMAを使う/使わない
EMA_ALPHA = 0.6        # 0.5～0.8（大きいほど最新フレームを重視）

# Cityscapes 19クラス簡易パレット（BGR）
PALETTE = np.array([
    [255,255,255],[244, 35,232],[ 70, 70, 70],[102,102,156],[190,153,153],
    [153,153,153],[250,170, 30],[220,220,  0],[107,142, 35],[152,251,152],
    [ 70,130,180],[220, 20, 60],[255,  0,  0],[  0,  0,255],[  0,  0, 70],
    [  0, 60,100],[  0, 80,100],[  0,  0,230],[119, 11, 32]
], dtype=np.uint8)

def nc1hwc2_to_nchw(x):
    # 例: (1, C1, H, W, 2) -> (1, C, H, W)
    n, c1, h, w, c2 = x.shape
    x = x.reshape(n, c1, c2, h, w).transpose(0,1,2,3,4)
    return x.reshape(n, c1*c2, h, w)

def ensure_prob(scores_nchw: np.ndarray) -> np.ndarray:
    """
    NCHWスコアを確率に（softmax前/後どちらでもOK）
    - すでに確率（各画素の和 ≈ 1）っぽければそのまま
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

rk = RKNNLite()
assert rk.load_rknn(RKNN_MODEL) == 0
assert rk.init_runtime() == 0

cap = cv2.VideoCapture(VIDEO)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(
    f"segmentation/inference_result.mp4",
    fourcc,
    cap.get(cv2.CAP_PROP_FPS),
    (int(cap.get(3)), int(cap.get(4)))
)

# ===== 追加: EMA用の前フレーム確率 =====
prev_probs = None

start = time.time()

while True:
    ret, frame_bgr = cap.read()
    if not ret: break

    # ----- 前処理 -----
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_LINEAR)
    inp = np.expand_dims(resized, axis=0)  # [1,H,W,C] (RKNNはNHWCでOK)

    # ----- 推論 -----
    outs = rk.inference(inputs=[inp])
    out0 = outs[0]

    # ----- 形状を判定し、NCHWのスコアに統一 -----
    if out0.ndim == 5:               # NC1HWC2
        scores = nc1hwc2_to_nchw(out0)     # [1,C,H,W]
    elif out0.ndim == 4:
        # [1,H,W,C] or [1,C,H,W]
        if out0.shape[1] in (19, 20, 21):  # Cが2次元目→NCHW
            scores = out0
        else:                               # NHWC -> NCHW
            scores = out0.transpose(0, 3, 1, 2)
    else:
        raise RuntimeError(f'unexpected output shape: {out0.shape}')

    # ===== 追加: 確率に変換（softmax後ならそのまま） =====
    probs = ensure_prob(scores)  # [1,C,H,W], float32, 各画素合計=1

    # ===== 追加: 時系列EMA（指数移動平均） =====
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
    pred = probs.argmax(axis=1)[0].astype(np.uint8)     # [H,W]

    # ----- 元フレームサイズへ最近傍で拡大（ブロック化は正常） -----
    mask = cv2.resize(pred, (frame_bgr.shape[1], frame_bgr.shape[0]),
                      interpolation=cv2.INTER_NEAREST)

    # ----- カラー化 & オーバーレイ -----
    color = PALETTE[mask]                               # [H,W,3] BGR
    overlay = color
    # もし元映像と合成したいなら以下を使う：
    # overlay = cv2.addWeighted(frame_bgr, 1-ALPHA, color, ALPHA, 0)

    out.write(overlay)
    # リアル表示は不要とのことなのでコメントアウトのまま
    # cv2.imshow('seg', overlay); cv2.waitKey(1)

cap.release()
out.release()

end = time.time()
elapsed_time = end - start
print(f"処理時間: {elapsed_time:.4f} 秒")
print(elapsed_time/45)
