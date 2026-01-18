import cv2

# 入力動画リスト（指定どおり）
video_files = [
    "4870-6610_二段階_一時停止.mp4",
    "20290-20860_二段階.mp4",
    "21450-22440_二段階_一時停止.mp4",
    "9760-10280_一時停止.mp4",
    "17300-17840_二段階.mp4"
]

caps = [cv2.VideoCapture(v) for v in video_files]

# 最初の動画から基本情報を取得
fps = caps[0].get(cv2.CAP_PROP_FPS)
width  = int(caps[0].get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(caps[0].get(cv2.CAP_PROP_FRAME_HEIGHT))

# 出力動画設定
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = cv2.VideoWriter("merged.mp4", fourcc, fps, (width, height))

for cap in caps:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        out.write(frame)
    cap.release()

out.release()
print("結合完了: merged.mp4")
