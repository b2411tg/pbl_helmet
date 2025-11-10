import threading
from gps.detect_two_stage_turn_main import Detect2ndTurn
from yolo.detect_stop_backwards_yolo import DetectStopAndBackwards
from segmentation.detect_sidewalk_segmentation import DetectSidewalk
import cv2
import queue
from threading import Event

class Shared:
    def __init__(self):
        self.detect_stop = Event()
        self.detect_intersection_30m = Event()
        
def put_latest(q: queue.Queue, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)

def capture_loop(cap, outputs):
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while True:
        ret, frame = cap.read()
#        frame = cv2.rotate(frame, cv2.ROTATE_180)
        if not ret:
            break
        # 各タスク入力キューへ複製を配信
        for q in outputs:
            put_latest(q, frame.copy())
    cap.release()

def main():
    while True:
        if not frame_yolo_in.empty():
            frame = frame_yolo_in.get_nowait()
            cv2.imshow("YOLO DETECT", frame)
        if not frame_seg_in.empty():
            frame = frame_seg_in.get_nowait()
            cv2.imshow("SEGMENTATION DETECT", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':
    shared = Shared()

    # ｶﾒﾗのﾌﾚｰﾑﾃﾞｰﾀ保存ｷｭｰと推論結果後のﾌﾚｰﾑ を保存するｷｭｰ
    frame_yolo_in = queue.Queue(maxsize=2)
    frame_seg_in = queue.Queue(maxsize=2)
    frame_yolo_out = queue.Queue(maxsize=2)
    frame_seg_out = queue.Queue(maxsize=2)

    # ﾒｲﾝ側でｶﾒﾗをｵｰﾌﾟﾝしてﾌﾚｰﾑを取得するﾀｽｸ 設定
    cap = cv2.VideoCapture(11)
    net, frame = cap.read()
    thread_cap = threading.Thread(target=capture_loop, args=(cap, [frame_yolo_out, frame_seg_out]), daemon=True)
    thread_cap.start()

    # 各ﾀｽｸの設定
    detect_2ndturn = Detect2ndTurn(shared)
    detect_stp_back = DetectStopAndBackwards(shared, in_queue=frame_yolo_out, out_queue=frame_yolo_in)
    detect_sidewalk = DetectSidewalk(in_queue=frame_seg_out, out_queue=frame_seg_in)
    thread_2ndturn = threading.Thread(target=detect_2ndturn.main, daemon=True)
    thread_stp_back = threading.Thread(target=detect_stp_back.main, daemon=True)
    thread_sidewalk = threading.Thread(target=detect_sidewalk.main, daemon=True)
    thread_2ndturn.start()
    thread_stp_back.start()
    #thread_sidewalk.start()

    main()

