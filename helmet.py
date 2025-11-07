import threading
from gps.detect_two_stage_turn_main import Detect2ndTurn
from yolo.detect_stop_backwards_yolo import DetectStopAndBackwards
from segmentation.detect_sidewalk_segmentation import DetectSidewalk
import cv2
import queue

def main():
    while True:
        if not frame_yolo.empty():
            frame = frame_yolo.get_nowait()
            cv2.imshow("YOLO DETECT", frame)
        if not frame_seg.empty():
            frame = frame_seg.get_nowait()
            cv2.imshow("SEGMENTATION DETECT", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':
    frame_yolo = queue.Queue()
    frame_seg = queue.Queue()
    detect_2ndturn = Detect2ndTurn()
    detect_stp_back = DetectStopAndBackwards(out_queue=frame_yolo)
    detect_sidewalk = DetectSidewalk(out_queue=frame_seg)
    thread_2ndturn = threading.Thread(target=detect_2ndturn.main, daemon=True)
    thread_stp_back = threading.Thread(target=detect_stp_back.main, daemon=True)
    thread_sidewalk = threading.Thread(target=detect_sidewalk.main, daemon=True)
    thread_2ndturn.start()
    thread_stp_back.start()
    thread_sidewalk.start()
    main()

