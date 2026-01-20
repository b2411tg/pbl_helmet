#!/usr/bin/env python3
import threading
from detect_stop_two.demo_stop_and_right_turn_detect import Detect2ndTurn
from yolo.yolo_object_detect import DetectYoloObject
from segmentation.detect_sidewalk_segmentation import DetectSidewalk
from gnss.positioning import GetPositioning
from sql.PostgreSQL_sample import PostgreSQL
import cv2
import queue
from threading import Event
import time
import sounddevice as sd
import soundfile as sf

class Shared:
    def __init__(self):
        self.detect_stop = Event()
        self.detect_intersection_30m = Event()
        self.detect_reverse = Event()
        self.gnss_position_ready = threading.Event()
        self.gnss_position = None
        self.sql_insert_line = threading.Event()
        self.sql_insert_data = None
        self.detect_status = 0
        self.stop_wav_run = False
        self.gnss_status = 0
        self.idx = 0
        self.frame_update = threading.Event()

class RunningMsg:
    def __init__(self, shared):
        self.shared = shared
        self.wav_data_running, self.wav_samplerate_running = sf.read("sound/anzen_kanshi.wav", dtype="float32")
        self.wav_data_no_gnss, self.wav_samplerate_no_gnss = sf.read("sound/no_gnss.wav", dtype="float32")
        self.wav_data_normal_gnss, self.wav_samplerate_normal_gnss = sf.read("sound/normal.wav", dtype="float32")
        self.wav_data_sub_gnss, self.wav_samplerate_sub_gnss = sf.read("sound/submeter.wav", dtype="float32")
        self.wav_data_float_gnss, self.wav_samplerate_float_gnss = sf.read("sound/float.wav", dtype="float32")
        self.wav_data_centi_gnss, self.wav_samplerate_centi_gnss = sf.read("sound/centimeter.wav", dtype="float32")

    def run(self):
        sd.play(self.wav_data_running, self.wav_samplerate_running, blocking=False)
        while True:
            time.sleep(10)
            if self.shared.detect_status != 2 and not self.shared.stop_wav_run:
                if self.shared.gnss_status == 0:
                    sd.play(self.wav_data_no_gnss, self.wav_samplerate_no_gnss, blocking=False)
                elif self.shared.gnss_status == 1:
                    sd.play(self.wav_data_normal_gnss, self.wav_samplerate_normal_gnss, blocking=False)
                elif self.shared.gnss_status == 2:
                    sd.play(self.wav_data_sub_gnss, self.wav_samplerate_sub_gnss, blocking=False)
                elif self.shared.gnss_status == 5:
                    sd.play(self.wav_data_float_gnss, self.wav_samplerate_float_gnss, blocking=False)
                elif self.shared.gnss_status == 4:
                    sd.play(self.wav_data_centi_gnss, self.wav_samplerate_centi_gnss, blocking=False)
                else:
                    sd.play(self.wav_data_running, self.wav_samplerate_running, blocking=False)

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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    while True:
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        if not ret:
            break
        # 各タスク入力キューへ複製を配信
        for q in outputs:
            put_latest(q, frame.copy())
    cap.release()

class MoviePlay:
    def __init__(self, shared):
        self.shared = shared

    def movie_play(self, cap, output):
        self.shared.idx = 0
        while True:
            self.shared.idx += 1
            self.shared.frame_update.set()
            ret, frame = cap.read()
            frame = cv2.resize(frame, (1920,1080))
            if not ret:
                break
            if not frame_demo_in.full():
                put_latest(frame_demo_in, frame.copy())
            time.sleep(0.02)
        cap.release()

def main():
    while True:
        if not frame_yolo_in.empty():
            frame = frame_yolo_in.get_nowait()
            cv2.imshow("YOLO DETECT", frame)
            cv2.moveWindow("YOLO DETECT", 1500, 700)
        if not frame_seg_in.empty():
            frame = frame_seg_in.get_nowait()
            cv2.imshow("SEGMENTATION DETECT", frame)
        if not frame_demo_in.empty():
            frame = frame_demo_in.get_nowait()
            cv2.imshow("DEMO_MOVIE", frame)
            cv2.moveWindow("DEMO_MOVIE", 0, 0)
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
    frame_demo_in = queue.Queue(maxsize=2)

    # ﾒｲﾝ側でｶﾒﾗをｵｰﾌﾟﾝしてﾌﾚｰﾑを取得するﾀｽｸ 設定
    cap = cv2.VideoCapture(11)
    thread_cap = threading.Thread(target=capture_loop, args=(cap, [frame_yolo_out, frame_seg_out]), daemon=True)
    thread_cap.start()

    # 各ﾀｽｸの設定
    detect_2ndturn = Detect2ndTurn(shared)
    detect_stp_back = DetectYoloObject(shared, in_queue=frame_yolo_out, out_queue=frame_yolo_in)
    detect_sidewalk = DetectSidewalk(in_queue=frame_seg_out, out_queue=frame_seg_in)
#    get_positionig = GetPositioning(shared)
    running_msg = RunningMsg(shared)
    sql_log_save = PostgreSQL(shared)
    thread_2ndturn = threading.Thread(target=detect_2ndturn.main, daemon=True)
    thread_stp_back = threading.Thread(target=detect_stp_back.main, daemon=True)
    thread_sidewalk = threading.Thread(target=detect_sidewalk.main, daemon=True)
#    thread_positioning = threading.Thread(target=get_positionig.main, daemon=True)
    thread_running = threading.Thread(target=running_msg.run, daemon=True)
#    thread_sql = threading.Thread(target=sql_log_save.main, daemon=True)
    thread_2ndturn.start()
    thread_stp_back.start()
    #thread_sidewalk.start()
#    thread_positioning.start()
    thread_running.start()
#    thread_sql.start()

    #
    movie_play = MoviePlay(shared)
    cap_movie = cv2.VideoCapture("demo_join_movie.mp4")
    thread_cap_demo = threading.Thread(target=movie_play.movie_play, args=(cap_movie, frame_demo_in), daemon=True)
    thread_cap_demo.start()

    main()

