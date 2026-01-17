#!/usr/bin/env python3
import threading
from gnss.demo_make_positioning import GetPositioning
import cv2
import queue
from threading import Event
import time
import sounddevice as sd
import soundfile as sf
import os
from datetime import datetime

class Shared:
    def __init__(self):
        self.gnss_position_ready = threading.Event()
        self.gnss_position = None
        self.gnss_status = 0
        self.gnss_data = [0, 0, 0]

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

class MakeMovie:
    def __init__(self, shared):
        self.shared = shared

    def capture_loop(self):
        index = 1
        cap = cv2.VideoCapture("video_20260112_160019.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        writer = cv2.VideoWriter(f"video_{ts}.mp4", fourcc, 15, (1280, 720))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        try:
            while not stop_event.is_set():
                ret, frame = cap.read()
                if not ret:
                    break
                if index >= 34500:
                    writer.write(frame)
                index += 1
                if index > 35000:
                    break
        finally:
            print("END")
            writer.release()
            cap.release()
            os.close()

def main():
    try:
        while True:
            if not frame_image_in.empty():
                frame = frame_image_in.get_nowait()
                cv2.imshow("DEMO_VIDEO", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break           
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        stop_event.set()

if __name__ == '__main__':
    shared = Shared()
    stop_event = threading.Event()

    # ｶﾒﾗのﾌﾚｰﾑﾃﾞｰﾀ保存ｷｭｰと推論結果後のﾌﾚｰﾑ を保存するｷｭｰ
    frame_image_in = queue.Queue(maxsize=2)

    # ﾒｲﾝ側でｶﾒﾗをｵｰﾌﾟﾝしてﾌﾚｰﾑを取得するﾀｽｸ 設定
    make_movie = MakeMovie(shared)
    thread_make_movie = threading.Thread(target=make_movie.capture_loop, daemon=False)
    thread_make_movie.start()

    # 各ﾀｽｸの設定
    get_positionig = GetPositioning(shared)
    thread_positioning = threading.Thread(target=get_positionig.main, daemon=True)
    thread_positioning.start()

    running_msg = RunningMsg(shared)
    thread_running = threading.Thread(target=running_msg.run, daemon=True)
    thread_running.start()

    main()

