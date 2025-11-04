import threading
from gps import detect_two_stage_turn_main
from yolo import detect_video_rknn_nosave

detect1 = detect_two_stage_turn_main.Detect2ndTurn()            
detect2 = detect_video_rknn_nosave.DetectStopAndBackwards()
thread1 = threading.Thread(target=detect1.main)
thread2 = threading.Thread(target=detect2.main)
thread1.start()
thread2.start()