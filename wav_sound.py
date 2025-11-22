import sounddevice as sd
import soundfile as sf

def play_audio(path: str):
    # ファイル読み込み
    data, samplerate = sf.read(path, dtype="float32")
    # 再生
    sd.play(data, samplerate)
    sd.wait()   # 再生終了まで待機

if __name__ == "__main__":
    play_audio("sound/detect_stop_ok.wav")  # ←ここを再生したいファイルに


