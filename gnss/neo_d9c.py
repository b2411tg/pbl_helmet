import serial
import sounddevice as sd
import soundfile as sf


ser = serial.Serial("/dev/ttyS4", 230400, timeout=0.5)
while(True):
    line = ser.readline()
    if line:
        print(" ".join(f"{x:02X}" for x in line))



