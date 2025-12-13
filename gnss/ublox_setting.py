import serial
import time

UBX_VALSET_5HZ = bytes.fromhex('B5 62 06 8A 0A 00 01 01 00 00 01 00 21 30 C8 00 B6 8B')
UBX_CFG_VALGET = bytes.fromhex('B5 62 06 8B 08 00 00 00 00 00 01 00 21 30 EB 07')
UBX_CFG_SAVE = bytes.fromhex('B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 17 31 BF')
UBX_BAURD_RATE = bytes.fromhex('B5 62 06 8A 0C 00 01 01 00 00 01 00 52 40 00 84 03 00 B8 FB')

ser = serial.Serial("/dev/ttyS4", 115200, timeout=0.5)  # 変更前ボーレートでオープン
ser.write(UBX_VALSET_5HZ)
time.sleep(0.5)
ser.write(UBX_BAURD_RATE)
time.sleep(0.5)
ser.close()
ser = serial.Serial("/dev/ttyS4", 230400, timeout=0.5)  # 変更後ボーレートでオープン
ser.write(UBX_CFG_SAVE)
time.sleep(0.5)
ser.close()
