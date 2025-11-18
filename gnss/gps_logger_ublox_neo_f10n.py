import serial
import time
from datetime import datetime
from pathlib import Path
import sys

class GetGpsPositioning:
  def __init__(self):
    self.open_ser = False

  def gps_init(self):
    try:
      self.ser = serial.Serial("/dev/ttyS4", 230400, timeout=0.5)
      time.sleep(0.5)
      self.open_ser = True
    except Exception as e:
      print(e)
    return self.open_ser

  def open_csv(self):
    try:   
      ts = datetime.now().strftime("%Y%m%d_%H%M%S")
      csv_path = Path(f"gps_log_{ts}.csv")
      self.f = open(csv_path, "w", newline="", encoding="utf-8")
      column_name = 'UTC,latitude,longitude\n'
      self.f.writelines(column_name)
      return True
    except Exception as e:
      print(e)
      return False
    
  def get_pos(self):
    while(True):
      line = self.ser.readline().decode(errors="ignore").strip()
      if line and line.startswith("$GPGGA"):
        self.line = line.split(',')
        break

  def gga_to_decimal(self, coord):
    deg = int(coord / 100)
    minutes = coord - deg * 100
    return round(deg + minutes / 60, 8)

  def save_pos(self):
    if self.line[0] == '$GPGGA':
      latitude = self.gga_to_decimal(float(self.line[2]))
      longitude = self.gga_to_decimal(float(self.line[4]))
      data = f'{float(self.line[1]):.1f},{latitude:.8f},{longitude:.8f}'
      print(data)
#      self.f.writelines(data + '\n')
#      self.f.flush()

  def main(self):
    if self.gps_init() == False:
      sys.exit()
#    if self.open_csv() == False:
#      sys.exit()
    while(True):
      self.get_pos()
      self.save_pos()

if __name__ == "__main__":
  gps_pos = GetGpsPositioning()
  gps_pos.main()