import serial
import time
from datetime import datetime
from pathlib import Path
import sys
from datetime import datetime, timedelta

class GetPositioning:
    def __init__(self, shared):
        self.open_ser = False
        self.shared = shared

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
            csv_path = Path(f"gps_log_{ts}_former.csv")
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
            if line and line.startswith("$GPRMC"):
                self.line = line.split(',')
                break

    def gga_to_decimal(self, coord):
        deg = int(coord / 100)
        minutes = coord - deg * 100
        return round(deg + minutes / 60, 8)

    def utc_to_jst_str(self, time, date):
        dd = int(date[0:2])
        MM = int(date[2:4])
        year = 2000 + int(date[4:6])
        hh = int(time[0:2])
        mm = int(time[2:4])
        ss = int(time[4:6])
        dt_utc = datetime(year, MM, dd, hh, mm, ss)
        dt_jst = dt_utc + timedelta(hours=9)
        return dt_jst.strftime('%Y%m%d%H%M%S') + time[6:8]
    
    def save_pos(self):
        if self.line[0] == '$GPRMC':
            if not self.line[3] and not self.line[5]:
                return
            jst_str = self.utc_to_jst_str(self.line[1], self.line[9])
            latitude = self.gga_to_decimal(float(self.line[3]))
            longitude = self.gga_to_decimal(float(self.line[5]))
            data = f'{jst_str},{latitude:.8f},{longitude:.8f}'
            self.shared.gnss_position = data
            self.shared.gnss_position_ready.set()
#            print(data)
            self.f.writelines(data + '\n')
            self.f.flush()

    def main(self):
        if self.gps_init() == False:
            sys.exit()
        if self.open_csv() == False:
            sys.exit()
        while(True):
            self.get_pos()
            self.save_pos()
