from gps.gnss_turn_detect.nearest_intersection import nearest_intersection, nearest_intersection_with_distance
from gps.gnss_turn_detect.distance_and_bearing import distance_and_bearing_east0
from gps.gnss_turn_detect.online_matcher import OfflineSequentialMatcher
import pandas as pd
from collections import deque
import math
import time

PATH = "./gps/gps_log_neo_f10n_20251021_213516.csv"
PREV_DENSITY_DATA = 7           # 密集を検出するデータの範囲
DENSITY_DETECT_DISTANCE = 1     # 密集を検出する範囲(m)
PREV_SAVE_SIZE = 10             # 検出に使用する為のデータ保存数
DIRECTION_PREV_DIFF = 5         # 方位を何個前のデータと比較するか
DETECT_MOVE_DISTANCE = 2        # 密集を検出した際、走行しているかどうかを判断する走行距離(m)
DETECT_STOP_RADIUS = 15          # 違反検出位置から停止するを半径(m)（重複防止）


class Detect2ndTurn:
    def __init__(self, shared):
        self.matcher = OfflineSequentialMatcher(search_m=8, switch_penalty_m=30, w_dist=1.0, w_head=0.4, w_prog=0.6)
        self.prev = None
        self.prev_match_data = deque(maxlen=PREV_SAVE_SIZE)     # マッチングデータ過去データ配列
        self.prev_former_data = deque(maxlen=PREV_SAVE_SIZE)    # 元データ過去データ配列
        self.from_detect_pos = 0
        self.detect_pos = 0
        self.detect_on = True
        self.shared = shared

    def _bearing_deg(self, lat1, lon1, lat2, lon2):
        """(lat1,lon1)->(lat2,lon2) の方位角[deg]"""
        r = math.radians
        dlon = r(lon2 - lon1)
        lat1, lat2 = r(lat1), r(lat2)
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
        brng = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
        return brng

    # マップマッチング関数
    def _location_matching(self, utc, latitude, longitude):
            if len(self.prev_former_data) < (PREV_SAVE_SIZE):
                distance_m = -1
                angle_deg = -1
                heading = None
            else:
                # 元データと指定の過去データから移動した距離と方角を取得
                distance_m, angle_deg = distance_and_bearing_east0(
                     self.prev_former_data[PREV_SAVE_SIZE-DIRECTION_PREV_DIFF][1],
                     self.prev_former_data[PREV_SAVE_SIZE-DIRECTION_PREV_DIFF][2],
                     latitude, longitude) 
                heading = self._bearing_deg(self.prev_former_data[PREV_SAVE_SIZE-1][1], self.prev_former_data[PREV_SAVE_SIZE-1][2], latitude, longitude)
            # 元データからマップマッチングの緯度経度を取得
            st = self.matcher.update(lat=float(latitude), lon=float(longitude),
                            t=float(utc), gps_heading=heading, prev=self.prev, vmax_mps=15.0)
            # 元データ配列、マップマッチングデータ配列に追加
            if st is None:
                self.prev_match_data.append((utc, -1, -1, angle_deg, distance_m))
                self.prev_former_data.append((utc, latitude, longitude, angle_deg, distance_m))
                return utc, -1, -1, angle_deg
            else:
                self.prev = st
                self.prev_match_data.append((utc, st.lat, st.lon, angle_deg, distance_m))
                self.prev_former_data.append((utc, latitude, longitude, angle_deg, distance_m))
                return utc, st.lat, st.lon, angle_deg, distance_m
            
    def main(self):
        #TODO GPSから緯度経度取得
        df = pd.read_csv(PATH)  # 列: UTC, latitude, longitude
        for idx, r in df.iterrows():
            time.sleep(0.1)            
        #TODO ここまで

            # マップマッチングデータ取得（緯度、経度、走行距離/s、走行方角
            try:
                utc, match_lat, match_lon, angle_deg, move_distance_m = self._location_matching(r["UTC"], r["latitude"], r["longitude"])
            except:
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m}')
                continue

            # 前回検出地点から指定の半径は検出しない（重複防止）
            if self.detect_on == False:
                self.from_detect_pos, _ = distance_and_bearing_east0(self.detect_pos[0], self.detect_pos[1], r["latitude"], r["longitude"])
                if self.from_detect_pos < DETECT_STOP_RADIUS:
                    print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m}')
                    continue
            self.detect_on = True

            # 近くの交差点緯度経度取得
            near_lat, near_lon, near_intersection_distance = nearest_intersection_with_distance(match_lat, match_lon)

            # 交差点まで30m以内か
            if near_intersection_distance > 30:
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m}')
                self.shared.detect_intersection_30m.clear()
                continue

            # yoloﾀｽｸに交差点30m圏内である事を通知
            self.shared.detect_intersection_30m.set()

            # マップマッチングデータにおける密集度算出（指定の過去データと距離を算出）
            max_distance = 0
            min_distance = 30
            for i in range(8):
                prev_density_distance_m, _ = distance_and_bearing_east0(
                         self.prev_match_data[PREV_SAVE_SIZE-i-1][1],
                         self.prev_match_data[PREV_SAVE_SIZE-i-1][2],
                         near_lat, near_lon)                   
                if prev_density_distance_m > max_distance:
                    max_distance = prev_density_distance_m
                if prev_density_distance_m < min_distance:
                    min_distance = prev_density_distance_m
                prev_density_distance_m = max_distance - min_distance
            # 判定　ﾏｯﾁﾝｸﾞﾃﾞｰﾀ密集度＋移動距離/m＋交差点の方角＋走行方角
            if prev_density_distance_m > DENSITY_DETECT_DISTANCE:   # ﾏｯﾁﾝｸﾞﾃﾞｰﾀは指定の距離の範囲内に密集しているか
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m},{prev_density_distance_m}')
                continue
            if move_distance_m < DETECT_MOVE_DISTANCE:              # 自転車は走行しているか
                # 走行停止の場合
                if self.shared.detect_stop.is_set():                # 一時停止監視中の場合
                    _, intersection_angle_deg = distance_and_bearing_east0( # 交差点の方角取得
                        self.prev_former_data[PREV_SAVE_SIZE-1][1],
                        self.prev_former_data[PREV_SAVE_SIZE-1][2],
                                near_lat, near_lon)
                    d = (intersection_angle_deg - angle_deg) % 360
                    if not (90 < d < 270):                          # 走行方角に対し交差点は前方か
                        print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m},{prev_density_distance_m},"detect_stop_OK"')
                        continue
                    else:                                           # 走行方角に対し交差点は後方
                        print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m},{prev_density_distance_m},"detect_stop_NG"')
                        continue
                else:   # 一時停止監視中でない場合は何もしない
                    print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m},{prev_density_distance_m}')
                    continue
            # 走行していた場合交差点の方角取得
            _, intersection_angle_deg = distance_and_bearing_east0( # 交差点の方角取得
                self.prev_former_data[PREV_SAVE_SIZE-1][1],
                self.prev_former_data[PREV_SAVE_SIZE-1][2],
                         near_lat, near_lon)
            d = (intersection_angle_deg - angle_deg) % 360
            if 180 < d < 360:                                       # 走行方角に対し交差点は右側か左側か
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m},{prev_density_distance_m}')
                continue
            # 違反とする
            print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{move_distance_m},{prev_density_distance_m},"detect_2nd_turn"')
            self.detect_pos = [r["latitude"], r["longitude"]]
            self.detect_on = False   # 指定の半径は検出を停止する（重複防止）

if __name__ == "__main__":
    detect = Detect2ndTurn()
    detect.main()