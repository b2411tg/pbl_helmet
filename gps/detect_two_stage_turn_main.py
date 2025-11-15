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

    def _get_heading_distance(self, lat, lon):
        if len(self.prev_former_data) < (PREV_SAVE_SIZE):
            former_move_distance = -1
            past_angle = -1
            heading = None
        else:
            # 指定の過去データに対し現在位置までの移動距離と方向( past_angleは東0度で半時計まわりである事に注意)
            former_move_distance, past_angle = distance_and_bearing_east0(
                self.prev_former_data[PREV_SAVE_SIZE-DIRECTION_PREV_DIFF][1],
                self.prev_former_data[PREV_SAVE_SIZE-DIRECTION_PREV_DIFF][2],
                lat, lon) 
            # 現在の走行方向(地理座標系の方位角であり北0度で時計回りである事に注意)
            heading = self._bearing_deg(self.prev_former_data[PREV_SAVE_SIZE-1][1], self.prev_former_data[PREV_SAVE_SIZE-1][2], lat, lon)
        return former_move_distance, past_angle, heading

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
    def _location_matching(self, utc, latitude, longitude, heading):
            # 元データからマップマッチングの緯度経度を取得
            st = self.matcher.update(lat=float(latitude), lon=float(longitude),
                            t=float(utc), gps_heading=heading, prev=self.prev, vmax_mps=15.0)
            # 元データ配列、マップマッチングデータ配列に追加
            if st is None:
                return utc, -1, -1, -1
            else:
                self.prev = st
                return utc, st.lat, st.lon, st.heading
            
    def main(self):
        near_intersection_distance = 0

        #TODO GPSから緯度経度取得
        df = pd.read_csv(PATH)  # 列: UTC, latitude, longitude
        for idx, r in df.iterrows():
            time.sleep(0.1)            
        #TODO ここまで
            utc = r["UTC"]
            latitude = r["latitude"]
            longitude = r["longitude"]

            # 現在の緯度経度と前の緯度経度から移動距離、方角を取得
            former_move_distance, angle_deg, heading = self._get_heading_distance(latitude, longitude)

            # マップマッチングデータ取得（緯度、経度、走行方角、走行距離m/s
            try:
                utc, match_lat, match_lon, match_heading= self._location_matching(utc, latitude, longitude, heading)
                # 近くの交差点緯度経度取得
                self.prev_distance = near_intersection_distance
                near_lat, near_lon, near_intersection_distance = nearest_intersection_with_distance(match_lat, match_lon)
                self.prev_match_data.append((utc, match_lat, match_lon, match_heading, near_intersection_distance))
                self.prev_former_data.append((utc, latitude, longitude, angle_deg, former_move_distance))
            except:
                self.prev_match_data.append((utc, match_lat, match_lon, match_heading, near_intersection_distance))
                self.prev_former_data.append((utc, latitude, longitude, angle_deg, former_move_distance))
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance}')
                continue

            # 前回検出地点から指定の半径は検出しない（重複防止）
            if self.detect_on == False:
                self.from_detect_pos, _ = distance_and_bearing_east0(self.detect_pos[0], self.detect_pos[1], r["latitude"], r["longitude"])
                if self.from_detect_pos < DETECT_STOP_RADIUS:
                    print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance}')
                    continue
            self.detect_on = True

            # 交差点まで30m以内か
            if near_intersection_distance > 30:
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance}')
                self.shared.detect_intersection_30m.clear()
                continue
            
            # 交差点に近づいていってるか
            if not self.shared.detect_intersection_30m.is_set() and self.prev_distance < near_intersection_distance:
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance}')
                self.shared.detect_intersection_30m.clear()
                continue

            # 交差点監視ﾌﾗｸﾞｾｯﾄ
            self.shared.detect_intersection_30m.set()

            # マップマッチングデータにおける密集度算出（指定の過去データと距離を算出）
            subset = list(self.prev_match_data)[PREV_SAVE_SIZE-PREV_DENSITY_DATA: PREV_SAVE_SIZE]
            column_distance = [row[4] for row in subset]
            prev_density_distance_m = max(column_distance) - min(column_distance)

            # 判定　ﾏｯﾁﾝｸﾞﾃﾞｰﾀ密集度＋移動距離/m＋交差点の方角＋走行方角
            if prev_density_distance_m > DENSITY_DETECT_DISTANCE:   # ﾏｯﾁﾝｸﾞﾃﾞｰﾀは指定の距離の範囲内に密集しているか
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance},{prev_density_distance_m}')
                continue
            if former_move_distance < DETECT_MOVE_DISTANCE:              # 自転車は走行しているか
                # 走行停止の場合
                if self.shared.detect_stop.is_set():                # 一時停止監視中の場合
                    _, intersection_angle_deg = distance_and_bearing_east0( # 交差点の方角取得
                        self.prev_former_data[PREV_SAVE_SIZE-1][1],
                        self.prev_former_data[PREV_SAVE_SIZE-1][2],
                                near_lat, near_lon)
                    d = (intersection_angle_deg - angle_deg) % 360
                    if not (90 < d < 270):                          # 走行方角に対し交差点は前方か
                        print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance},{prev_density_distance_m},"detect_stop_OK"')
                        continue
                    else:                                           # 走行方角に対し交差点は後方
                        print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance},{prev_density_distance_m},"detect_stop_NG"')
                        continue
                else:   # 一時停止監視中でない場合は何もしない
                    print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance},{prev_density_distance_m}')
                    continue
            # 走行していた場合交差点の方角取得
            _, intersection_angle_deg = distance_and_bearing_east0( # 交差点の方角取得
                self.prev_former_data[PREV_SAVE_SIZE-1][1],
                self.prev_former_data[PREV_SAVE_SIZE-1][2],
                         near_lat, near_lon)
            deg = (intersection_angle_deg - angle_deg) % 360
            if 180 < deg < 360:                                       # 走行方角に対し交差点は右側か左側か
                print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance},{prev_density_distance_m}')
                continue
            # 違反とする
            print(f'{utc},{match_lat:.6f},{match_lon:.6f},{angle_deg},{former_move_distance},{near_intersection_distance},{prev_density_distance_m},"detect_2nd_turn"')
            self.detect_pos = [latitude, longitude]
            self.detect_on = False   # 指定の半径は検出を停止する（重複防止）

if __name__ == "__main__":
    detect = Detect2ndTurn()
    detect.main()