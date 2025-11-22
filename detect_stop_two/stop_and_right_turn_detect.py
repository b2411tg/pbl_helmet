from detect_stop_two.gnss_turn_detect.nearest_intersection import nearest_intersection, nearest_intersection_with_distance
from detect_stop_two.gnss_turn_detect.distance_and_bearing import distance_and_bearing_east0
from detect_stop_two.gnss_turn_detect.online_matcher import OfflineSequentialMatcher
import pandas as pd
from collections import deque
import math
import time
import sounddevice as sd
import soundfile as sf
import numpy as np

#PATH = "./detect_stop_two/gps_log_neo_f10n_20251021_213516.csv"
#PATH = "./detect_stop_two/gps_log_20251116_134409.csv"
#PATH = "./detect_stop_two/gps_log_20251116_134409_1.csv"
PATH = "./detect_stop_two/gps_log_20251118_140059.csv"
PREV_DENSITY_DATA = 7           # 密集を検出するデータの範囲
DENSITY_DETECT_DISTANCE = 1.5   # 密集を検出する範囲(m)
PREV_SAVE_SIZE = 10             # 検出に使用する為のデータ保存数
DIRECTION_PREV_DIFF = 5         # 方位を何個前のデータと比較するか
DETECT_MOVE_DISTANCE = 2        # 密集を検出した際、走行しているかどうかを判断する走行距離(m)
DETECT_STOP_RADIUS = 15          # 違反検出位置から停止するを半径(m)（重複防止）


class Detect2ndTurn:
    def __init__(self, shared):
        self.matcher = OfflineSequentialMatcher(search_m=8, switch_penalty_m=20, w_dist=1.0, w_head=0.4, w_prog=0.6)
        self.prev = None
        self.prev_match_data = deque(maxlen=PREV_SAVE_SIZE)     # マッチングデータ過去データ配列
        self.prev_former_data = deque(maxlen=PREV_SAVE_SIZE)    # 元データ過去データ配列
        self.from_detect_pos = 0
        self.detect_pos = 0
        self.detect_on = True
        self.shared = shared
        self.wav_data_stop_ok, self.wav_samplerate_stop_ok = sf.read("sound/detect_stop_ok.wav", dtype="float32")
        self.wav_data_stop_ng, self.wav_samplerate_stop_ng = sf.read("sound/detect_stop_ng.wav", dtype="float32")
        self.wav_data_turn_ng, self.wav_samplerate_turn_ng = sf.read("sound/detect_two_turn_ng.wav", dtype="float32")
        self.wav_data_turn_ok, self.wav_samplerate_turn_ok = sf.read("sound/detect_two_turn_ok.wav", dtype="float32")

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
            heading = self._bearing_deg(self.prev_former_data[PREV_SAVE_SIZE-5][1], self.prev_former_data[PREV_SAVE_SIZE-1][2], lat, lon)
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
        match_intersection_distance = 0
        nomatch_intersection_distance = 0
        turn_stop_ok_flag =False
        match_lat = 0
        match_lon = 0
        leave_cnt = 0
        near_lat = 0
        near_lon = 0

        #TODO GPSから緯度経度取得
        df = pd.read_csv(PATH)  # 列: UTC, latitude, longitude
        for idx, r in df.iterrows():
            time.sleep(0.1)            
            utc = r["UTC"]
            latitude = r["latitude"]
            longitude = r["longitude"]
        #TODO ここまで
#        while True:
#            self.shared.gnss_position_ready.wait()
#            pos = (self.shared.gnss_position).split(",")
#            utc = np.float64(pos[0])
#            latitude = np.float64(pos[1])
#            longitude = np.float64(pos[2])
#            self.shared.gnss_position_ready.clear()

            if utc==50437.0:
                pass
            if utc==16044475.0:
                pass
            if utc==16044583.0:
                pass
            if utc==16044660.0:
                pass
            if utc==50712.0:
                pass

            ''' これよりマッチング緯度経度取得、移動距離取得、走行方向取得、交差点緯度経度距離処理 '''

            # 現在の緯度経度と前の緯度経度から移動距離、方角を取得
            former_move_distance, angle_deg, heading = self._get_heading_distance(latitude, longitude)

            # マップマッチングデータ取得（緯度、経度、走行方角、走行距離m/s
            try:
                prev_match_lat = match_lat
                prev_match_lon = match_lon
                utc, match_lat, match_lon, match_heading= self._location_matching(utc, latitude, longitude, heading)
                # 交差点近くはﾏｯﾁﾝｸﾞできない為、前回の値にする
                if match_lat == -1 and match_lon == -1:
                    match_lat = prev_match_lat
                    match_lon = prev_match_lon
                # 近くの交差点緯度経度取得
                self.match_prev_distance = match_intersection_distance
                self.nomatch_prev_distance = nomatch_intersection_distance
                prev_near_lat = near_lat
                prev_near_lon = near_lon
                near_lat, near_lon, match_intersection_distance = nearest_intersection_with_distance(match_lat, match_lon)
                _, _, nomatch_intersection_distance = nearest_intersection_with_distance(latitude, longitude)
                self.prev_match_data.append((utc, match_lat, match_lon, match_heading, match_intersection_distance))
                self.prev_former_data.append((utc, latitude, longitude, angle_deg, former_move_distance))
            except:
                self.prev_match_data.append((utc, match_lat, match_lon, match_heading, match_intersection_distance))
                self.prev_former_data.append((utc, latitude, longitude, angle_deg, former_move_distance))
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}')
                continue

            # データが指定数保存されるまでは検知に移行しない
            if len(self.prev_match_data) < PREV_SAVE_SIZE:
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}')
                continue

            ''' これより二段階右折、一時停止違反検出アルゴリズム '''

            # 前回検出地点から指定の半径は検出しない（重複防止）
            if self.detect_on == False:
                self.from_detect_pos, _ = distance_and_bearing_east0(self.detect_pos[0], self.detect_pos[1], latitude, longitude)
                if self.from_detect_pos < DETECT_STOP_RADIUS:
                    print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}')
                    continue
                else:
                    self.shared.detect_intersection_30m.clear()
                    self.detect_on = True
                    turn_stop_ok_flag = False

            # 近くの交差点検出位置が変化したら検出リセット
            if prev_near_lat != near_lat and prev_near_lon != near_lon:
                self.shared.detect_intersection_30m.clear()
                turn_stop_ok_flag = False

            # 交差点まで30m以内か
            if match_intersection_distance > 30:
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}')
                self.shared.detect_intersection_30m.clear()
                leave_cnt = 0
                turn_stop_ok_flag = False
                continue
            
            # 交差点に近づいていってるか
            if not self.shared.detect_intersection_30m.is_set() and self.match_prev_distance < match_intersection_distance:
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}')
                self.shared.detect_intersection_30m.clear()
                leave_cnt = 0
                continue
            # 交差点及び一時停止監視中に交差点が離れていった場合は一時停止違反と判定
            elif self.shared.detect_intersection_30m.is_set() and self.shared.detect_stop.is_set() and self.nomatch_prev_distance < nomatch_intersection_distance:
                # 離れていく軌跡連続5回でNGと判断
                leave_cnt += 1
                if leave_cnt >= 5:
                    print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}, "detect_stop_NG"')
                    sd.play(self.wav_data_stop_ng, self.wav_samplerate_stop_ng, blocking=False)
                    self.shared.detect_intersection_30m.clear()
                    self.detect_pos = [latitude, longitude]
                    self.detect_on = False   # 指定の半径は検出を停止する（重複防止）
                    continue
            else:
                leave_cnt = 0
            
            # 交差点監視ﾌﾗｸﾞ及び走行方向を保存
            if not self.shared.detect_intersection_30m.is_set():
                self.shared.detect_intersection_30m.set()
                save_running_direction = angle_deg

            # マップマッチングデータにおける密集度算出（指定の過去データと距離を算出）
            subset = list(self.prev_match_data)[PREV_SAVE_SIZE-PREV_DENSITY_DATA: PREV_SAVE_SIZE]
            column_distance = [row[4] for row in subset]
            prev_density_distance_m = max(column_distance) - min(column_distance)

            ''' 判定  ﾏｯﾁﾝｸﾞﾃﾞｰﾀ密集度＋移動距離/m＋交差点の方角＋走行方角 '''

            # ﾏｯﾁﾝｸﾞﾃﾞｰﾀは指定の距離の範囲内に密集しているか
            if prev_density_distance_m > DENSITY_DETECT_DISTANCE:
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}')
                continue

            # 元データは走行中か
            if former_move_distance < DETECT_MOVE_DISTANCE:
                # 元データにおいて停止中と判断した時 、一時停止監視中か
                if not self.shared.detect_stop.is_set():                

                    # TODO 交差点超えて停止していた場合、二段階右折正常監視開始
                    _, intersection_angle_deg = distance_and_bearing_east0( # 交差点の方角取得
                        self.prev_former_data[PREV_SAVE_SIZE-1][1],
                        self.prev_former_data[PREV_SAVE_SIZE-1][2],
                             near_lat, near_lon)
                    deg = (save_running_direction - intersection_angle_deg) % 360
                    if deg > 90 and deg < 270 :
                        if former_move_distance < 1:                 
                            turn_stop_ok_flag = True
                    print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}')
                    continue    # 一時停止監視中でない時は判定なし
                # 一時停止監視中の場合、元データは停止状態か確認 1m/s
                if former_move_distance > 1:                 
                    print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}')   
                # 一時停止監視中、停止したと確認できた為、OKと判定
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}, "detect_stop_OK"')
                sd.play(self.wav_data_stop_ok, self.wav_samplerate_stop_ok, blocking=False)
                self.detect_pos = [latitude, longitude]
                self.detect_on = False   # 指定の半径は検出を停止する（重複防止）
                continue

            # 走行していた場合二段階右折は右に40度以上の変化とする
            deg = (save_running_direction - angle_deg) % 360
            if not (40 < deg < 180):
                print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}')
                continue

            # 交差点の方角取得
            _, intersection_angle_deg = distance_and_bearing_east0( # 交差点の方角取得
                self.prev_former_data[PREV_SAVE_SIZE-1][1],
                self.prev_former_data[PREV_SAVE_SIZE-1][2],
                         near_lat, near_lon)
            deg = (intersection_angle_deg - angle_deg) % 360
            if 180 < deg < 360:                                       # 走行方角に対し交差点は右側か左側か
                if turn_stop_ok_flag:
                    print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}, detect_2nd_turn_ok')                   
                    sd.play(self.wav_data_turn_ok, self.wav_samplerate_turn_ng, blocking=False)
                    self.detect_pos = [latitude, longitude]
                    self.detect_on = False   # 指定の半径は検出を停止する（重複防止）
                else:
                    print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}')
                continue

            # ここまできたら二段階右折違反条件成立
            print(f'{utc}, {match_lat:.6f}, {match_lon:.6f}, head:{angle_deg:.6f}, move:{former_move_distance:.6f}, inter:{match_intersection_distance:.6f}, 密度:{prev_density_distance_m:.6f}, "detect_2nd_turn"')
            sd.play(self.wav_data_turn_ng, self.wav_samplerate_turn_ng, blocking=False)
            self.detect_pos = [latitude, longitude]
            self.detect_on = False   # 指定の半径は検出を停止する（重複防止）

if __name__ == "__main__":
    detect = Detect2ndTurn()
    detect.main()