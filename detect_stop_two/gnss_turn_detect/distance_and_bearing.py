'''
２地点間の距離と方角を取得する
入力：２地点の緯度経度
出力：距離と方角
'''
import math
from detect_stop_two.gnss_turn_detect.nearest_intersection import nearest_intersection, nearest_intersection_with_distance

def distance_and_bearing_east0(lat1, lon1, lat2, lon2, radius=6371000.0):
    """
    2地点の距離[m]と方角[deg]を返す。
    方角は 東=0°, 北=90°, 西=180°, 南=270°（反時計回り）で返す。

    Parameters
    ----------
    lat1, lon1 : 始点の緯度・経度（度）
    lat2, lon2 : 終点の緯度・経度（度）
    radius     : 地球半径[m]（既定: 6371000）

    Returns
    -------
    distance_m : float  大円距離[m]
    angle_deg  : float  東=0°, 0〜360 の角度（反時計回り）
    """
    # 度→ラジアン
    φ1 = math.radians(lat1)
    λ1 = math.radians(lon1)
    φ2 = math.radians(lat2)
    λ2 = math.radians(lon2)
    dλ = λ2 - λ1
    dφ = φ2 - φ1

    # --- 距離（haversine） ---
    sin_dφ2 = math.sin(dφ / 2.0)
    sin_dλ2 = math.sin(dλ / 2.0)
    a = sin_dφ2**2 + math.cos(φ1)*math.cos(φ2)*sin_dλ2**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    distance_m = radius * c

    # --- 初期方位（北=0°, 時計回り） ---
    y = math.sin(dλ) * math.cos(φ2)
    x = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(dλ)
    bearing_from_north = math.degrees(math.atan2(y, x)) % 360.0  # 北=0, 東=90

    # --- 東=0°, 反時計回りに変換 ---
    angle_deg = (90.0 - bearing_from_north) % 360.0

    return distance_m, angle_deg

