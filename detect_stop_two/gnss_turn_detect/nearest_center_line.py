import json
import math
from pathlib import Path
from typing import List, Tuple, Optional

EARTH_R = 6371000.0  # m


# ---------- 角度・座標のユーティリティ ----------

def _deg_diff(a: float, b: float) -> float:
    """角度 a, b (deg) の差を [-180, 180] で返す"""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def _latlon_to_xy(lat: float, lon: float,
                  lat0: float, lon0: float) -> Tuple[float, float]:
    """
    緯度経度 -> 交差点(lat0, lon0) を原点とした平面座標(x:東, y:北) に変換
    （小さいエリア前提の近似）
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    x = (lon_rad - lon0_rad) * math.cos((lat_rad + lat0_rad) / 2) * EARTH_R
    y = (lat_rad - lat0_rad) * EARTH_R
    return x, y


def _heading_to_vec(heading_deg: float) -> Tuple[float, float]:
    """
    進行方向(東0°,反時計回り) -> 単位ベクトル (x:東, y:北)
    """
    rad = math.radians(heading_deg)
    return math.cos(rad), math.sin(rad)


# ---------- GeoJSON から道路中心線を取得 ----------

def _load_centerlines(path: Path) -> List[List[Tuple[float, float]]]:
    """
    GeoJSON から道路中心線の LineString を読み込む。
    戻り値は [[(lat, lon), ...], ...] のリスト。
    """
    with path.open("r", encoding="utf-8") as f:
        gj = json.load(f)

    lines = []
    for feat in gj.get("features", []):
        geom = feat.get("geometry", {})
        gtype = geom.get("type")
        coords = geom.get("coordinates", [])

        if gtype == "LineString":
            line = [(lat, lon) for lon, lat in coords]
            lines.append(line)
        elif gtype == "MultiLineString":
            for part in coords:
                line = [(lat, lon) for lon, lat in part]
                lines.append(line)
    return lines


# ---------- 線分への射影・距離計算 ----------

def _nearest_point_on_segment(px, py, ax, ay, bx, by):
    """
    点P(px,py) から線分AB への最近点を求める。
    戻り値: (qx, qy, t, dist)  tはAB上の位置(0~1)
    """
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    seg_len2 = vx * vx + vy * vy
    if seg_len2 == 0:
        return ax, ay, 0.0, math.hypot(wx, wy)

    t = (vx * wx + vy * wy) / seg_len2
    t_clamp = max(0.0, min(1.0, t))
    qx = ax + vx * t_clamp
    qy = ay + vy * t_clamp
    dist = math.hypot(px - qx, py - qy)
    return qx, qy, t_clamp, dist


# ---------- 進行方向に対応する中央線の決定 ----------

def _find_centerline_segment(
    inter_lat: float, inter_lon: float,
    heading_deg: float,
    lines: List[List[Tuple[float, float]]],
    max_search_dist: float = 50.0,
    max_angle_diff_deg: float = 70.0,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:

    best = None
    best_score = None

    for line in lines:
        # line を交差点中心座標系の xy に変換
        xy = [_latlon_to_xy(lat, lon, inter_lat, inter_lon) for (lat, lon) in line]

        # 各線分に対して交差点からの最短距離と向きを調べる
        for (lat1, lon1), (lat2, lon2), (x1, y1), (x2, y2) in zip(
            line[:-1], line[1:], xy[:-1], xy[1:]
        ):
            qx, qy, t, dist = _nearest_point_on_segment(0.0, 0.0, x1, y1, x2, y2)
            if dist > max_search_dist:
                continue

            vx, vy = x2 - x1, y2 - y1
            if vx == 0 and vy == 0:
                continue

            seg_heading1 = (math.degrees(math.atan2(vy, vx)) + 360.0) % 360.0
            seg_heading2 = (seg_heading1 + 180.0) % 360.0

            diff1 = abs(_deg_diff(seg_heading1, heading_deg))
            diff2 = abs(_deg_diff(seg_heading2, heading_deg))
            diff = min(diff1, diff2)

            if diff > max_angle_diff_deg:
                continue

            score = dist + diff * 0.5
            if best_score is None or score < best_score:
                best_score = score
                best = ((lat1, lon1), (lat2, lon2))

    return best


# ---------- 中央線を越えたかの判定 ----------

def _side_of_line(px, py, ax, ay, bx, by) -> float:
    """
    線分ABに対して点Pがどちら側か (符号付き)
    """
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)


def has_crossed_centerline(
    lat: float, lon: float, heading_deg: float,
    inter_lat: float, inter_lon: float,
    map_geojson_path: str
) -> bool:
    """
    引数:
        lat, lon        : 現在位置
        heading_deg     : 進行方向（東=0°, 反時計回り）
        inter_lat, inter_lon : 右折した交差点の位置
        map_geojson_path: 道路中心線が入った GeoJSON ファイルパス

    戻り値:
        True  - 想定される車道中央線を越えた側にいる
        False - まだ越えていない or 中央線が見つからない
    """
    lines = _load_centerlines(Path(map_geojson_path))

    seg = _find_centerline_segment(inter_lat, inter_lon, heading_deg, lines)
    if seg is None:
        # 該当しそうな中央線が見つからなかった場合
        return False

    (cl_lat1, cl_lon1), (cl_lat2, cl_lon2) = seg

    # 交差点を原点とした座標系に変換
    ax, ay = _latlon_to_xy(cl_lat1, cl_lon1, inter_lat, inter_lon)
    bx, by = _latlon_to_xy(cl_lat2, cl_lon2, inter_lat, inter_lon)
    px, py = _latlon_to_xy(lat,      lon,      inter_lat, inter_lon)

    # 「交差点に来る直前の位置」を進行方向から仮定
    vx, vy = _heading_to_vec(heading_deg)
    d_back = 10.0  # 10m 手前から来たと仮定
    refx = -vx * d_back
    refy = -vy * d_back

    side_ref = _side_of_line(refx, refy, ax, ay, bx, by)
    side_now = _side_of_line(px, py,   ax, ay, bx, by)

    eps = 1e-6

    # 参照点がほぼ線上に乗ってしまったときは判定不能とする
    if abs(side_ref) < eps:
        return False

    # 符号が変われば中央線を越えたとみなす
    if abs(side_now) < eps:
        # ほぼ中央線上
        return True

    return (side_ref > 0) != (side_now > 0)
