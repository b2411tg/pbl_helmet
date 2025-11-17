'''
online_mapから呼び出さえる
'''
# two_stage_right.py
import json, math
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
from shapely.geometry import shape, LineString, MultiLineString, Point
from shapely.strtree import STRtree

def _deg_per_m(m: float) -> float:
    return m / 111_000.0

def _ang_diff(a: float, b: float) -> float:
    return abs((a - b + 180) % 360 - 180)

@dataclass
class TurnEvent:
    when_s: float
    inter_lon: float
    inter_lat: float
    entry_heading: float
    exit_heading: float
    delta_deg: float
    dwell_s: float
    kind: str   # "two_stage_ok" | "right_turn_violation" | "left_turn" | "straight" など

class TwoStageRightTurnDetector:
    """
    交差点近傍に入退出した際の“進入方位/退出方位/滞在時間”を使って
    - 二段階右折（適法）と
    - 交差点内の直接右折（違反）
    をしきい値ベースで判定する簡易ロジック。
    """
    def __init__(self,
                 roads_geojson: str,
                 min_degree: int = 3,
                 node_merge_tol_m: float = 6.0,
                 enter_radius_m: float = 12.0,
                 exit_radius_m: float = 15.0,
                 right_turn_min_deg: float = 60.0,
                 right_turn_max_deg: float = 120.0,
                 two_stage_min_dwell_s: float = 4.0):
        self.enter_r_deg = _deg_per_m(enter_radius_m)
        self.exit_r_deg  = _deg_per_m(exit_radius_m)
        self.right_min   = right_turn_min_deg
        self.right_max   = right_turn_max_deg
        self.two_stage_min = two_stage_min_dwell_s

        # ---- 交差点ノードを roads.geojson から作る（端点をマージして次数カウント）
        with open(roads_geojson, "r", encoding="utf-8") as f:
            gj = json.load(f)
        lines: List[LineString] = []
        for feat in gj["features"]:
            geom = shape(feat["geometry"])
            if isinstance(geom, LineString):
                lines.append(geom)
            elif isinstance(geom, MultiLineString):
                lines.extend(list(geom.geoms))
        if not lines:
            raise ValueError("No LineString/MultiLineString in roads_geojson")

        # 端点をバケット化して近接ノードをまとめる
        tol_deg = _deg_per_m(node_merge_tol_m)
        buckets: Dict[Tuple[int,int], List[Tuple[float,float]]] = {}
        def key_of(lon, lat):
            return (int(lon / tol_deg), int(lat / tol_deg))
        for ln in lines:
            x0, y0 = ln.coords[0][0], ln.coords[0][1]
            x1, y1 = ln.coords[-1][0], ln.coords[-1][1]
            buckets.setdefault(key_of(x0,y0), []).append((x0,y0))
            buckets.setdefault(key_of(x1,y1), []).append((x1,y1))

        # 代表点（平均）＋次数
        inter_points: List[Point] = []
        degrees: List[int] = []
        for pts in buckets.values():
            if len(pts) == 0: continue
            xs, ys = zip(*pts)
            cx = sum(xs) / len(xs); cy = sum(ys) / len(ys)
            deg = len(pts)
            if deg >= min_degree:
                inter_points.append(Point(cx, cy))
                degrees.append(deg)

        if not inter_points:
            raise ValueError("No intersection nodes detected")

        self.inters = inter_points
        self.inter_tree = STRtree(self.inters)
        self._geom_to_idx = {id(g): i for i, g in enumerate(self.inters)}
        # ---- 状態管理（交差点1件ずつ“エピソード”）
        self.active_inter_index: Optional[int] = None
        self.in_enter = False
        self.entry_heading: Optional[float] = None
        self.t_enter: Optional[float] = None
        self.last_state = None  # 前回の MatchState

        self.events: List[TurnEvent] = []      

    def _nearest_intersection(self, lon: float, lat: float) -> Tuple[int, Point, float]:
        import numpy as np
        p = Point(lon, lat)

        cand = self.inter_tree.query(p.buffer(self.exit_r_deg).envelope)
        # cand は Shapely のバージョンで「ジオメトリ配列」または「インデックス配列（np.ndarray of int）」になり得る
        if cand is None:
            return -1, None, float("inf")

        best_idx = -1
        best_pt: Optional[Point] = None
        best_d = float("inf")

        # numpy 配列で来ることがあるので反復可能に
        for c in (cand if isinstance(cand, (list, tuple, np.ndarray)) else [cand]):
            # 1) インデックスで返ってきた場合
            if isinstance(c, (int, np.integer)):
                idx = int(c)
                g = self.inters[idx]
            else:
                # 2) ジオメトリで返ってきた場合
                g = c
                idx = self._geom_to_idx.get(id(g))
                if idx is None:
                    # 念のため同値検索（ほぼ通らない想定）
                    idx = self.inters.index(g)

            d = p.distance(g)
            if d < best_d:
                best_d = d
                best_pt = g
                best_idx = idx

        if best_pt is None:
            return -1, None, float("inf")

        return best_idx, best_pt, best_d

    def update(self, st) -> Optional[TurnEvent]:
        """
        st: MatchState (online_matcher.MatchState)
        - st.lat, st.lon (WGS84度)
        - st.heading (道路接線の推定方位[deg])
        - st.t (秒)  が必要
        """
        if st is None or st.heading is None or st.t is None:
            self.last_state = st
            return None

        idx, ipt, d = self._nearest_intersection(st.lon, st.lat)

        # 交差点に“入る”
        if idx >= 0 and d <= self.enter_r_deg and not self.in_enter:
            self.active_inter_index = idx
            self.in_enter = True
            self.entry_heading = st.heading
            self.t_enter = st.t

        # 交差点から“出る”
        if self.in_enter:
            # 出る条件：現在の交差点から一定以上離れたら
            # （エピソードの終端）
            # 注意：距離は度→mにしてもしきい値一貫
            p = Point(st.lon, st.lat)
            inter_pt = self.inters[self.active_inter_index]
            if p.distance(inter_pt) >= self.exit_r_deg:
                # 退出時の情報
                exit_heading = st.heading
                dwell = max(0.0, st.t - (self.t_enter or st.t))
                delta = (exit_heading - (self.entry_heading or exit_heading) + 540) % 360 - 180
                kind = "straight"

                # 右折角度帯なら、滞在時間で“二段階” vs “違反”を分ける
                if self.right_min <= abs(delta) <= self.right_max:
                    if dwell < self.two_stage_min:
                        kind = "right_turn_violation"  # 交差点内で直接曲がった疑い
                    else:
                        kind = "two_stage_ok"         # 二段階右折（適法）
                elif delta <= -self.right_min and delta >= -self.right_max:
                    kind = "left_turn"
                else:
                    kind = "straight"

                ev = TurnEvent(
                    when_s=st.t,
                    inter_lon=inter_pt.x,
                    inter_lat=inter_pt.y,
                    entry_heading=self.entry_heading or float("nan"),
                    exit_heading=exit_heading,
                    delta_deg=delta,
                    dwell_s=dwell,
                    kind=kind
                )
                self.events.append(ev)

                # エピソード終了・初期化
                self.active_inter_index = None
                self.in_enter = False
                self.entry_heading = None
                self.t_enter = None

                self.last_state = st
                return ev

        self.last_state = st
        return None
