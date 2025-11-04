
# online_matcher.py — Offline, sequential (streaming) road matcher
# NOTE: "online" here means sequential/real-time style processing, not internet usage.
import json, math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from shapely.geometry import shape, LineString, MultiLineString, Point
from shapely.strtree import STRtree

ROAD_PATH = "./gps/map/roads.geojson"

def bearing_deg(p1: Tuple[float,float], p2: Tuple[float,float]) -> float:
    lat1, lon1 = map(math.radians, p1)
    lat2, lon2 = map(math.radians, p2)
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    return (math.degrees(math.atan2(y,x)) + 360.0) % 360.0

def ang_diff(a, b):
    if a is None or b is None:
        return 0.0
    return abs((a - b + 180) % 360 - 180)

@dataclass
class MatchState:
    line: LineString
    s: float                  # position along the line (degrees)
    lat: float
    lon: float
    heading: Optional[float]  # deg (estimate of road tangent)
    t: Optional[float]        # seconds

class OfflineSequentialMatcher:
    """
    Fully offline road matcher using a local roads GeoJSON (LineString/MultiLineString).
    "Online" means sequential updates per GPS sample. No internet access required.
    """
    def __init__(self, roads_geojson: str = ROAD_PATH,
                 search_m: float = 40.0,
                 switch_penalty_m: float = 15.0,
                 w_dist: float = 1.0,
                 w_head: float = 0.5,
                 w_prog: float = 0.7):
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

        self.lines = lines
        self.index = STRtree(lines)

        self.search_deg = search_m / 111_000.0
        self.switch_penalty_m = switch_penalty_m
        self.w_dist = w_dist
        self.w_head = w_head
        self.w_prog = w_prog

    def _line_heading_at(self, ln: LineString, s_deg: float, eps: float = 1e-6) -> float:
        s0 = max(0.0, s_deg - eps); s1 = min(ln.length, s_deg + eps)
        p0 = ln.interpolate(s0); p1 = ln.interpolate(s1)
        return bearing_deg((p0.y, p0.x), (p1.y, p1.x))

    def update(self, lat: float, lon: float,
               t: Optional[float] = None,
               gps_heading: Optional[float] = None,
               prev: Optional[MatchState] = None,
               vmax_mps: float = 15.0) -> Optional[MatchState]:
        """
        Consume one GPS sample and return a new MatchState (or None if no road nearby).
        - lat, lon: WGS84 degrees
        - t: seconds (monotonic). If None, progression gating is disabled
        - gps_heading: degrees 0..360 (optional). If None, reuse prev.heading if available.
        - prev: previous MatchState
        - vmax_mps: maximum plausible speed to gate link progression
        """
        import numpy as np
        p = Point(lon, lat)
        cands = self.index.query(p.buffer(self.search_deg).envelope)
        cands = list(cands) if cands is not None else []
        if len(cands) == 0:
            return None
        if isinstance(cands[0], (int, np.integer, np.int64)):
            cands = [self.lines[int(i)] for i in cands]
        preferred = prev.line if prev else None
        best = None
        best_score = float("inf")

        # try each candidate line
        for ln in cands:
            s = ln.project(p)                  # position on line (degrees)
            np = ln.interpolate(s)             # nearest point on line
            dist_m = p.distance(np) * 111_000.0

            # heading consistency
            line_head = self._line_heading_at(ln, s)
            head_in = gps_heading if gps_heading is not None else (prev.heading if prev else None)
            head_pen = ang_diff(head_in, line_head)  # degrees

            # progression gate (limit how much we can progress along the same link)
            prog_pen_m = 0.0
            if prev and prev.line.equals(ln) and prev.t is not None and t is not None:
                dt = max(0.0, t - prev.t)
                max_prog_m = vmax_mps * dt + 10.0  # slack
                ds_m = abs((s - prev.s) * 111_000.0)
                if ds_m > max_prog_m:
                    prog_pen_m = ds_m - max_prog_m

            # switching cost to discourage hopping to parallel links
            switch_cost = 0.0
            if prev and not ln.equals(preferred):
                switch_cost = self.switch_penalty_m

            score = self.w_dist*dist_m + self.w_head*head_pen + self.w_prog*prog_pen_m + switch_cost
            if score < best_score:
                best_score = score
                best = (ln, s, np, line_head)

        if best is None:
            return None

        ln, s, np, line_head = best
        return MatchState(line=ln, s=s, lat=np.y, lon=np.x, heading=line_head, t=t)
