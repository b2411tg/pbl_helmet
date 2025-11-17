'''
指定の緯度経度に対し「intersection_centers.geojson」を使用して
近くの交差点の緯度、経度、距離を出力する
'''
import json
from pathlib import Path
import numpy as np
import pandas as pd
import os
INTERSECTIONS_PATH = Path("./detect_stop_two/map/intersection_nodes.geojson")

def _load_intersections(path: Path = INTERSECTIONS_PATH):
    if not path.exists():
        raise FileNotFoundError(f"Intersection file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        gj = json.load(f)
    lats, lons = [], []
    for feat in gj.get("features", []):
        geom = feat.get("geometry", {})
        if geom.get("type") == "Point":
            coords = geom.get("coordinates", None)
            if (
                isinstance(coords, (list, tuple)) 
                and len(coords) == 2 
                and coords[0] is not None 
                and coords[1] is not None
            ):
                lon, lat = float(coords[0]), float(coords[1])
                lats.append(lat)
                lons.append(lon)
    if not lats:
        raise ValueError("No Point geometries found in intersection_nodes.geojson")
    return np.array(lats, dtype=float), np.array(lons, dtype=float)

_INTER_LAT, _INTER_LON = _load_intersections()

def _haversine_vec(lat1, lon1, lat2_arr, lon2_arr):
    lat1 = np.radians(lat1)
    lon1 = np.radians(lon1)
    lat2 = np.radians(lat2_arr)
    lon2 = np.radians(lon2_arr)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    R = 6371000.0
    return R * c

def nearest_intersection(lat: float, lon: float):
    if not (np.isfinite(lat) and np.isfinite(lon)):
        raise ValueError("lat/lon must be finite floats")
    d = _haversine_vec(lat, lon, _INTER_LAT, _INTER_LON)
    idx = int(np.argmin(d))
    return float(_INTER_LAT[idx]), float(_INTER_LON[idx])

def nearest_intersection_with_distance(lat: float, lon: float):
    d = _haversine_vec(lat, lon, _INTER_LAT, _INTER_LON)
    idx = int(np.argmin(d))
    return float(_INTER_LAT[idx]), float(_INTER_LON[idx]), float(d[idx])

def batch_nearest_intersection(df: pd.DataFrame, lat_col="lat", lon_col="lon"):
    lats = df[lat_col].to_numpy(dtype=float)
    lons = df[lon_col].to_numpy(dtype=float)
    inter_lat = np.empty_like(lats, dtype=float)
    inter_lon = np.empty_like(lons, dtype=float)
    inter_dist = np.empty_like(lats, dtype=float)
    CHUNK = 10000
    for start in range(0, len(lats), CHUNK):
        end = start + CHUNK
        for i, (la, lo) in enumerate(zip(lats[start:end], lons[start:end])):
            d = _haversine_vec(la, lo, _INTER_LAT, _INTER_LON)
            idx = int(np.argmin(d))
            inter_lat[start + i] = _INTER_LAT[idx]
            inter_lon[start + i] = _INTER_LON[idx]
            inter_dist[start + i] = d[idx]
    out = df.copy()
    out["inter_lat"] = inter_lat
    out["inter_lon"] = inter_lon
    out["inter_dist_m"] = inter_dist
    return out
