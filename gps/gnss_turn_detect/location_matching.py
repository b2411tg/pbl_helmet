import pandas as pd
from gps.gnss_turn_detect.online_matcher import OfflineSequentialMatcher
import os

PATH = "gps_log_neo_f10n_20251021_213516.csv"

os.chdir("./GPS")
roads = "roads.geojson"
matcher = OfflineSequentialMatcher(roads_geojson=roads, search_m=12.0, switch_penalty_m=30.0,
                                   w_dist=1.0, w_head=0.4, w_prog=0.6)

df = pd.read_csv(PATH)  # 列: UTC, latitude, longitude
rows = []
prev = None
for idx, r in df.iterrows():
    st = matcher.update(lat=float(r["latitude"]), lon=float(r["longitude"]),
                        t=float(r["UTC"]), gps_heading=None, prev=prev, vmax_mps=15.0)
    if st is None:
        rows.append([r["UTC"], "", ""])
    else:
        rows.append([r["UTC"], f"{st.lat:.6f}", f"{st.lon:.6f}"])
        prev = st
        print(f'{r["UTC"]},{st.lat:.6f},{st.lon:.6f}')

out = pd.DataFrame(rows, columns=["UTC","latitude","longitude"])
out.to_csv("matched_utc_lat_lon.csv", index=False)
