import osmnx as ox

POINT = (35.5381, 139.6897)
DIST = 800

# 対象となる場所の道路ネットワークを取得
G = ox.graph_from_point(POINT, dist=DIST, network_type="drive") 

# ノード情報（交差点）をGeoDataFrameに変換
nodes = ox.graph_to_gdfs(G, edges=False)

# 地図ファイル作成
intersections = nodes[nodes["street_count"] >= 3][["geometry"]]
intersections.to_file("intersection_nodes.geojson", driver="GeoJSON")

# 交差点データ表示
for row in intersections.geometry:
    print(f"{row.y},{row.x}")

