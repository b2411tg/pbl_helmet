'''
指定の位置周辺の地図ファイルを作成する
'''
import osmnx as ox
G = ox.graph_from_point((35.5381, 139.6897), dist=800, network_type="all")
gdf = ox.graph_to_gdfs(G, nodes=False, edges=True)  # edges = 道路ライン
gdf.to_file("roads.geojson", driver="GeoJSON")
