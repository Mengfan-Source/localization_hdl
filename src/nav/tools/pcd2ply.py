import os
import open3d as o3d

map_pcd_path = str(os.path.dirname(os.path.abspath(__file__))) + "/file.pcd"
map_ply_path = str(os.path.dirname(os.path.abspath(__file__))) + "/file.ply"
map_pcd_data = o3d.io.read_point_cloud(map_pcd_path)
o3d.visualization.draw_geometries([map_pcd_data])

o3d.io.write_point_cloud(map_ply_path, map_pcd_data)