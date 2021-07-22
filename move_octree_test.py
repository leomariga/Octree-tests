import open3d as o3d
import numpy as np
from pympler import asizeof
from aux.aux_octree import *
import sys
import pickle
import cloudpickle

def getNoisySphere():
    mesh_in = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
    mesh_in.paint_uniform_color([0.1, 0.9, 0.1])
    
    return mesh_in.sample_points_uniformly(number_of_points=5000)

print('input')
N = 2000
pcd = o3d.io.read_point_cloud("dataset/caixa.ply")

o3d.visualization.draw_geometries([pcd])

print('octree division')


octree = pcd_to_octree(pcd)
o3d.visualization.draw_geometries([octree])

o_pcd = octree_to_pcd(octree, 50)
o3d.visualization.draw_geometries([o_pcd])

pcd_shere = getNoisySphere()
# fit to unit cube
pcd_shere.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
pcd_shere.paint_uniform_color([0.1, 0.9, 0.1])
o3d.visualization.draw_geometries([pcd_shere, octree])

o3d.visualization.draw_geometries([append_pcd_to_octree(pcd_shere, octree)])