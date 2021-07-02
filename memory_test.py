import open3d as o3d
import numpy as np
from pympler import asizeof
import sys
import pickle
import cloudpickle

def get_mem_voxel_grid(voxel_grid):
    qtd_voxels = len(voxel_grid.get_voxels())
    info =	{"qtd_voxels": qtd_voxels,
            "mem_size": 16+qtd_voxels*23,
            "mem_size_colorless": 16+qtd_voxels*20
            }
    return info

def get_mem_pcd(pcd):
    info =	{"qtd_points": qtd_root,
            "mem_size": np.asarray(pcd.points).shape[0]*15,
            "mem_size_colorless": np.asarray(pcd.points).shape[0]*12
            }
    return info



def get_mem_octree(octree, method_1=True):
    global qtd_root
    global qtd_internal_node
    global qtd_leaf_node
    qtd_root = 1
    qtd_internal_node = 0
    qtd_leaf_node = 0
    def f_traverse(node, node_info):
        global qtd_root
        global qtd_internal_node
        global qtd_leaf_node
        early_stop = False
        if isinstance(node, o3d.geometry.OctreeInternalNode):
            qtd_internal_node = qtd_internal_node+1
        elif isinstance(node, o3d.geometry.OctreeLeafNode):
            qtd_leaf_node = qtd_leaf_node+1
        else:
            raise NotImplementedError('Node type not recognized!')

        # early stopping: if True, traversal of children of the current node will be skipped
        return early_stop

    octree.traverse(f_traverse)
    info =	{"qtd_root": qtd_root,
            "qtd_internal_node": qtd_internal_node,
            "qtd_leaf_node": qtd_leaf_node
            }
    if method_1:
        info["mem_size"] = qtd_root*84 + qtd_internal_node*64 + qtd_leaf_node*3
        info["mem_size_colorless"] = qtd_root*84 + qtd_internal_node*64 + qtd_leaf_node*1
    else:
        info["mem_size"] = qtd_root*80 + qtd_internal_node*88 + qtd_leaf_node*27
        info["mem_size_colorless"] = qtd_root*80 + qtd_internal_node*88 + qtd_leaf_node*27
    return info

def getOctreeStructure(pcd):
    octree = o3d.geometry.Octree(max_depth=5)
    octree.convert_from_point_cloud(pcd, size_expand=0.01)
    return octree
    #pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
    #          center=pcd.get_center())

def get_voxel_vis_from_octree(octree):
    print("converteu")
    voxel_grid = octree.to_voxel_grid()
    octree_copy = voxel_grid.to_octree(max_depth=5)
    return octree.to_voxel_grid()

def getVoxelStructure(pcd):
    return o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)

def getNoisySphere(N):
    mesh_in = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
    vertices = np.asarray(mesh_in.vertices)
    noise = 0.2
    vertices += np.random.logistic(0,noise, size=vertices.shape)
    mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
    mesh_in.compute_vertex_normals()
    mesh_in.paint_uniform_color([0.1, 0.9, 0.1])
    
    return mesh_in.sample_points_uniformly(number_of_points=N)



print('input')
N = 100000
pcd = getNoisySphere(N)
voxel_grid = getVoxelStructure(pcd)
octree = getOctreeStructure(pcd)

o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([voxel_grid])
o3d.visualization.draw_geometries([octree])

print("Get voxels:")
print(len(voxel_grid.get_voxels()))

print("size pcd: ", get_mem_pcd(pcd))
print("size octree: ", get_mem_octree(octree))
# o3d.io.write_point_cloud("pointcloud.pcd", pcd)
# o3d.io.write_voxel_grid("voxel_grid.ply", voxel_grid)
# o3d.io.write_octree("octree.json", octree)

# pcd2 = o3d.io.read_point_cloud("pointcloud.pcd")
# voxel_grid_2 = o3d.io.read_voxel_grid("voxel_grid.ply")
# octree2 = o3d.io.read_octree("octree.json")

# o3d.visualization.draw_geometries([pcd2])
# o3d.visualization.draw_geometries([voxel_grid_2])
# o3d.visualization.draw_geometries([octree2])