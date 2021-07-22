import open3d as o3d
import numpy as np

def get_mem_voxel_grid(voxel_grid, method="open3d"):
    #memq = voxel_grid.get_mem_size()
    if method == "open3d":
        qtd_voxels = len(voxel_grid.get_voxels())
        qtd_bucket = int(qtd_voxels/0.7)
        info =	{"qtd_voxels": qtd_voxels,
                "qtd_buckets": qtd_bucket,
                "grid_size": 144,
                "bucket_size": qtd_bucket*16,
                "voxel_size": qtd_voxels*23,
                "mem_size":qtd_voxels*23 + qtd_bucket*16 +144,
                "mem_size_colorless":qtd_voxels*20 + qtd_bucket*16 +144
                }
    else:
        grid_size = np.asarray(voxel_grid.get_max_bound()-voxel_grid.get_min_bound())
        qtd_cells_space = np.ceil(grid_size/voxel_grid.voxel_size)
        qtd_cells = qtd_cells_space[0]*qtd_cells_space[1]*qtd_cells_space[2]
        info =	{"qtd_voxels": qtd_cells,
                "mem_size":qtd_cells*16 + 24,
                "mem_size_colorless":qtd_cells*13 + 24,
                }
    return info

def get_mem_feature(feature):
    if feature == "plane":
        info =	{"mem_size": 35,
                 "mem_size_colorless": 32
                }
    elif feature == "cylinder":
        info =	{"mem_size": 23,
                 "mem_size_colorless": 20
                }
    elif feature == "cuboid":
        info =	{"mem_size": 27,
                 "mem_size_colorless": 24
                }
    elif feature == "sphere":
        info =	{"mem_size": 19,
                 "mem_size_colorless": 16
                }
    return info


def get_mem_pcd(pcd):
    info =	{"qtd_points": np.asarray(pcd.points).shape[0],
            "mem_size": np.asarray(pcd.points).shape[0]*15,
            "mem_size_colorless": np.asarray(pcd.points).shape[0]*12
            }
    return info



def get_mem_octree(octree, method="open3d"):
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
    if method == "open3d" :
        info["mem_size"] = qtd_root*84 + qtd_internal_node*64 + qtd_leaf_node*3
        info["mem_size_colorless"] = qtd_root*84 + qtd_internal_node*64 + qtd_leaf_node*1
    else:
        info["mem_size"] = qtd_root*80 + qtd_internal_node*88 + qtd_leaf_node*27
        info["mem_size_colorless"] = qtd_root*80 + qtd_internal_node*88 + qtd_leaf_node*1
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

# print("Get voxels:")
# print(len(voxel_grid.get_voxels()))

print("size pcd: ", get_mem_pcd(pcd)['mem_size']/1024)

print("size voxel grid (traditional): ", get_mem_voxel_grid(voxel_grid,"traditional")['mem_size']/1024)
print("size voxel grid (open3d): ", get_mem_voxel_grid(voxel_grid,"open3d")['mem_size']/1024)
print("size octree (traditional): ", get_mem_octree(octree, "traditional")['mem_size']/1024)
print("size octree (open3d): ", get_mem_octree(octree, "open3d")['mem_size']/1024)

# o3d.io.write_point_cloud("pointcloud.pcd", pcd)
# o3d.io.write_voxel_grid("voxel_grid.ply", voxel_grid)
# o3d.io.write_octree("octree.json", octree)

# pcd2 = o3d.io.read_point_cloud("pointcloud.pcd")
# voxel_grid_2 = o3d.io.read_voxel_grid("voxel_grid.ply")
# octree2 = o3d.io.read_octree("octree.json")

# o3d.visualization.draw_geometries([pcd2])
# o3d.visualization.draw_geometries([voxel_grid_2])
# o3d.visualization.draw_geometries([octree2])