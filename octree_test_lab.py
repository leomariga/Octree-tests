import open3d as o3d
import numpy as np


def getOctreeStructure(pcd):
    octree = o3d.geometry.Octree(max_depth=5)
    octree.convert_from_point_cloud(pcd, size_expand=0.01)
    return octree
    #pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
    #          center=pcd.get_center())

def getVoxelStructure(pcd):
    return o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)

def getNoisySphere():
    mesh_in = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
    vertices = np.asarray(mesh_in.vertices)
    noise = 0.2
    vertices += np.random.logistic(0,noise, size=vertices.shape)
    mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
    mesh_in.compute_vertex_normals()
    mesh_in.paint_uniform_color([0.1, 0.9, 0.1])
    
    return mesh_in.sample_points_uniformly(number_of_points=2000)


def f_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1
            print(
                "{}{}: Internal node at depth {} has {} children and {} points ({})"
                .format('    ' * node_info.depth,
                        node_info.child_index, node_info.depth, n,
                        len(node.indices), node_info.origin))

            # we only want to process nodes / spatial regions with enough points
            early_stop = len(node.indices) < 250
    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            print("{}{}: Leaf node at depth {} has {} points with origin {}".
                  format('    ' * node_info.depth, node_info.child_index,
                         node_info.depth, len(node.indices), node_info.origin))
    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop

print('input')
N = 2000
pcd = o3d.io.read_point_cloud("dataset/caixa.ply")
#pcd = getNoisySphere()
# fit to unit cube
#pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
#          center=pcd.get_center())
#pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
o3d.visualization.draw_geometries([pcd])

print('octree division')

print('voxelization')
voxel_grid = getVoxelStructure(pcd)
o3d.visualization.draw_geometries([voxel_grid])


octree = getOctreeStructure(pcd)
o3d.visualization.draw_geometries([octree])
octree.traverse(f_traverse)
