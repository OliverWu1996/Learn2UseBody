import open3d as o3d
import numpy as np
import trimesh


def test():
    #open a stl file with open3d
    mesh = o3d.io.read_triangle_mesh("/home/y/Documents/WYY_META/effectit/ros_kortex/kortex_description/arms/gen3/7dof/meshes/ForeArm_Link.STL")

    # cluster = mesh.cluster_connected_triangles()
    # print(cluster[0])
    # # find the biggest element in cluster[2]
    # max = 0
    # indice = 0
    # for i in range(len(cluster[2])):
    #     if cluster[2][i] > max:
    #         max = cluster[2][i]
    #         indice = i
    
    # print(max)
    # print(indice)
    # print(cluster[0][indice])
    # print(cluster[1][indice])

    # mesh.compute_triangle_normals()
    # # print(mesh_normals)
    # # mesh.paint_uniform_color([0.1, 0.1, 0.7])
    # # #get mesh reference frame
    # # print(mesh.get_rotation_matrix_from_xyz((0, 0, 0)))

    # # #visualize the mesh
    # # o3d.visualization.draw_geometries([mesh])
    # # print(mesh)
    # mesh.create_sphere()
    # o3d.visualization.draw_geometries([mesh])

    # # #get mesh vertices
    mesh_vertices = np.asarray(mesh.vertices)
    print(len(mesh_vertices))
    # # #get mesh faces
    mesh_faces = np.asarray(mesh.triangles)
    print(len(mesh_faces))
    # # #get mesh normals
    mesh.compute_triangle_normals()
    mesh_normals = np.asarray(mesh.triangle_normals)
    print(len(mesh_normals))

    #help me cluster the mesh normals
    # mesh_normals = np.asarray(mesh_normals)
    






if __name__ == "__main__":
    test()
