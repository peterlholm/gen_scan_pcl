"Scan stl file from a number of positions"
from pathlib import Path
import copy
import math
import open3d as o3d

BLENDER_LIGHT_POS = (-0.028, 0, 0.0223)
LIGHT_POS =[-0.028, 0, 0.0323]
OBJECT_POS = -0.015

def show_objects(objlist, name=""):
    "Show the object list"
    o3d.visualization.draw_geometries(objlist, window_name=name, width=1000, height=1000, point_show_normal=False, mesh_show_wireframe=False)

def show_objects_test(obj, name=""):
    "Show the object list"
    #print(f"CAM {CAM_POSITION} LOOK_AT {LOOK_AT} UP {UP} ZOOM {ZOOM}")
    o3d.visualization.draw_geometries(obj, window_name=name, width=1000, height=1000,
                                       zoom=ZOOM, front=CAM_POSITION, lookat=LOOK_AT, up=UP)

def create_pointcloud_u(mesh):
    "create pointcloud uniformly"
    pcl = mesh.sample_points_uniformly(10000)
    return pcl

def create_pointcloud_p(mesh):
    "create pointcloud poisson"
    pcl = mesh.sample_points_poisson_disk(10000)
    return pcl

def hide_point(pcl, camera=BLENDER_LIGHT_POS, radius=40):
    "hide points without light"
    #camera = LIGHT_POS
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
    return pcd

def transform(mesh, translate, rotate=(0,0,0)):
    "transform to position"
    mesh_c = copy.deepcopy(mesh)
    mesh_c = mesh_c.translate(translate, relative=True)
    r = mesh.get_rotation_matrix_from_xyz((rotate[0]/180*math.pi, rotate[1]/180*math.pi, rotate[2]/180*math.pi))
    mesh_c.rotate(r)
    return mesh_c

if __name__ == "__main__":
    trans_list = [(-15,0,0),(15,0,0),(0,-15,0),(0,15,0),(0,0,-15),(0,0,15)]

    modelpath = Path("models/tooth.stl")
    mymesh = o3d.io.read_triangle_mesh(str(modelpath))
    if not mymesh.has_triangle_normals():
        mymesh.compute_triangle_normals()
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01,origin=(0,0,0))

    #show_objects([mymesh, coord])
    i = 1
    for t in trans_list:
        obj = transform(mymesh,(0,0,-0.010), t)
        #obj.paint_uniform_color((0,0,1))
        #obj = transform(mymesh,(0, 0, OBJECT_POS))
        # pcl = create_pointcloud(mymesh)
        mypcl = create_pointcloud_p(obj)
        mypcl.paint_uniform_color((0,0,1))
        pcl2 = hide_point(mypcl)
        o3d.io.write_point_cloud("obj_"+str(i)+".ply", pcl2 )
        #show_objects([ obj, pcl2, coord])
        i +=1
    #show_objects([pcl, coord])

    mesh2 = transform(mymesh, [0,0,-0.00], (30/180*math.pi,0,0))
    mesh2.paint_uniform_color((1,0,0))
    mesh2.compute_triangle_normals()
    show_objects([mymesh, pcl2, coord])
                     