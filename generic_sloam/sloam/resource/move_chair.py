
import open3d as o3d

# load chair.stl and translate along y -1.0 m
chair = o3d.io.read_triangle_mesh("chair_input.stl")
# translate to 0,0,0
chair.translate([-chair.get_center()[0], -chair.get_center()[1], -chair.get_center()[2]])
# visualize origin frame
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([chair, mesh], mesh_show_wireframe=True, mesh_show_back_face=True)
# the new chair using write_triangle_mesh as .stl
# compute normals first
chair.compute_vertex_normals()
o3d.io.write_triangle_mesh("chair.stl", chair)