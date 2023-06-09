import tpbr_python as tpbr
import numpy as np
import open3d as o3d


def from_pointT_to_np(pointT):
    return np.array([pointT.x, pointT.y, pointT.z])

def cloud_cpp_to_np(cloud_cpp):
    cloud_np = np.zeros((len(cloud_cpp), 3))
    for i in range(len(cloud_cpp)):
        cloud_np[i] = from_pointT_to_np(cloud_cpp[i])
    return cloud_np

def from_np_to_pointT(np_point):
    return tpbr.PointT(np_point[0], np_point[1], np_point[2])

if __name__ == "__main__":
    print("Hello World!")
    print(dir(tpbr))
    params = tpbr.TwoPlaneBasedRegistrationSolver.Params()
    params.source_cloud_path = "data/data/0001/lidar_0_0_2023-06-05_13-27-28.pcd"
    params.target_cloud_path = "data/data/0001/rgbd_0_1_2023-06-05_13-27-42.pcd"
    params.source_resolution = 0.5
    params.source_min_point_per_voxel = 300

    params.target_resolution = 0.5
    params.target_min_point_per_voxel = 300

    params.planarity_score = 0.6

    params.min_angle = 10 * np.pi / 180
    params.max_angle = 110 * np.pi / 180

    solver = tpbr.TwoPlaneBasedRegistrationSolver(params)
    success, correspondences, transform = solver.solve()
    print("Success: ", success)
    print("Transform: ", transform)
    print("Correspondences: ", correspondences)

    source_points = solver.get_source_cloud()
    target_points = solver.get_target_cloud()

    source_cloud = o3d.geometry.PointCloud()
    source_cloud.points = o3d.utility.Vector3dVector(source_points)
    source_cloud.paint_uniform_color([1, 0, 0])
    source_cloud.transform(transform)

    target_cloud = o3d.geometry.PointCloud()
    target_cloud.points = o3d.utility.Vector3dVector(target_points)
    target_cloud.paint_uniform_color([0, 1, 0])

    o3d.visualization.draw_geometries([source_cloud, target_cloud])
