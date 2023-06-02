#include <iostream>
#include <memory>
#include <string>

#include "planarity_extractor.h"
#include "utils.h"
#include "tool_types.h"
#include "visualization.h"
#include "base_processor.h"

#define CLION_DEBUG 0

double min_angle = 10.0 * M_PI / 180.0;
double max_angle = 110.0 * M_PI / 180.0;

using Planes = std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>;
using Bases = std::vector<std::tuple<int, int, double>>;
using CompleteCloud = std::tuple<Octree::Ptr, Planes, Bases>;

CompleteCloud
preparePointCloud(const std::string &cloud_path, double resolution, int min_point_per_voxel, double planarity_score) {
    auto cloud = loadPcd(cloud_path);
    // Convert the point cloud to an octree
    Octree octree = toOctree(cloud, resolution);
    auto octree_ptr = std::make_shared<Octree>(octree);

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    // Remove not needed voxels
    removeVoxelsWithLessThanXPoints(octree_ptr, min_point_per_voxel);
    removeNonPlanarVoxels(octree_ptr, planarity_score);

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    // Generate planes
    auto planes = extractPlane(octree_ptr);
    auto plane_params = planes.first;

#if SHOW_ALL_PLANES
    visualizePlanesOnCloud(cloud, planes.second);
#endif
    // Generate bases
    Bases bases;
    generateBasesFromPlaneParams(plane_params, &bases, min_angle, max_angle);


#if VISUALIZE
    visualizePlanesOnCloud(cloud, planes.second);
#endif

#if DEBUG
    checkOctree(octree_ptr);
#endif

    return std::make_tuple(octree_ptr, planes, bases);
}


int main(int argc, char **argv) {

#if CLION_DEBUG
    std::string cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/multiple_robots/lidar_0_0.pcd";
    double lidar_resolution = 0.5;
    double rgbd_resolution = 0.5;
    int lidar_min_points_per_voxel = 1000;
    int rgbd_ min_points_per_voxel = 1000;
    double planarity_score = 0.6;
#else
    if (argc != 7) {
        std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <lidar_resolution> <lidar_min_points_per_voxel> <rgbd_resolution> <rgbd_min_points_per_voxel> <planarity_score>" << std::endl;
        std::cerr << "EXAMPLE: ./main file_path voxel_size_for_lidar lidar_min_points_per_voxel voxel_size_for_rgbd rgbd_min_points_per_voxel planarity_score" << std::endl;
        return -1;
    }
    std::string cloud_path(argv[1]);
    double lidar_resolution = std::atof(argv[2]);
    int lidar_min_points_per_voxel = std::atof(argv[3]);
    double rgbd_resolution = std::atof(argv[4]);
    int rgbd_min_points_per_voxel = std::atoi(argv[5]);
    double planarity_score = std::atof(argv[6]);
#endif

    cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/gazebo/lidar_0_0.pcd";
    auto first_cloud = preparePointCloud(cloud_path, lidar_resolution, lidar_min_points_per_voxel, planarity_score);
    auto first_octree_ptr = std::get<0>(first_cloud);
    auto first_planes = std::get<1>(first_cloud);
    auto first_bases = std::get<2>(first_cloud);
    std::cout << "First cloud has " << first_planes.first.size() << " planes" << std::endl;
    std::cout << "First cloud has " << first_bases.size() << " bases" << std::endl;

    cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/gazebo/rgbd_0_1.pcd";
    auto second_cloud = preparePointCloud(cloud_path, rgbd_resolution, rgbd_min_points_per_voxel, planarity_score);
    auto second_octree_ptr = std::get<0>(second_cloud);
    auto second_planes = std::get<1>(second_cloud);
    auto second_bases = std::get<2>(second_cloud);
    std::cout << "Second cloud has " << second_planes.first.size() << " planes" << std::endl;
    std::cout << "Second cloud has " << second_bases.size() << " bases" << std::endl;

    visualizeTwoPointClouds(first_octree_ptr->getInputCloud(), second_octree_ptr->getInputCloud());

    auto optimal_correspondence = findOptimalCorrespondences(first_cloud, second_cloud);
    if(optimal_correspondence.size() < 3){
        std::cout << "Not enough correspondences found" << std::endl;
        return -1;
    }

    for (auto corr: optimal_correspondence)
        std::cout << "optimal_correspondence: " << corr.first << " and " << corr.second << std::endl;

    // Visualize Correspondances
    visualizeCorrespondences(first_cloud, second_cloud, optimal_correspondence);

    // Compute the transformation
    std::shared_ptr<Eigen::MatrixXf> transformation(new Eigen::MatrixXf);
    estimateRigidTransformation(first_planes.first, second_planes.first, optimal_correspondence, transformation, true);
    std::cout << "Transformation: " << std::endl << *transformation << std::endl;

    PointCloudPtr transformed_cloud = transformTargetPointCloud(first_octree_ptr->getInputCloud(), *transformation);
    visualizeTwoPointClouds(second_octree_ptr->getInputCloud(), transformed_cloud);

    // Visualize Correspondances
    Octree::Ptr transformed_octree_ptr = std::make_shared<Octree>(toOctree(transformed_cloud, lidar_resolution));
    CompleteCloud transformed_cloud_complete = std::make_tuple(transformed_octree_ptr, first_planes, first_bases);
    visualizeCorrespondences(transformed_cloud_complete, second_cloud, optimal_correspondence);

    return 0;
}
