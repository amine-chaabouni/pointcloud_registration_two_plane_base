#include <iostream>
#include <memory>
#include <string>

#include "planarity_extractor.h"
#include "utils.h"
#include "tool_types.h"
#include "visualization.h"
#include "base_processor.h"


double min_angle = 10.0 * M_PI / 180.0;
double max_angle = 80.0 * M_PI / 180.0;

using Planes = std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>;
using Bases = std::vector<std::tuple<int, int, double>>;
using CompleteCloud = std::tuple<Octree::Ptr, Planes, Bases>;

CompleteCloud
preparePointCloud(const std::string &cloud_path, double resolution, double planarity_score) {
    auto cloud = loadPcd(cloud_path);
    // Convert the point cloud to an octree
    Octree octree = toOctree(cloud, resolution);
    auto octree_ptr = std::make_shared<Octree>(octree);

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    // Remove not needed voxels
    auto nb_removed = removeVoxelsWithLessThanXPoints(octree_ptr, 10);
    nb_removed = removeNonPlanarVoxels(octree_ptr, planarity_score);

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    // Generate planes
    auto planes = extractPlane(octree_ptr);
    auto plane_params = planes.first;

    // Generate bases
    Bases bases;
    generetaeBasesFromPlaneParams(plane_params, &bases, min_angle, max_angle);


#if VISUALIZE
    visualizePlanesOnCloud(cloud, planes.second);
#endif

#if DEBUG
    checkOctree(octree_ptr);
#endif

    return std::make_tuple(octree_ptr, planes, bases);
}



PointCloudPtr
transformPointCloud(const PointCloud::ConstPtr &cloud, const Eigen::MatrixXf &transformation) {
    PointCloudPtr transformed_cloud(new PointCloud);
    transformed_cloud = cloud->makeShared();
    transformed_cloud->points.clear();

    for (auto point: cloud->points) {
        Eigen::Vector4f point_4d;
        point_4d << point.x, point.y, point.z, 1;
        auto transformed_point = transformation * point_4d;
        transformed_cloud->points.emplace_back(
                pcl::PointXYZ(transformed_point(0), transformed_point(1), transformed_point(2))
        );
    }
    return transformed_cloud;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <resolution>" << std::endl;
        std::cerr << "EXAMPLE: ./main file_path voxel_size planarity_score" << std::endl;
        return -1;
    }
    std::string cloud_path(argv[1]);
    double resolution = std::atof(argv[2]);
    double planarity_score = std::atof(argv[3]);

    cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/multiple_robots/lidar_0_0.pcd";
    auto first_cloud = preparePointCloud(cloud_path, resolution, planarity_score);
    auto first_octree_ptr = std::get<0>(first_cloud);
    auto first_planes = std::get<1>(first_cloud);
    auto first_bases = std::get<2>(first_cloud);
    std::cout << "First cloud has " << first_planes.first.size() << " planes" << std::endl;
    std::cout << "First cloud has " << first_bases.size() << " bases" << std::endl;

    cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/multiple_robots/lidar_0_0_transformed.pcd";
    auto second_cloud = preparePointCloud(cloud_path, resolution, planarity_score);
    auto second_octree_ptr = std::get<0>(second_cloud);
    auto second_planes = std::get<1>(second_cloud);
    auto second_bases = std::get<2>(second_cloud);
    std::cout << "Second cloud has " << second_planes.first.size() << " planes" << std::endl;
    std::cout << "Second cloud has " << second_bases.size() << " bases" << std::endl;

    auto optimal_correspondence = findOptimalCorrespondences(first_cloud, second_cloud);
    for (auto corr: optimal_correspondence)
        std::cout << "optimal_correspondence: " << corr.first << " and " << corr.second << std::endl;

    // Visualize Correspondances
    visualizeCorrespondences(first_cloud, second_cloud, optimal_correspondence);

    // Compute the transformation
    std::shared_ptr<Eigen::MatrixXf> transformation(new Eigen::MatrixXf);
    estimateRigidTransformation(first_planes.first, second_planes.first, optimal_correspondence, transformation);
    std::cout << "Transformation: " << std::endl << transformation << std::endl;

    PointCloudPtr transformed_cloud = transformPointCloud(first_octree_ptr->getInputCloud(), *transformation);
    VisualizeTwoPointClouds(transformed_cloud, second_octree_ptr->getInputCloud());

    return 0;
}
