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
    for (int i = 0; i < plane_params.size(); i++) {
        std::cout << i << " nb collinear = " << std::get<2>(plane_params[i]) << std::endl;
    }

#if SHOW_ALL_PLANES
    visualizePlanesOnCloud(cloud, planes.second);
#endif
    // Generate bases
    Bases bases;
    generateBasesFromPlaneParams(plane_params, &bases, min_angle, max_angle);


#if VISUALIZE
    visualizePlanesOnCloud(cloud, planes.second);
#endif

#if HEAVY_DEBUG
    checkOctree(octree_ptr);
#endif

    return std::make_tuple(octree_ptr, planes, bases);
}


std::tuple<bool, Correspondences, Eigen::Matrix4f>
executeRegression(const CompleteCloud source_cloud, const CompleteCloud target_cloud) {

    auto source_octree_ptr = std::get<0>(source_cloud);
    auto source_planes = std::get<1>(source_cloud);
    auto source_bases = std::get<2>(source_cloud);
#if MINIMAL_OUTPUT
    std::cout << "Source cloud has " << source_planes.first.size() << " planes" << std::endl;
    std::cout << "Source cloud has " << source_bases.size() << " bases" << std::endl;
#endif

    auto target_octree_ptr = std::get<0>(target_cloud);
    auto target_planes = std::get<1>(target_cloud);
    auto target_bases = std::get<2>(target_cloud);
#if MINIMAL_OUTPUT
    std::cout << "Target cloud has " << target_planes.first.size() << " planes" << std::endl;
    std::cout << "Target cloud has " << target_bases.size() << " bases" << std::endl;
#endif

    // Compute correspondences
    auto optimal_correspondence = findOptimalCorrespondences(source_cloud, target_cloud);
    if (optimal_correspondence.size() < 3) {
        optimal_correspondence.clear();
        return std::make_tuple(false, optimal_correspondence, Eigen::Matrix4f::Identity());
    }

    // Compute the transformation
    Eigen::Matrix4f transformation;
    estimateRigidTransformation(source_planes.first, target_planes.first, optimal_correspondence, transformation, true);


    return std::make_tuple(true, optimal_correspondence, transformation);
}


int main(int argc, char **argv) {

#if CLION_DEBUG
    std::string source_cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/multiple_robots/lidar_0_0.pcd";
    std::string target_cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/multiple_robots/lidar_0_0.pcd";
    double source_resolution_max = 0.5;
    double target_resolution_max = 0.5;
    int source_min_points_per_voxel_max = 1000;
    int target_min_points_per_voxel_max = 1000;
    double planarity_score_max = 0.6;
#else
    if (argc != 8) {
        std::cerr
                << "ERROR: Syntax is ./main <source pcd file> <target pcd file> <lidar_resolution> <lidar_min_points_per_voxel> <rgbd_resolution> <rgbd_min_points_per_voxel> <planarity_score>"
                << std::endl;
        return -1;
    }
    std::string source_cloud_path(argv[1]);
    std::string target_cloud_path(argv[2]);

    double source_resolution_max = std::atof(argv[3]);
    int source_min_points_per_voxel_max = std::atof(argv[4]);

    double target_resolution_max = std::atof(argv[5]);
    int target_min_points_per_voxel_max = std::atoi(argv[6]);

    double planarity_score_max = std::atof(argv[7]);
#endif

//    source_cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/gazebo/lidar_0_0.pcd";

    std::vector<std::tuple<double, int, double, int, double>> possible_parameters{
            std::make_tuple(source_resolution_max,
                            source_min_points_per_voxel_max,
                            target_resolution_max,
                            target_min_points_per_voxel_max,
                            planarity_score_max),
//            std::make_tuple(0.3,
//                            100,
//                            0.3,
//                            100,
//                            0.6),
//            std::make_tuple(0.4,
//                            50,
//                            0.4,
//                            50,
//                            0.6),
//            std::make_tuple(0.5,
//                            300,
//                            0.5,
//                            300,
//                            0.8),
    };

    CompleteCloud final_source_cloud, final_target_cloud;
    Correspondences final_optimal_correspondence;
    Eigen::Matrix4f final_transformation;
    double resolution = source_resolution_max;

    ulong max_planes = 0;

    for (auto parameters: possible_parameters) {
        double source_resolution = std::get<0>(parameters);
        int source_min_points_per_voxel = std::get<1>(parameters);
        double target_resolution = std::get<2>(parameters);
        int target_min_points_per_voxel = std::get<3>(parameters);
        double planarity_score = std::get<4>(parameters);


        clock_t begin_time = clock();
        auto source_cloud = preparePointCloud(source_cloud_path, source_resolution, source_min_points_per_voxel,
                                              planarity_score);
        std::cout << "Source cloud processed in : " << float(clock() - begin_time) / CLOCKS_PER_SEC << " seconds"
                  << std::endl;


//    target_cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/gazebo/rgbd_0_1.pcd";
        begin_time = clock();
        auto target_cloud = preparePointCloud(target_cloud_path, target_resolution, target_min_points_per_voxel,
                                              planarity_score);
        std::cout << "Target cloud processed in : " << float(clock() - begin_time) / CLOCKS_PER_SEC << " seconds"
                  << std::endl;

        begin_time = clock();

        auto regression_result = executeRegression(source_cloud, target_cloud);

        std::cout << "Regression computed in : " << float(clock() - begin_time) / CLOCKS_PER_SEC << " seconds"
                  << std::endl;

        auto success = std::get<0>(regression_result);
        if (!success) {
            std::cout << "Regression failed" << std::endl;
            continue;
        }
        std::cout << "Regression succeeded" << std::endl;


        auto optimal_correspondence = std::get<1>(regression_result);
        for (auto corr: optimal_correspondence) {
            std::cout << "optimal_correspondence: " << corr.first << " and " << corr.second << std::endl;
        }

        auto transformation = std::get<2>(regression_result);
        std::cout << "Transformation: " << std::endl << transformation << std::endl;

#if VISUALIZE_FINAL_RESULTS
        visualizeFinalResults(source_cloud,
                              target_cloud,
                              optimal_correspondence,
                              transformation,
                              source_resolution);
#endif

        if (optimal_correspondence.size() > max_planes) {
            max_planes = optimal_correspondence.size();
            final_source_cloud = source_cloud;
            final_target_cloud = target_cloud;
            final_optimal_correspondence = optimal_correspondence;
            final_transformation = transformation;
            resolution = source_resolution;
        }
    }

#if VISUALIZE_FINAL_RESULTS
    visualizeFinalResults(final_source_cloud,
                          final_target_cloud,
                          final_optimal_correspondence,
                          final_transformation,
                          resolution);
#endif

    return 0;
}
