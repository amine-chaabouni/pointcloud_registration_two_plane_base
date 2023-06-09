#include <iostream>
#include <memory>
#include <string>
#include <filesystem>

#include "regression.h"

#define CLION_DEBUG 0

double min_angle = 10.0 * M_PI / 180.0;
double max_angle = 110.0 * M_PI / 180.0;

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
    if (argc < 7 || argc > 8) {
        std::cerr
                << "ERROR: Syntax is ./main <lidar_resolution> <lidar_min_points_per_voxel> <rgbd_resolution> <rgbd_min_points_per_voxel> <planarity_score> <files_directory>"
                << std::endl;
        std::cerr
                << "or ./main <lidar_resolution> <lidar_min_points_per_voxel> <rgbd_resolution> <rgbd_min_points_per_voxel> <planarity_score> <source pcd file> <target pcd file>"
                << std::endl;
        return -1;
    }

    std::string source_cloud_path, target_cloud_path;

    if (argc == 7) {
        std::string files_directory(argv[6]);
        auto folder_and_files = std::filesystem::directory_iterator(files_directory);
        std::vector<std::filesystem::path> v;

        copy(std::filesystem::directory_iterator(files_directory), std::filesystem::directory_iterator(),
             std::back_inserter(v));

        sort(v.begin(), v.end());

        source_cloud_path = v[0].string();
        target_cloud_path = v[3].string();
    }
    if (argc == 8) {
        source_cloud_path = argv[6];
        target_cloud_path = argv[7];
    }
    double source_resolution = std::atof(argv[1]);
    int source_min_points_per_voxel = std::atof(argv[2]);

    double target_resolution = std::atof(argv[3]);
    int target_min_points_per_voxel = std::atoi(argv[4]);

    double planarity_score = std::atof(argv[5]);
#endif

//    source_cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/gazebo/lidar_0_0.pcd";

    clock_t begin_time = clock();
    auto source_cloud = preparePointCloud(source_cloud_path, source_resolution, source_min_points_per_voxel,
                                          planarity_score, min_angle, max_angle);
    std::cout << "Source cloud processed in : " << float(clock() - begin_time) / CLOCKS_PER_SEC << " seconds"
              << std::endl;


//    target_cloud_path = "/home/amine/nn_i2p/RegTR/data/own_test/gazebo/rgbd_0_1.pcd";
    begin_time = clock();
    auto target_cloud = preparePointCloud(target_cloud_path, target_resolution, target_min_points_per_voxel,
                                          planarity_score, min_angle, max_angle);
    std::cout << "Target cloud processed in : " << float(clock() - begin_time) / CLOCKS_PER_SEC << " seconds"
              << std::endl;

    begin_time = clock();

    auto regression_result = executeRegression(source_cloud, target_cloud);

    std::cout << "Regression computed in : " << float(clock() - begin_time) / CLOCKS_PER_SEC << " seconds"
              << std::endl;

    auto success = std::get<0>(regression_result);
    if (!success) {
        std::cout << "Regression failed" << std::endl;
        return -1;
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


    return 0;
}
