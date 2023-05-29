#include <iostream>
#include <memory>
#include <string>

//#include "types.h"
#include "planarity_extractor.h"
#include "utils.h"
#include "custom_lib/include/types.h"


#define DEBUG 0
#define VISUALIZE 1

double min_angle = 10.0 * M_PI / 180.0;
double max_angle = 80.0 * M_PI / 180.0;

using Planes = std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>;
using Bases = std::vector<std::pair<int, int>>;

std::tuple<Octree::Ptr, Planes, Bases>
preparePointCloud(const std::string &cloud_path, double resolution, double planarity_score) {
    auto cloud = loadPcd(cloud_path);
    // Convert the point cloud to an octree
    Octree octree = toOctree(cloud, resolution);
    auto octree_ptr = std::make_shared<Octree>(octree);

    // Remove not needed voxels
    auto nb_removed = removeVoxelsWithLessThanXPoints(octree_ptr, 10);
    nb_removed = removeNonPlanarVoxels(octree_ptr, planarity_score);

    // Generate planes
    auto planes = extractPlane(octree_ptr);
    auto plane_params = planes.first;

    // Generate bases
    std::vector<std::pair<int, int>> bases;
    auto nb_planes = plane_params.size();
    for (int i = 0; i < nb_planes; i++) {
        for (int j = i + 1; j < nb_planes; j++) {
            auto plane1 = plane_params[i];
            auto plane2 = plane_params[j];
            auto angle = angleBetweenVectors(plane1.first, plane2.first);
            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            bases.emplace_back(i, j);
        }
    }

    return std::make_tuple(octree_ptr, planes, bases);

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

#if VISUALIZE
    visualizePlanesOnCloud(cloud, planes.second);
#endif

#if DEBUG
    checkOctree(octree_ptr);
#endif
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

    auto first_cloud = preparePointCloud(cloud_path, resolution, planarity_score);
    auto octree_ptr = std::get<0>(first_cloud);
    auto planes = std::get<1>(first_cloud);
    auto bases = std::get<2>(first_cloud);


    return 0;
}
