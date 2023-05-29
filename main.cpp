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

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <resolution>" << std::endl;
        std::cerr << "EXAMPLE: ./main file_path voxel_size planarity_score" << std::endl;
        return -1;
    }
    std::string cloud_path(argv[1]);
    double resolution = std::atof(argv[2]);
    double planarity_score = std::atof(argv[3]);

    // Load the point cloud
    auto cloud = loadPcd(cloud_path);


    std::cout << "Number of points = " << cloud->size() << std::endl;

    // Convert the point cloud to an octree
    Octree octree = toOctree(cloud, resolution);
    auto octree_ptr = std::make_shared<Octree>(octree);

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    std::cout << "Number of voxels = " << octree_ptr->getLeafCount() << std::endl;

    // Remove low populated voxels
    auto nb_removed = removeVoxelsWithLessThanXPoints(octree_ptr, 10, cloud);
    std::cout << "Number of removed voxels = " << nb_removed << std::endl;

    nb_removed = removeNonPlanarVoxels(octree_ptr, planarity_score, cloud);
    std::cout << "Number of removed voxels = " << nb_removed << std::endl;
    std::cout << "Number of voxels = " << octree_ptr->getLeafCount() << std::endl;
    std::cout << "Number of points = " << cloud->size() << std::endl;
#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    auto planes = extractPlane(octree_ptr, cloud);
    std::cout << "Number of planes = " << planes.size() << std::endl;

#if VISUALIZE
    visualizeOctree(cloud, octree_ptr);
#endif

    std::vector <std::pair<int, int>> bases;
    auto nb_planes = planes.size();
    for (int i = 0; i < nb_planes; i++) {
        for (int j = i + 1; j < nb_planes; j++) {
            auto plane1 = planes[i];
            auto plane2 = planes[j];
            auto angle = angleBetweenVectors(plane1.first, plane2.first);
            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            bases.emplace_back(i, j);
        }
    }

    std::cout << "Number of bases = " << bases.size() << std::endl;

#if DEBUG
    checkOctree(octree_ptr);
#endif


    return 0;
}
