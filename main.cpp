#include <iostream>
#include <memory>
#include <string>

#include "types.h"
#include "utils.h"
#include "planarity_extractor.h"


#define DEBUG 0
#define VISUALIZE 1



int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <resolution>" << std::endl;
        std::cerr << "EXAMPLE: ./octreeVisu bun0.pcd 0.001" << std::endl;
        return -1;
    }
    std::string cloud_path(argv[1]);
    double resolution = std::atof(argv[2]);

    // Load the point cloud
    auto cloud = loadPcd(cloud_path);

    // Visualize the point cloud as octreeq
#if VISUALIZE
    visualizeOctree(cloud, resolution);
#endif


    std::cout << "Number of points = " << cloud->size() << std::endl;

    // Convert the point cloud to an octree
    Octree octree = toOctree(cloud, resolution);
    auto octree_ptr = std::make_shared<Octree>(octree);

    std::cout << "Number of voxels = " << octree_ptr->getLeafCount() << std::endl;

    // Remove low populated voxels
    auto nb_removed = removeVoxelsWithLessThanXPoints(octree_ptr, 10, cloud);
    std::cout << "Number of removed voxels = " << nb_removed << std::endl;

    nb_removed = removeNonPlanarVoxels(octree_ptr, 0.03, cloud);
    std::cout << "Number of removed voxels = " << nb_removed << std::endl;
    std::cout << "Number of voxels = " << octree_ptr->getLeafCount() << std::endl;
    std::cout << "Number of points = " << cloud->size() << std::endl;
#if VISUALIZE
    visualizeOctree(cloud, resolution);
#endif

    auto planes = extractPlane(octree_ptr, cloud);

#if VISUALIZE
    visualizeOctree(cloud, resolution);
#endif

#if DEBUG
    checkOctree(octree_ptr);
#endif


    return 0;
}
