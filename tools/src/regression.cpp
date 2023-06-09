//
// Created by amine on 09.06.23.
//

#include "regression.h"

CompleteCloud
preparePointCloud(const std::string &cloud_path, double resolution, int min_point_per_voxel, double planarity_score,
                  double min_angle, double max_angle) {
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
executeRegression(const CompleteCloud& source_cloud, const CompleteCloud& target_cloud) {

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


