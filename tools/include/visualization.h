//
// Created by amine on 30.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_VISUALIZATION_H
#define VOXEL_BASED_REGISTRATION_VISUALIZATION_H

#include "tool_types.h"

/**
 * @brief Add a vector representing the normal of a plane
 * @param point_on_plane The point on the plane
 * @param normal The normal of the plane
 * @param viewer The viewer
 * @param name The name of the vector
 */
void
addNormals(const PointT &point_on_plane, Eigen::Vector3f &normal, pcl::visualization::PCLVisualizer &viewer,
           const std::string &name);

/**
 * @brief Add a cloud to the viewer as well as its planes
 * @details Not all the planes are visualized, only those that are in the vector of indices. The color of the planes go from red to blue
 * @param cloud
 * @param planes
 * @param viewer
 * @param name
 */
void
preparePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes,
                     pcl::visualization::PCLVisualizer &viewer, const std::string &name);


/**
 * @brief Add two clouds to the viewer
 * @param first_cloud
 * @param second_cloud
 * @param viewer
 */
void
prepareTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud,
                           pcl::visualization::PCLVisualizer &viewer);


/**
 * @brief Visualize a point cloud
 * @param cloud
 */
void
visualizePcd(const PointCloud::ConstPtr &cloud);


/**
 * @brief Visualize a point cloud with its planes
 * @param cloud
 * @param planes
 */
void
visualizePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes);


/**
 * @brief Visualize two point clouds as well as the correspondences between their planes
 * @param first_cloud
 * @param second_cloud
 * @param correspondences
 */
void
visualizeCorrespondences(const CompleteCloud &, const CompleteCloud &, const Correspondences &);


/**
 * @brief Visualize a point cloud with its octree structure
 * @param cloud
 * @param octree
 */
void
visualizeOctree(const PointCloudPtr &cloud, const Octree::Ptr &octree);


/**
 * @brief Visualize two point clouds
 * @param first_cloud
 * @param second_cloud
 */
void
visualizeTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud);


/**
 * @brief Visualize two point clouds with a chose bases and their normals @see addNormals
 * @param first_cloud
 * @param first_planes
 * @param second_cloud
 * @param second_planes
 */
void
visualizeBases(const PointCloud::ConstPtr &first_cloud, const Planes &first_planes,
                    const PointCloud::ConstPtr &second_cloud, const Planes &second_planes);

#endif //VOXEL_BASED_REGISTRATION_VISUALIZATION_H
