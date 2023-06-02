//
// Created by amine on 29.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H
#define VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "../../custom_lib/include/types.h"


/**
 * @brief Compute the plane parameters
 * @details Compute the plane parameters (a, b, c, d) from a point cloud using RANSAC
 * @param plane
 * @param distance_threshold
 * @param probability
 * @param max_iter
 * @return the plane parameters and the inliers (indices of the points that belong to the plane)
 */
std::pair<Eigen::VectorXf, pcl::Indices>
refinePlane(PointCloudPtr &plane, double distance_threshold = 0.1, double probability = 0.80, int max_iter = 5000);


/**
 * @brief Generate planes from a point cloud
 * @param octree
 * @return plane parameters and the indices of the points that belong to the plane in the point cloud
 */
std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>
extractPlane(Octree::Ptr &octree);


/**
 * @brief Compute the planarity score of a voxel
 * @details Compute the planarity score of a voxel using the eigenvalues of the covariance matrix
 * mathematically:
 * \f$ \frac{\lambda_3}{\lambda_1 + \lambda_2 + \lambda_3} \f$
 * where \f$ \lambda_1, \lambda_2, \lambda_3 \f$ are the eigenvalues of the covariance matrix in increasing order
 * @param octree
 * @param indices
 * @param centroid
 * @return planarity score
 */
double
computePlanarityScore(Octree::Ptr &octree, pcl::Indices &indices, PointT &centroid);


/**
 * @brief Remove points from a point cloud
 * @param cloud
 * @param inliers
 */
void
removePointsFromCloud(PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &outliers);


/**
 * @brief Remove voxels with less than a given number of points
 * @param octree
 * @param min_points_per_voxel_arg
 * @return number of deleted voxels
 */
int
removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg);


/**
 * @brief Remove non planar voxels
 * @details Remove non planar voxels from a point cloud using the planarity score
 * @param octree
 * @param max_score
 * @return number of deleted voxels
 */
int
removeNonPlanarVoxels(Octree::Ptr &octree, double max_score);

#endif //VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H
