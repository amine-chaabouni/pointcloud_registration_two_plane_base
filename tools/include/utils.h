//
// Created by amine on 29.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_UTILS_H
#define VOXEL_BASED_REGISTRATION_UTILS_H

#include "tool_types.h"
#include <Eigen/SVD>


/**
 * @brief Load a point cloud from a pcd file
 * @param pcd_file_path
 * @return a point cloud
 */
PointCloudPtr
loadPcd(const std::string &pcd_file_path);


/**
 * @brief Convert a point cloud to an octree
 * @param pcd_file_path
 * @return an octree
 */
Octree
toOctree(const PointCloud::ConstPtr &cloud, double resolution);


/**
 * @brief Check the voxels in an octree
 * @details This function is used for debugging
 * @param octree
 */
void
checkOctree(const Octree::Ptr &octree);


/**
 * @brief Calculate the angle between two vectors
 * @param v1
 * @param v2
 * @return the angle between the vectors
 */
double
angleBetweenVectors(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);


/**
 * @brief Find the rotation between two planes
 * @details Use an SVD method to find the rotation between two planes
 * This function can be used to find a rotation around the z axis or in the space
 * @param source_normals normals of the source plane concatenated in a matrix
 * @param target_normals normals of the target plane concatenated in a matrix
 * @param[out] R rotation matrix
 */
void
findRotationBetweenPlanes(const Eigen::MatrixXf &, const Eigen::MatrixXf &, std::shared_ptr<Eigen::Matrix3f> &);


/**
 * @brief transform a point cloud using a transformation matrix
 * @param cloud
 * @param transformation
 * @return The transformed point cloud
 */
PointCloudPtr
transformSourcePointCloud(const PointCloud::ConstPtr &cloud, const Eigen::MatrixXf &transformation);


/**
 * @brief Rotate a point cloud using a rotation matrix
 * @param cloud
 * @param rotation
 * @return The rotated point cloud
 */
PointCloudPtr
rotatePointCloud(const PointCloud::ConstPtr &cloud, const Eigen::Matrix3f &rotation);

#endif //VOXEL_BASED_REGISTRATION_UTILS_H

