//
// Created by amine on 29.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_UTILS_H
#define VOXEL_BASED_REGISTRATION_UTILS_H

#include "tool_types.h"
#include <Eigen/SVD>

PointCloudPtr loadPcd(const std::string &pcd_file_path);

Octree toOctree(const PointCloud::ConstPtr &cloud, double resolution);

void checkOctree(const Octree::Ptr &octree);

double angleBetweenVectors(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);

void findRotationBetweenPlanes(const Eigen::MatrixXf &, const Eigen::MatrixXf &, std::shared_ptr<Eigen::Matrix3f> &);

#endif //VOXEL_BASED_REGISTRATION_UTILS_H

