//
// Created by amine on 29.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_UTILS_H
#define VOXEL_BASED_REGISTRATION_UTILS_H

#include "types.h"

PointCloudPtr loadPcd(const std::string &pcd_file_path);

Octree toOctree(const PointCloud::ConstPtr &cloud, double resolution);

void checkOctree(const Octree::Ptr &octree);

void visualizePcd(const PointCloud::ConstPtr &cloud);

void visualizePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes);

void visualizeOctree(const PointCloudPtr &cloud, const Octree::Ptr &octree);

double angleBetweenVectors(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);

#endif //VOXEL_BASED_REGISTRATION_UTILS_H

