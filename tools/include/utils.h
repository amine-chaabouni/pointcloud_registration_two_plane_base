//
// Created by amine on 29.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_UTILS_H
#define VOXEL_BASED_REGISTRATION_UTILS_H

#include "types.h"
#include "octree_viewer.h"

PointCloudPtr loadPcd(const std::string &pcd_file_path);

Octree toOctree(const PointCloud::ConstPtr &cloud, double resolution);

void checkOctree(const Octree::Ptr &octree);

void visualizePcd(const PointCloud::ConstPtr &cloud);

void visualizeOctree(const PointCloudPtr &cloud, double resolution);


#endif //VOXEL_BASED_REGISTRATION_UTILS_H

