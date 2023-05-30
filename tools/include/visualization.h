//
// Created by amine on 30.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_VISUALIZATION_H
#define VOXEL_BASED_REGISTRATION_VISUALIZATION_H

#include "tool_types.h"

void visualizePcd(const PointCloud::ConstPtr &cloud);

void visualizePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes);

void visualizeCorrespondences(const CompleteCloud &, const CompleteCloud &, const Correspondences &);

void visualizeOctree(const PointCloudPtr &cloud, const Octree::Ptr &octree);

void VisualizeTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud);

#endif //VOXEL_BASED_REGISTRATION_VISUALIZATION_H
