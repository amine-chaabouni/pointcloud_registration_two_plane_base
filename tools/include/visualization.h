//
// Created by amine on 30.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_VISUALIZATION_H
#define VOXEL_BASED_REGISTRATION_VISUALIZATION_H

#include "tool_types.h"

void preparePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes,
                          pcl::visualization::PCLVisualizer &viewer, const std::string &name);

void prepareTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud,
                           pcl::visualization::PCLVisualizer &viewer);

void visualizePcd(const PointCloud::ConstPtr &cloud);

void visualizePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes);

void visualizeCorrespondences(const CompleteCloud &, const CompleteCloud &, const Correspondences &);

void visualizeOctree(const PointCloudPtr &cloud, const Octree::Ptr &octree);

void visualizeTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud);

void visualizeBases(const PointCloud::ConstPtr &first_cloud, const std::vector<pcl::Indices> &first_planes,
                    const PointCloud::ConstPtr &second_cloud, const std::vector<pcl::Indices> &second_planes);

#endif //VOXEL_BASED_REGISTRATION_VISUALIZATION_H
