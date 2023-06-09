//
// Created by amine on 09.06.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_REGRESSION_H
#define VOXEL_BASED_REGISTRATION_REGRESSION_H

#include <iostream>
#include <memory>
#include <string>

#include "planarity_extractor.h"
#include "utils.h"
#include "tool_types.h"
#include "visualization.h"
#include "base_processor.h"

double min_angle = 10.0 * M_PI / 180.0;
double max_angle = 110.0 * M_PI / 180.0;

using Planes = std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>;
using Bases = std::vector<std::tuple<int, int, double>>;
using CompleteCloud = std::tuple<Octree::Ptr, Planes, Bases>;

CompleteCloud
preparePointCloud(const std::string &cloud_path, double resolution, int min_point_per_voxel, double planarity_score);

std::tuple<bool, Correspondences, Eigen::Matrix4f>
executeRegression(const CompleteCloud& source_cloud, const CompleteCloud& target_cloud);

#endif //VOXEL_BASED_REGISTRATION_REGRESSION_H
