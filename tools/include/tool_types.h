//
// Created by amine on 30.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_TOOL_TYPES_H
#define VOXEL_BASED_REGISTRATION_TOOL_TYPES_H

#include "types.h"

#define MINIMAL_OUTPUT 0
#define DEBUG 0
#define VISUALIZE 0
#define VISUALIZE_FINAL_RESULTS 0
#define SHOW_ALL_PLANES 0
#define SHOW_ALL_BASES 0
#define SHOW_ROTATION 0
#define ROTATION_ON_Z 1

using Correspondences = std::vector<std::pair<int, int>>;
using Planes = std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>;
using Bases = std::vector<std::tuple<int, int, double>>;
using CompleteCloud = std::tuple<Octree::Ptr, Planes, Bases>;

#endif //VOXEL_BASED_REGISTRATION_TOOL_TYPES_H
