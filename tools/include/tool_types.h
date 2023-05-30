//
// Created by amine on 30.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_TOOL_TYPES_H
#define VOXEL_BASED_REGISTRATION_TOOL_TYPES_H

#include "types.h"

#define DEBUG 0
#define VISUALIZE 0
#define SHOW_ALL_BASES 0

using Correspondences = std::vector<std::pair<int, int>>;
using Planes = std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>;
using Bases = std::vector<std::tuple<int, int, double>>;
using CompleteCloud = std::tuple<Octree::Ptr, Planes, Bases>;

#endif //VOXEL_BASED_REGISTRATION_TOOL_TYPES_H
