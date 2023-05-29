//
// Created by amine on 29.05.23.
//
#pragma once

#ifndef VOXEL_BASED_REGISTRATION_TYPES_H
#define VOXEL_BASED_REGISTRATION_TYPES_H

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include "octree_voxel_container.h"
#include "octree_search_centroid.h"

using PointT = pcl::PointXYZ;
using LeafContainerT = pcl::octree::OctreeVoxelContainer<PointT>;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using Octree = pcl::octree::OctreeVoxelBasedRegistration<PointT, LeafContainerT>;
#endif //VOXEL_BASED_REGISTRATION_TYPES_H
