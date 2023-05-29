//
// Created by amine on 29.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H
#define VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include "types.h"
#include "utils.h"

double computePlanarityScore(Octree::Ptr &octree, LeafContainerT &leaf, PointT &centroid);

void removePointsFromCloud(PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &inliers);

int removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg, PointCloud::Ptr &cloud);

int removeNonPlanarVoxels(Octree::Ptr &octree, double max_score, PointCloud::Ptr &cloud);

#endif //VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H
