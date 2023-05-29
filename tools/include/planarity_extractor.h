//
// Created by amine on 29.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H
#define VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "../../custom_lib/include/types.h"

std::pair<Eigen::VectorXf, pcl::Indices> refinePlane(PointCloudPtr &plane, double distance_threshold = 0.1, double probability = 0.80, int max_iter = 5000);

std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>> extractPlane(Octree::Ptr &octree);

double computePlanarityScore(Octree::Ptr &octree, LeafContainerT &leaf, PointT &centroid);

void removePointsFromCloud(PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &inliers);

int removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg);

int removeNonPlanarVoxels(Octree::Ptr &octree, double max_score);

#endif //VOXEL_BASED_REGISTRATION_PLANARITY_EXTRACTOR_H
