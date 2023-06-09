//
// Created by amine on 09.06.23.
//

#include "solver.h"

Solver::Solver(Params params) {
    source_cloud_ = preparePointCloud(params.source_cloud_path, params.source_resolution,
                                      params.source_min_point_per_voxel, params.planarity_score,
                                      params.min_angle, params.max_angle);
    target_cloud_ = preparePointCloud(params.target_cloud_path, params.target_resolution,
                                      params.target_min_point_per_voxel, params.planarity_score,
                                      params.min_angle, params.max_angle);
}

std::tuple<bool, Correspondences, Eigen::Matrix4f>
Solver::solve() {
    return executeRegression(source_cloud_, target_cloud_);
}

int Solver::getSourceNbPlanes() const {
    return std::get<1>(source_cloud_).second.size();
}

int Solver::getTargetNbPlanes() const {
    return std::get<1>(target_cloud_).second.size();
}

int Solver::getSourceNbBases() const {
    return std::get<2>(source_cloud_).size();
}

int Solver::getTargetNbBases() const {
    return std::get<2>(target_cloud_).size();
}
