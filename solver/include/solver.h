//
// Created by amine on 09.06.23.
//

#ifndef VOXEL_BASED_REGISTRATION_SOLVER_H
#define VOXEL_BASED_REGISTRATION_SOLVER_H

#include <iostream>
#include "tool_types.h"
#include "regression.h"

class Solver {
public:
    struct Params {
        std::string source_cloud_path;
        std::string target_cloud_path;
        double source_resolution;
        int source_min_point_per_voxel;
        double target_resolution;
        int target_min_point_per_voxel;
        double planarity_score;
        double min_angle;
        double max_angle;
    };

    Solver() = default;

    Solver(Params params);

    virtual ~Solver() = default;

    virtual std::tuple<bool, Correspondences, Eigen::Matrix4f>
    solve();

    void visualizeSourceCloudWithPlanes();

    void visualizeTargetCloudWithPlanes();

    [[nodiscard]] int getSourceNbPlanes() const;

    [[nodiscard]] int getTargetNbPlanes() const;

    [[nodiscard]] int getSourceNbBases() const;

    [[nodiscard]] int getTargetNbBases() const;

    std::vector<Eigen::Vector3d> getSourceCloud() const {
        auto points = std::get<0>(source_cloud_)->getInputCloud()->points;
        std::vector<Eigen::Vector3d> result;
        for (const auto& point : points) {
            result.emplace_back(point.x, point.y, point.z);
        }
        return result;
    }

    std::vector<Eigen::Vector3d> getTargetCloud() const {
        auto points = std::get<0>(target_cloud_)->getInputCloud()->points;
        std::vector<Eigen::Vector3d> result;
        for (const auto& point : points) {
            result.emplace_back(point.x, point.y, point.z);
        }
        return result;
    }

private:
    CompleteCloud source_cloud_;
    CompleteCloud target_cloud_;
};

#endif //VOXEL_BASED_REGISTRATION_SOLVER_H
