//
// Created by amine on 30.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_BASE_PROCESSOR_H
#define VOXEL_BASED_REGISTRATION_BASE_PROCESSOR_H

#include "tool_types.h"
#include "utils.h"

void
generetaeBasesFromPlaneParams(const std::vector<PlaneParam> &plane_params, Bases *bases, double min_angle,
                              double max_angle);

std::pair<double, std::vector<std::pair<int, int>>>
processBasePair(const std::vector<PlaneParam> &source_planes,
                const std::vector<PlaneParam> &target_planes,
                const std::pair<int, int> &source_base_pair,
                const std::pair<int, int> &target_base_pair);

Correspondences
identifyPlaneCorrespondences(std::vector<std::pair<int, double>> &source_correspondences,
                             std::vector<std::pair<int, double>> &target_correspondences);

void
estimateTranslationParameters(const std::vector<PlaneParam> &source_planes,
                              const std::vector<PlaneParam> &target_planes,
                              const Correspondences &plane_correspondences,
                              std::shared_ptr<Eigen::Vector3f> translation);

std::pair<std::vector<std::pair<int, double>>, std::vector<std::pair<int, double>>>
computeNormalDistances(const std::vector<PlaneParam> &source_planes,
                       const std::vector<PlaneParam> &target_planes,
                       const Eigen::Matrix3f &rotation);

Correspondences
findOptimalCorrespondences(const CompleteCloud &first_cloud,
                           const CompleteCloud &second_cloud);

void
estimateRigidTransformation(const std::vector<PlaneParam> &source_planes,
                            const std::vector<PlaneParam> &target_planes,
                            const Correspondences &correspondences,
                            std::shared_ptr<Eigen::MatrixXf> &transformation,
                            bool with_translation);

#endif //VOXEL_BASED_REGISTRATION_BASE_PROCESSOR_H
