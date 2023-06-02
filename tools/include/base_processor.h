//
// Created by amine on 30.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_BASE_PROCESSOR_H
#define VOXEL_BASED_REGISTRATION_BASE_PROCESSOR_H

#include "tool_types.h"
#include "utils.h"
#include "visualization.h"


/**
 * @brief Check if the correspondences are valid
 * @details Check if the correspondences are valid by checking if there are at least three planes and if these planes form a base in the space (three non-collinear planes)
 * @param source_planes
 * @param target_planes
 * @param final_correspondence
 * @return true if the correspondences are valid, false otherwise
 */
bool
correspondencesValid(const std::vector<PlaneParam> &source_planes, const std::vector<PlaneParam> &target_planes,
                     const Correspondences &final_correspondence);


/**
 * @brief Generate bases from plane parameters
 * @details Generate bases of two planes with an angle between min_angle and max_angle
 * @param plane_params
 * @param bases
 * @param min_angle
 * @param max_angle
 */
void
generateBasesFromPlaneParams(const std::vector<PlaneParam> &plane_params, Bases *bases, double min_angle,
                             double max_angle);


/**
 * @brief Process a base pair
 * @details Process a base pair by computing the LCP, the correspondences and the rotation matrix
 * The process goes as follow:
 * 1. Given a source base (i, j) and a target base (k, l), compute the LCP and the correspondences with i->h and j->k
 * 2. Repeat with i->k and j->h
 * @param source_planes
 * @param target_planes
 * @param source_base_pair
 * @param target_base_pair
 * @return a tuple containing the LCP, the correspondences and the rotation matrix
 */
std::tuple<double, std::vector<std::pair<int, int>>, Eigen::Matrix3f>
processBasePair(const std::vector<PlaneParam> &source_planes,
                const std::vector<PlaneParam> &target_planes,
                const std::pair<int, int> &source_base_pair,
                const std::pair<int, int> &target_base_pair);


/**
 * @brief Identify the correspondences between source planes and target planes
 * @details Identify the correspondences between source planes and target planes by checking
 * 1. For a plane in the source cloud, find the most collinear plane in the target cloud
 * 2. for that particular target plane, if the same source plane is the most collinear, then add it to the correspondences
 * 3. Otherwise, the source plane doesn't have a corresponding target plane
 *
 * The process of finding the most collinear plane is done in @see computeNormalDistances
 * @param source_correspondences
 * @param target_correspondences
 * @return A list of correspondences between the source and target planes
 */
Correspondences
identifyPlaneCorrespondences(std::vector<std::pair<int, double>> &source_correspondences,
                             std::vector<std::pair<int, double>> &target_correspondences);


/**
 * @brief Estimate the translation parameters
 * @details Estimate the translation parameters by computing the average translation between the source and target planes
 * The mathematical formula is as follow:
 * target_normal.T * translation = (source_distance - target_distance)
 *
 * The geometric interpretation is as follow:
 * For two corresponding planes, the distance between them, considering that they are collinear, is the difference of distance between the origin and the source plane and between the origin and the target plane
 * We are looking for the translation, in the direction of the normal of the planes (since they are the same - collinear), that will bring the source plane to the target plane
 * The left side of the formula [target_normal.T * translation] is the projection of the translation vector on the normal of the target plane.
 *
 * @param source_planes
 * @param target_planes
 * @param plane_correspondences correspondences between the source and target planes
 * @param[out] translation
 */
void
estimateTranslationParameters(const std::vector<PlaneParam> &source_planes,
                              const std::vector<PlaneParam> &target_planes,
                              const Correspondences &plane_correspondences,
                              std::shared_ptr<Eigen::Vector3f> translation);

/**
 * @brief Compute the closest planes between two clouds
 * @details For each plane in the source, use their normals to find the most collinear plane in the target cloud, and vice versa
 * The process goes as follow:
 * 1. For each plane in the source cloud, compute the distance between its normal and the normals of the target planes
 * 2. For each plane in the target cloud, compute the distance between its normal and the normals of the source planes
 * 3. For each plane in the source cloud, find the most collinear plane in the target cloud
 * 4. For each plane in the target cloud, find the most collinear plane in the source cloud
 *
 * @param source_planes
 * @param target_planes
 * @param rotation used to rotated the source planes to the target planes in order for the planes to be collinear
 * @return
 */
std::pair<std::vector<std::pair<int, double>>, std::vector<std::pair<int, double>>>
computeNormalDistances(const std::vector<PlaneParam> &source_planes,
                       const std::vector<PlaneParam> &target_planes,
                       const Eigen::Matrix3f &rotation);


/**
 * @brief Find the optimal correspondences between two clouds
 * @details Find the optimal correspondences between two clouds by computing the LCP for each base pair @see processBasePair
 * @param first_cloud
 * @param second_cloud
 * @return
 */
Correspondences
findOptimalCorrespondences(const CompleteCloud &first_cloud,
                           const CompleteCloud &second_cloud);


/**
 * @brief Estimate the rigid transformation between two clouds
 * @details Estimate the rigid transformation between two clouds by computing the rotation matrix and the translation vector
 * @param source_planes
 * @param target_planes
 * @param correspondences list of correspondences between the source and target planes. The transformation should transform one source plane to its corresponding target plane
 * @param[out] transformation transformation matrix from source to target
 * @param with_translation decide if the translation should be computed
 */
void
estimateRigidTransformation(const std::vector<PlaneParam> &source_planes,
                            const std::vector<PlaneParam> &target_planes,
                            const Correspondences &correspondences,
                            std::shared_ptr<Eigen::MatrixXf> &transformation,
                            bool with_translation);

#endif //VOXEL_BASED_REGISTRATION_BASE_PROCESSOR_H
