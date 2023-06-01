//
// Created by amine on 30.05.23.
//

#include "base_processor.h"

#include <utility>

bool
correspondencesValid(const std::vector<PlaneParam> &source_planes, const std::vector<PlaneParam> &target_planes,
                     const Correspondences &final_correspondence) {
    /*
     * Correspondences are valid if there are at least three non-collinear planes
     * An angle of PI counts as collinear
     */
    std::vector<int> source_possible_three_base_planes;
    std::vector<int> target_possible_three_base_planes;

    source_possible_three_base_planes.emplace_back(final_correspondence[0].first);
    target_possible_three_base_planes.emplace_back(final_correspondence[0].second);

    for (int i = 1; i < final_correspondence.size(); i++) {
        auto correspondence = final_correspondence[i];
        auto source_plane = source_planes[correspondence.first];
        int possible_new_source = 0;

        for (auto plane_idx: source_possible_three_base_planes) {
            auto angle = angleBetweenVectors(source_plane.first, source_planes[plane_idx].first);
            if (angle > (5.0 * M_PI / 180.0) && angle < (175.0 * M_PI / 180.0)) {
                possible_new_source++;
            }
        }
        int size_base = source_possible_three_base_planes.size();
        if (possible_new_source >= std::min(2, size_base)) {
            source_possible_three_base_planes.emplace_back(correspondence.first);
        }

        auto target_plane = target_planes[correspondence.second];
        int possible_new_target = 0;
        for (auto plane_idx: target_possible_three_base_planes) {
            auto angle = angleBetweenVectors(target_plane.first, target_planes[plane_idx].first);
            if (angle > (5.0 * M_PI / 180.0) && angle < (175.0 * M_PI / 180.0)) {
                possible_new_target++;
            }
        }
        size_base = target_possible_three_base_planes.size();
        if (possible_new_target >=  std::min(2, size_base)) {
            target_possible_three_base_planes.emplace_back(correspondence.second);
        }
    }
    return (source_possible_three_base_planes.size() >= 3 && target_possible_three_base_planes.size() >= 3);
}

void
generateBasesFromPlaneParams(const std::vector<PlaneParam> &plane_params, Bases *bases, double min_angle,
                             double max_angle) {
    auto nb_planes = plane_params.size();
    for (int i = 0; i < nb_planes; i++) {
        for (int j = i + 1; j < nb_planes; j++) {
            auto plane1 = plane_params[i];
            auto plane2 = plane_params[j];
            auto angle = angleBetweenVectors(plane1.first, plane2.first);
            if (angle > min_angle && angle < max_angle) {
                (*bases).emplace_back(i, j, angle);
            }
        }
    }
}

Correspondences
identifyPlaneCorrespondences(std::vector<std::pair<int, double>> &source_correspondences,
                             std::vector<std::pair<int, double>> &target_correspondences) {
    std::vector<std::pair<int, int>> plane_correspondence;
    for (int i = 0; i < source_correspondences.size(); i++) {
        auto source_correspondence = source_correspondences[i];
        auto target_correspondence = target_correspondences[source_correspondence.first];
        if (target_correspondence.first == i) {
            // pair source_id, target_id
            plane_correspondence.emplace_back(i, source_correspondence.first);
        }
    }

    return plane_correspondence;
}

void
estimateTranslationParameters(const std::vector<PlaneParam> &source_planes,
                              const std::vector<PlaneParam> &target_planes,
                              const std::vector<std::pair<int, int>> &plane_correspondences,
                              std::shared_ptr<Eigen::Vector3f> translation) {
    // compute the translation between the two planes

    Eigen::MatrixXf target_normals(3, plane_correspondences.size());
    Eigen::VectorXf target_d(plane_correspondences.size());
    Eigen::VectorXf source_d(plane_correspondences.size());
    for (int i = 0; i < plane_correspondences.size(); i++) {
        target_normals.col(i) = target_planes[plane_correspondences[i].second].first;
        target_d(i) = target_planes[plane_correspondences[i].second].second;
        source_d(i) = source_planes[plane_correspondences[i].first].second;
    }
    auto distance = source_d - target_d;
#if DEBUG
    std::cout << "Target d: " << std::endl << target_d << std::endl;
    std::cout << "Source d: " << std::endl << source_d << std::endl;
    std::cout << "Distance: " << std::endl << distance << std::endl;
#endif
//    *translation =  (target_normals * target_normals.transpose()).inverse() * target_normals * distance;
//    *translation = target_normals.transpose().colPivHouseholderQr().solve(distance);
    *translation = target_normals.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(distance);
}

std::pair<std::vector<std::pair<int, double>>, std::vector<std::pair<int, double>>>
computeNormalDistances(const std::vector<PlaneParam> &source_planes,
                       const std::vector<PlaneParam> &target_planes,
                       const Eigen::Matrix3f &rotation) {
#if DEBUG
    std::cout << "Rotation: " << std::endl << rotation << std::endl;
#endif
    std::vector<std::pair<int, double>> source_correspondence(source_planes.size(), std::make_pair(-1, -1));
    std::vector<std::pair<int, double>> target_correspondence(source_planes.size(), std::make_pair(-1, -1));

    for (int h = 0; h < source_planes.size(); h++) {
        auto source_plane = source_planes[h];
        Eigen::Vector3f normal = source_plane.first;
        auto rotated_normal = rotation * normal;

        for (int k = 0; k < target_planes.size(); k++) {
            auto target_plane = target_planes[k];
            auto dist = (rotated_normal - target_plane.first).norm();
            if (dist > 0.1) {
                // The vectors do not correspond
                continue;
            }
            auto prev_correspondence = target_correspondence[k].second;
            if (prev_correspondence < 0 || prev_correspondence > dist) {
                target_correspondence[k] = std::make_pair(h, dist);
            }
            prev_correspondence = source_correspondence[h].second;
            if (prev_correspondence < 0 || prev_correspondence > dist) {
                source_correspondence[h] = std::make_pair(k, dist);
            }
        }
    }
    return std::make_pair(source_correspondence, target_correspondence);
}

std::tuple<double, std::vector<std::pair<int, int>>, Eigen::Matrix3f>
processBasePair(const std::vector<PlaneParam> &source_planes,
                const std::vector<PlaneParam> &target_planes,
                const std::pair<int, int> &source_base_pair,
                const std::pair<int, int> &target_base_pair) {
    auto source_plane_i = source_planes[source_base_pair.first];
    auto source_plane_j = source_planes[source_base_pair.second];

    auto target_plane_h = target_planes[target_base_pair.first];
    auto target_plane_k = target_planes[target_base_pair.second];

    auto normal_i = source_plane_i.first;
    auto normal_j = source_plane_j.first;
    auto normal_h = target_plane_h.first;
    auto normal_k = target_plane_k.first;

#if ROTATION_ON_Z
    // For rotation only on Z
    Eigen::MatrixXf source_normals(normal_i.rows() - 1, normal_i.cols() + normal_j.cols());
    source_normals << normal_i.block<2, 1>(0, 0), normal_j.block<2, 1>(0, 0);

    Eigen::MatrixXf target_normals(normal_h.rows() - 1, normal_h.cols() + normal_k.cols());
    target_normals << normal_h.block<2, 1>(0, 0), normal_k.block<2, 1>(0, 0);
#else
    Eigen::MatrixXf source_normals(normal_i.rows(), normal_i.cols() + normal_j.cols());
    source_normals << normal_i, normal_j;

    Eigen::MatrixXf target_normals(normal_h.rows(), normal_h.cols() + normal_k.cols());
    target_normals << normal_h, normal_k;
#endif

    // Compute the rotation between the two planes in case i->h and j->k
    std::shared_ptr<Eigen::Matrix3f> rotation(new Eigen::Matrix3f);
    findRotationBetweenPlanes(
            source_normals,
            target_normals,
            rotation
    );


    // Rotate source cloud and create correspondences of planes
    auto correspondences = computeNormalDistances(source_planes, target_planes, *rotation);
    auto source_correspondence = correspondences.first;
    auto target_correspondence = correspondences.second;

#if DEBUG
    std::cout << "Compute initial correspondences for base (" << source_base_pair.first << ", "
              << source_base_pair.second
              << ") and (" << target_base_pair.first << ", " << target_base_pair.second << ")" << std::endl;
    std::cout << "There are : " << source_correspondence.size() << " correspondences" << std::endl;
#endif

    // Identify initial plane correspondences
    auto plane_correspondence = identifyPlaneCorrespondences(source_correspondence, target_correspondence);

    std::shared_ptr<Eigen::MatrixXf> transformation(new Eigen::MatrixXf);

    if (plane_correspondence.size() >= 2) {
        //Estimate initial transformation parameters
        estimateRigidTransformation(source_planes, target_planes, plane_correspondence, transformation, true);
    } else {
        return std::make_tuple(-1, std::vector<std::pair<int, int>>(), *rotation);
    }

    // Transform source plane
    std::vector<PlaneParam> transformed_target_planes;
    for (const auto &target_plane: target_planes) {
        Eigen::Vector4f normal;
        normal << target_plane.first, target_plane.second;
        auto transformed_normal = normal.transpose() * (*transformation);
        transformed_target_planes.emplace_back(
                Eigen::Vector3f(transformed_normal(0), transformed_normal(1), transformed_normal(2)),
                transformed_normal(3)
        );
    }

    auto new_correspondences = computeNormalDistances(source_planes, transformed_target_planes,
                                                      Eigen::Matrix3f::Identity());
    auto new_source_correspondence = new_correspondences.first;
    auto new_target_correspondence = new_correspondences.second;

    auto new_plane_correspondence = identifyPlaneCorrespondences(new_source_correspondence, new_target_correspondence);

#if DEBUG
    std::cout << "Compute new correspondences for base (" << source_base_pair.first << ", " << source_base_pair.second
              << ") and (" << target_base_pair.first << ", " << target_base_pair.second << ")" << std::endl;
    std::cout << "There are : " << new_plane_correspondence.size() << " correspondences" << std::endl;
#endif

    auto LCP = 0;
    auto threshold = 1.5;
    std::vector<std::pair<int, int>> final_correspondence;
    for (const auto &plane_corr: new_plane_correspondence) {
        // Check if the distance between the two planes is less than the threshold
#if DEBUG
        std::cout << "Correspondance " << "source " << plane_corr.first << " target " << plane_corr.second << " : "
                  << std::endl;
        std::cout << "Distance: " << std::abs(source_planes[plane_corr.first].second -
                                              transformed_target_planes[plane_corr.second].second) << std::endl;
#endif
        if (std::abs(
                source_planes[plane_corr.first].second - transformed_target_planes[plane_corr.second].second) <
            threshold) {
            LCP++;
            final_correspondence.emplace_back(plane_corr);
        }
    }

#if DEBUG
    std::cout << "Compute optimal correspondences for base (" << source_base_pair.first << ", "
              << source_base_pair.second
              << ") and (" << target_base_pair.first << ", " << target_base_pair.second << ")" << std::endl;
    std::cout << "There are : " << final_correspondence.size() << " correspondences" << std::endl;
#endif

    if (correspondencesValid(source_planes, transformed_target_planes, final_correspondence)) {
        return std::make_tuple(LCP, final_correspondence, *rotation);
    }
    else{
        return std::make_tuple(-1, std::vector<std::pair<int, int>>(), *rotation);
    }
}

Correspondences
findOptimalCorrespondences(const CompleteCloud &first_cloud,
                           const CompleteCloud &second_cloud) {
    double LCP = 0;
    std::vector<std::pair<int, int>> plane_correspondence;
    Eigen::Matrix3f best_rotation = Eigen::Matrix3f::Identity();
    std::pair<int, int> best_base_pair;
    double threshold = 1.0 * M_PI / 180.0;
    auto first_bases = std::get<2>(first_cloud);
    auto second_bases = std::get<2>(second_cloud);

//    first_bases.clear();
//    first_bases.emplace_back(0, 12, M_PI / 2);
//
//
//    second_bases.clear();
//    second_bases.emplace_back(11, 8, M_PI / 2);

    int nb_base_pairs = 0;

    for (auto base_1: first_bases) {
        for (auto base_2: second_bases) {
            auto angle_1 = std::get<2>(base_1);
            auto angle_2 = std::get<2>(base_2);
            if (std::abs(angle_1 - angle_2) < threshold) {
                nb_base_pairs++;

#if SHOW_ALL_BASES
                std::vector<pcl::Indices> indices;
                auto first_cloud_indices = std::get<1>(first_cloud).second;
                indices.emplace_back(first_cloud_indices[std::get<0>(base_1)]);
                indices.emplace_back(first_cloud_indices[std::get<1>(base_1)]);
                std::cout << "Base 1: " << std::get<0>(base_1) << " " << std::get<1>(base_1) << std::endl;
                visualizePlanesOnCloud(std::get<0>(first_cloud)->getInputCloud(), indices);

                auto second_cloud_indices = std::get<1>(second_cloud).second;
                indices.clear();
                indices.emplace_back(first_cloud_indices[std::get<0>(base_2)]);
                indices.emplace_back(first_cloud_indices[std::get<1>(base_2)]);
                std::cout << "Base 2: " << std::get<0>(base_2) << " " << std::get<1>(base_2) << std::endl;
                visualizePlanesOnCloud(std::get<0>(second_cloud)->getInputCloud(), indices);
#endif
                for (int runs = 0; runs < 1; runs++) {
                    int i = std::get<0>(base_1);
                    int j = std::get<1>(base_1);
                    int h, k;
                    if (runs == 0) {
                        h = std::get<0>(base_2);
                        k = std::get<1>(base_2);
                    } else {
                        h = std::get<1>(base_2);
                        k = std::get<0>(base_2);
                    }

                    // Visualize bases
                    Planes first_planes;
                    std::vector<PlaneParam> first_planes_params(2);
                    first_planes_params[0] = std::get<1>(first_cloud).first[i];
                    first_planes_params[1] = std::get<1>(first_cloud).first[j];
                    std::vector<pcl::Indices> first_cloud_indices(2);
                    first_cloud_indices[0] = std::get<1>(first_cloud).second[i];
                    first_cloud_indices[1] = std::get<1>(first_cloud).second[j];
                    first_planes = std::make_pair(first_planes_params, first_cloud_indices);

                    Planes second_planes;
                    std::vector<PlaneParam> second_planes_params(2);
                    second_planes_params[0] = std::get<1>(second_cloud).first[h];
                    second_planes_params[1] = std::get<1>(second_cloud).first[k];
                    std::vector<pcl::Indices> second_cloud_indices(2);
                    second_cloud_indices[0] = std::get<1>(second_cloud).second[h];
                    second_cloud_indices[1] = std::get<1>(second_cloud).second[k];
                    second_planes = std::make_pair(second_planes_params, second_cloud_indices);

#if SHOW_ROTATION
                    visualizeBases(std::get<0>(first_cloud)->getInputCloud(), first_planes,
                                   std::get<0>(second_cloud)->getInputCloud(), second_planes);
#endif
                    // Compute the base correspondences
                    auto base_correspondences = processBasePair(std::get<1>(first_cloud).first,
                                                                std::get<1>(second_cloud).first,
                                                                std::make_pair(i, j),
                                                                std::make_pair(h, k));

                    auto rotation = std::get<2>(base_correspondences);
#if SHOW_ROTATION
                    // Visualize the base rotation
                    auto transformed_source_cloud = rotatePointCloud(std::get<0>(first_cloud)->getInputCloud(),
                                                                     rotation);
                    for (auto &plane_params: first_planes.first) {
                        plane_params.first = rotation * plane_params.first;
                    }
                    visualizeBases(transformed_source_cloud, first_planes,
                                   std::get<0>(second_cloud)->getInputCloud(), second_planes);
#endif

                    // Check if the LCP is greater than the current LCP
                    auto LCP_i = std::get<0>(base_correspondences);
                    if (LCP_i == 0) {
                        continue;
                    }
                    auto plane_correspondence_i = std::get<1>(base_correspondences);
                    if (LCP_i > LCP) {
                        best_base_pair = std::make_pair(i, j);
                        best_rotation = rotation;
                        LCP = LCP_i;
                        plane_correspondence = plane_correspondence_i;
                    }


                    // Check if the LCP is already at the maximum
                    if (LCP == std::get<1>(first_cloud).first.size() || LCP == std::get<1>(second_cloud).first.size()) {
                        return plane_correspondence;
                    }
                } // end for runs
            } // end if -- base pair found
        } // end for base_2
    } // end for base_1
    std::cout << "nb_base_pairs: " << nb_base_pairs << std::endl;
    std::cout << "LCP: " << LCP << std::endl;
    std::cout << "best rotation = " << std::endl << best_rotation << std::endl;
    std::cout << "best base pair = " << std::endl << best_base_pair.first << "and" << best_base_pair.second
              << std::endl;
    return plane_correspondence;
}

void
estimateRigidTransformation(const std::vector<PlaneParam> &source_planes,
                            const std::vector<PlaneParam> &target_planes,
                            const Correspondences &correspondences,
                            std::shared_ptr<Eigen::MatrixXf> &transformation,
                            bool with_translation) {
    // Compute optimal transformation
#if ROTATION_ON_Z
    // For rotation only on Z
    Eigen::MatrixXf source_normals(2, correspondences.size());
    Eigen::MatrixXf target_normals(2, correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
        source_normals.col(i) = source_planes[correspondences[i].first].first.block<2, 1>(0, 0);
        target_normals.col(i) = target_planes[correspondences[i].second].first.block<2, 1>(0, 0);
    }
#else
    Eigen::MatrixXf source_normals(3, correspondences.size());
    Eigen::MatrixXf target_normals(3, correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
        source_normals.col(i) = source_planes[correspondences[i].first].first;
        target_normals.col(i) = target_planes[correspondences[i].second].first;
    }
#endif

    std::shared_ptr<Eigen::Matrix3f> rotation(new Eigen::Matrix3f());
    findRotationBetweenPlanes(source_normals, target_normals, rotation);

    // Compute the transformation matrix
    transformation->resize(4, 4); //4x4
    transformation->block<3, 3>(0, 0) = *rotation;
    if (with_translation) {
        std::shared_ptr<Eigen::Vector3f> translation(new Eigen::Vector3f());
        estimateTranslationParameters(source_planes, target_planes, correspondences, translation);
        transformation->block(0, 3, 3, 1) = *translation;
    } else {
        transformation->block(0, 3, 3, 1) = Eigen::Vector3f(0, 0, 0);
    }
    transformation->block(3, 0, 1, 4) = Eigen::Vector4f(0, 0, 0, 1).transpose();
}