//
// Created by amine on 30.05.23.
//

#include "base_processor.h"

#include <utility>

bool
correspondencesValid(const std::vector<PlaneParam> &source_planes, const std::vector<PlaneParam> &target_planes,
                     const Correspondences &correspondences) {
    return computeNumberOfNonColinearPlanes(source_planes, target_planes, correspondences) >= 3;
}

bool
correspondencesValid(int number_of_non_collinear_planes) {
    return number_of_non_collinear_planes >= 3;
}

int
computeNumberOfNonColinearPlanes(const std::vector<PlaneParam> &source_planes,
                                 const std::vector<PlaneParam> &target_planes,
                                 const Correspondences &correspondences) {
    /*
     * Correspondences are valid if there are at least three non-collinear planes
     * An angle of PI counts as collinear
     */

    //
    if (correspondences.size() < 3) {
        return false;
    }

    // Angle threshold to consider two planes collinear
    double angle_threshold = 5.0 * M_PI / 180.0;

    std::vector<int> source_possible_three_base_planes;
    std::vector<int> target_possible_three_base_planes;

    source_possible_three_base_planes.emplace_back(correspondences[0].first);
    target_possible_three_base_planes.emplace_back(correspondences[0].second);

    for (int i = 1; i < correspondences.size(); i++) {
        auto correspondence = correspondences[i];

        // Check if the source plane is collinear with the previous ones
        auto first_source_normal = std::get<0>(source_planes[correspondence.first]);
        int possible_part_of_base = 0;

        for (auto plane_idx: source_possible_three_base_planes) {
            auto second_source_normal = std::get<0>(source_planes[plane_idx]);
            auto angle = angleBetweenVectors(first_source_normal, second_source_normal);
            if (angle > angle_threshold && angle < (M_PI - angle_threshold)) {
                possible_part_of_base++;
            }
        }
        int size_base = source_possible_three_base_planes.size();
        if (possible_part_of_base >= std::min(2, size_base)) {
            // The plane is not collinear with the previous ones and can be added
            source_possible_three_base_planes.emplace_back(correspondence.first);
        }

        // Check if the target plane is collinear with the previous ones
        auto first_target_normal = std::get<0>(target_planes[correspondence.second]);
        possible_part_of_base = 0;
        for (auto plane_idx: target_possible_three_base_planes) {
            auto second_target_normal = std::get<0>(target_planes[plane_idx]);
            auto angle = angleBetweenVectors(first_target_normal, second_target_normal);
            if (angle > angle_threshold && angle < (M_PI - angle_threshold)) {
                possible_part_of_base++;
            }
        }
        size_base = target_possible_three_base_planes.size();
        if (possible_part_of_base >= std::min(2, size_base)) {
            // The plane is not collinear with the previous ones and can be added
            target_possible_three_base_planes.emplace_back(correspondence.second);
        }
    }
    int number_of_non_collinear_planes = std::min(source_possible_three_base_planes.size(),
                                                  target_possible_three_base_planes.size());
    return number_of_non_collinear_planes;
}

void
generateBasesFromPlaneParams(const std::vector<PlaneParam> &plane_params, Bases *bases, double min_angle,
                             double max_angle) {
    auto nb_planes = plane_params.size();
    for (int i = 0; i < nb_planes; i++) {
        for (int j = i + 1; j < nb_planes; j++) {
            auto plane1 = plane_params[i];
            auto plane2 = plane_params[j];
            auto angle = angleBetweenVectors(std::get<0>(plane1), std::get<0>(plane2));
            if (angle > min_angle && angle < max_angle) {
                (*bases).emplace_back(i, j, angle);
            }
        }
    }
}

Correspondences
identifyPlaneCorrespondences(
        std::pair<std::vector<std::pair<int, double>>, std::vector<std::pair<int, double>>> &computed_correspondences) {

    auto source_correspondences = computed_correspondences.first;
    auto target_correspondences = computed_correspondences.second;

#if HEAVY_DEBUG
    std::cout << "Source correspondences: " << std::endl;
    for (auto correspondence: source_correspondences) {
        std::cout << correspondence.first << " " << correspondence.second << std::endl;
    }
    std::cout << "Target correspondences: " << std::endl;
    for (auto correspondence: target_correspondences) {
        std::cout << correspondence.first << " " << correspondence.second << std::endl;
    }
#endif

    std::vector<std::pair<int, int>> plane_correspondence;
    std::vector<int> look_for_second;

    // Check that if a plane (i) is matched to another plane (h), the other plane (h) is also matched to the first one (i)
    for (int i = 0; i < source_correspondences.size(); i++) {
        auto source_correspondence = source_correspondences[i];
        if (source_correspondence.first == -1) {
            continue;
        }
        // source_correspondences[i].first = h
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
        target_normals.col(i) = std::get<0>(target_planes[plane_correspondences[i].second]);

        target_d(i) = std::get<1>(target_planes[plane_correspondences[i].second]);
        source_d(i) = std::get<1>(source_planes[plane_correspondences[i].first]);
    }
    auto distance = source_d - target_d;
#if HEAVY_DEBUG
    std::cout << "Target d: " << std::endl << target_d << std::endl;
    std::cout << "Source d: " << std::endl << source_d << std::endl;
    std::cout << "Distance: " << std::endl << distance << std::endl;
#endif
//    *translation =  (target_normals * target_normals.transpose()).inverse() * target_normals * distance;
//    *translation = target_normals.transpose().colPivHouseholderQr().solve(distance);
    // Compute the translation using the Least Squares solver
    *translation = target_normals.transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(distance);
}

std::pair<std::vector<std::pair<int, double>>, std::vector<std::pair<int, double>>>
computeNormalDistances(const std::vector<PlaneParam> &source_planes,
                       const std::vector<PlaneParam> &target_planes,
                       const Eigen::Matrix3f &rotation,
                       const std::pair<int, int> &source_base_pair,
                       const std::pair<int, int> &target_base_pair) {

    // For each plane in the source, find the closest plane in the target
    std::vector<std::pair<int, double>> source_correspondence(source_planes.size(), std::make_pair(-1, -1));
    // For each plane in the target, find the closest plane in the source
    std::vector<std::pair<int, double>> target_correspondence(target_planes.size(), std::make_pair(-1, -1));

    for (int h = 0; h < source_planes.size(); h++) {
        auto source_plane = source_planes[h];
        Eigen::Vector3f normal = std::get<0>(source_plane);

        // Rotate the source plane to be collinear to its target (if it exists)
        auto rotated_normal = rotation * normal;

        for (int k = 0; k < target_planes.size(); k++) {
            auto target_plane = target_planes[k];
            auto dist = (rotated_normal - std::get<0>(target_plane)).norm();

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


    if (source_base_pair.first != -1 && target_base_pair.first != -1) {
        // encourage correspondences of bases
        if (source_correspondence[source_base_pair.first].first == target_base_pair.first) {
            source_correspondence[source_base_pair.first].second = 0;
            target_correspondence[target_base_pair.first] = std::make_pair(source_base_pair.first, 0);
        } else if (target_correspondence[target_base_pair.first].first == source_base_pair.first) {
            target_correspondence[target_base_pair.first].second = 0;
            source_correspondence[source_base_pair.first] = std::make_pair(target_base_pair.first, 0);
        }
    }
    if (source_base_pair.second != -1 && target_base_pair.second != -1) {
        // encourage correspondences of bases
        if (source_correspondence[source_base_pair.second].first == target_base_pair.second) {
            source_correspondence[source_base_pair.second].second = 0;
            target_correspondence[target_base_pair.second] = std::make_pair(source_base_pair.second, 0);
        } else if (target_correspondence[target_base_pair.second].first == source_base_pair.second) {
            target_correspondence[target_base_pair.second].second = 0;
            source_correspondence[source_base_pair.second] = std::make_pair(target_base_pair.second, 0);
        }
    }


    return std::make_pair(source_correspondence, target_correspondence);
}

std::tuple<double, std::vector<std::pair<int, int>>, Eigen::Matrix4f>
processBasePair(const std::vector<PlaneParam> &source_planes,
                const std::vector<PlaneParam> &target_planes,
                const std::pair<int, int> &source_base_pair,
                const std::pair<int, int> &target_base_pair) {
    // Select the base pair of planes
    auto source_plane_i = source_planes[source_base_pair.first];
    auto source_plane_j = source_planes[source_base_pair.second];

    auto target_plane_h = target_planes[target_base_pair.first];
    auto target_plane_k = target_planes[target_base_pair.second];

    auto normal_i = std::get<0>(source_plane_i);
    auto normal_j = std::get<0>(source_plane_j);
    auto normal_h = std::get<0>(target_plane_h);
    auto normal_k = std::get<0>(target_plane_k);

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
    auto correspondences = computeNormalDistances(source_planes,
                                                  target_planes,
                                                  *rotation,
                                                  source_base_pair,
                                                  target_base_pair);

    // Identify initial plane correspondences
    auto plane_correspondence = identifyPlaneCorrespondences(correspondences);

#if DEBUG
    std::cout << "Compute initial correspondences for base (" << source_base_pair.first << ", "
              << source_base_pair.second
              << ") and (" << target_base_pair.first << ", " << target_base_pair.second << ")" << std::endl;
    std::cout << "There are : " << plane_correspondence.size() << " correspondences" << std::endl;
    for (auto &corr: plane_correspondence) {
        std::cout << "source : " << corr.first << " -- target :" << corr.second << std::endl;
    }
#endif

    //Estimate initial transformation parameters
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    if (correspondencesValid(source_planes, target_planes, plane_correspondence)) {
        estimateRigidTransformation(source_planes, target_planes, plane_correspondence, transformation, true);
    } else {
        return std::make_tuple(-1, std::vector<std::pair<int, int>>(), transformation);
    }

    // Transform target plane
    std::vector<PlaneParam> transformed_target_planes;
    for (const auto &target_plane: target_planes) {
        Eigen::Vector4f normal;
        normal << std::get<0>(target_plane), std::get<1>(target_plane);
        auto transformed_normal = normal.transpose() * transformation;
        transformed_target_planes.emplace_back(
                Eigen::Vector3f(transformed_normal(0), transformed_normal(1), transformed_normal(2)),
                transformed_normal(3),
                std::get<2>(target_plane)
        );
    }

    correspondences = computeNormalDistances(source_planes,
                                             target_planes,
                                             transformation.block<3, 3>(0, 0),
                                             source_base_pair,
                                             target_base_pair);
    plane_correspondence = identifyPlaneCorrespondences(correspondences);

#if DEBUG
    std::cout << "Compute new correspondences for base (" << source_base_pair.first << ", "
              << source_base_pair.second
              << ") and (" << target_base_pair.first << ", " << target_base_pair.second << ")" << std::endl;
    std::cout << "There are : " << plane_correspondence.size() << " correspondences" << std::endl;
    for (auto &corr: plane_correspondence) {
        std::cout << "source : " << corr.first << " -- target : " << corr.second << std::endl;
    }
#endif

    auto LCP = 0;
    auto threshold = 0.5;
    std::vector<std::pair<int, int>> final_correspondence;
    for (const auto &plane_corr: plane_correspondence) {
        // Check if the distance between the two planes is less than the threshold
        double source_distance = std::get<1>(source_planes[plane_corr.first]);
        double target_distance = std::get<1>(transformed_target_planes[plane_corr.second]);
#if HEAVY_DEBUG
        std::cout << "Correspondance " << "source " << plane_corr.first << " target " << plane_corr.second << " : "
                  << std::endl;
        std::cout << "Distance: " << std::abs(source_distance - target_distance) << std::endl;
#endif

        if (std::abs(source_distance - target_distance) < threshold) {
            LCP += std::get<2>(source_planes[plane_corr.first]) * std::get<2>(target_planes[plane_corr.second]);
//            LCP += std::min(std::get<2>(source_planes[plane_corr.first]), std::get<2>(target_planes[plane_corr.second]));
            final_correspondence.emplace_back(plane_corr);
        }
    }

#if DEBUG
    std::cout << "Compute optimal correspondences for base (" << source_base_pair.first << ", "
              << source_base_pair.second
              << ") and (" << target_base_pair.first << ", " << target_base_pair.second << ")" << std::endl;
    std::cout << "There are : " << final_correspondence.size() << " correspondences" << std::endl;
    for (auto &corr: final_correspondence) {
        std::cout << "source : " << corr.first << " -- target : " << corr.second << std::endl;
    }
#endif

#if DEBUG
    std::cout << "LCP_i: " << LCP << std::endl;
#endif

    if (correspondencesValid(source_planes, target_planes, final_correspondence)) {
#if DEBUG
        std::cout << "Correspondences are valid" << std::endl;
#endif
        return std::make_tuple(LCP, final_correspondence, transformation);
    } else {
#if DEBUG
        std::cout << "Correspondences are not valid" << std::endl;
#endif
        return std::make_tuple(-1, std::vector<std::pair<int, int>>(), transformation);
    }
}

Correspondences
findOptimalCorrespondences(const CompleteCloud &first_cloud,
                           const CompleteCloud &second_cloud) {
    double LCP = 0;
    std::vector<std::pair<int, int>> plane_correspondence;
    Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();
    std::pair<int, int> best_base_pair;
    std::pair<int, int> best_corresponding_base_pair;

    // Threshold to consider bases are corresponding
    double threshold = 1.0 * M_PI / 180.0;

    auto first_bases = std::get<2>(first_cloud);
    auto second_bases = std::get<2>(second_cloud);

//    first_bases.clear();
//    first_bases.emplace_back(4, 5, M_PI / 2);
//
//
//    second_bases.clear();
//    second_bases.emplace_back(0, 2, M_PI / 2);

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
                for (int runs = 0; runs < 2; runs++) {
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

#if SHOW_ROTATION
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

                    visualizeBases(std::get<0>(first_cloud)->getInputCloud(), first_planes,
                                   std::get<0>(second_cloud)->getInputCloud(), second_planes);
#endif
                    // Compute the base correspondences
                    auto base_correspondences = processBasePair(std::get<1>(first_cloud).first,
                                                                std::get<1>(second_cloud).first,
                                                                std::make_pair(i, j),
                                                                std::make_pair(h, k));

                    Eigen::Matrix4f transformation = std::get<2>(base_correspondences);
#if SHOW_ROTATION
                    auto rotation = transformation.block<3, 3>(0, 0);

                    // Visualize the base rotation
                    auto rotated_source_cloud = rotatePointCloud(std::get<0>(first_cloud)->getInputCloud(),
                                                                     rotation);
                    for (auto &plane_params: first_planes.first) {
                        std::get<0>(plane_params) = rotation * std::get<0>(plane_params);
                    }

                    visualizeBases(rotated_source_cloud, first_planes,
                                   std::get<0>(second_cloud)->getInputCloud(), second_planes);

                    // Visualize the base transformation
                    auto transformed_source_cloud = transformSourcePointCloud(std::get<0>(first_cloud)->getInputCloud(),
                                                                              transformation);

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
                        best_corresponding_base_pair = std::make_pair(h, k);
                        best_transformation = transformation;
                        LCP = LCP_i;
                        plane_correspondence = plane_correspondence_i;
                    }

                } // end for runs
            } // end if -- base pair found
        } // end for base_2
    } // end for base_1
#if DEBUG
    std::cout << "nb_base_pairs: " << nb_base_pairs << std::endl;
    std::cout << "LCP: " << LCP << std::endl;
    std::cout << "best transformation = " << std::endl << best_transformation << std::endl;
    std::cout << "best base pair = " << std::endl << best_base_pair.first << " and " << best_base_pair.second
              << std::endl;
    std::cout << "best corresponding base pair = " << std::endl << best_corresponding_base_pair.first << " and "
              << best_corresponding_base_pair.second << std::endl;
#endif
    return plane_correspondence;
}

void
estimateRigidTransformation(const std::vector<PlaneParam> &source_planes,
                            const std::vector<PlaneParam> &target_planes,
                            const Correspondences &correspondences,
                            Eigen::Matrix4f &transformation,
                            bool with_translation) {
    // Compute optimal transformation
#if ROTATION_ON_Z
    // For rotation only on Z
    Eigen::MatrixXf source_normals(2, correspondences.size());
    Eigen::MatrixXf target_normals(2, correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
        auto source_normal = std::get<0>(source_planes[correspondences[i].first]);
        source_normals.col(i) = source_normal.block<2, 1>(0, 0);

        auto target_normal = std::get<0>(target_planes[correspondences[i].second]);
        target_normals.col(i) = target_normal.block<2, 1>(0, 0);
    }
#else
    Eigen::MatrixXf source_normals(3, correspondences.size());
    Eigen::MatrixXf target_normals(3, correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
        source_normals.col(i) = std::get<0>(source_planes[correspondences[i].first]);
        target_normals.col(i) = std::get<0>(target_planes[correspondences[i].second]);
    }
#endif

    std::shared_ptr<Eigen::Matrix3f> rotation(new Eigen::Matrix3f());
    findRotationBetweenPlanes(source_normals, target_normals, rotation);

    // Compute the transformation matrix
    transformation.resize(4, 4); //4x4
    transformation.block<3, 3>(0, 0) = *rotation;
    if (with_translation) {
        std::shared_ptr<Eigen::Vector3f> translation(new Eigen::Vector3f());
        estimateTranslationParameters(source_planes, target_planes, correspondences, translation);
        transformation.block(0, 3, 3, 1) = *translation;
    } else {
        transformation.block(0, 3, 3, 1) = Eigen::Vector3f(0, 0, 0);
    }
    transformation.block(3, 0, 1, 4) = Eigen::Vector4f(0, 0, 0, 1).transpose();
}