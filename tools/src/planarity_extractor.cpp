//
// Created by amine on 29.05.23.
//
#include "planarity_extractor.h"

double computePlanarityScore(Octree::Ptr &octree, pcl::Indices &indices, PointT &centroid) {
    // Create covariance matrix
    Eigen::Matrix3f matrice_covariance = Eigen::Matrix3f::Zero();
    for (auto indice: indices) {
        auto point = octree->getInputCloud()->at(indice);
        auto vec = point.getVector3fMap() - centroid.getVector3fMap();
        auto mat = vec * vec.transpose();
        matrice_covariance += mat;
    }

    // Compute eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(matrice_covariance);
    if (eigensolver.info() != Eigen::Success) abort();
    auto eigenvalues = eigensolver.eigenvalues();

    // Compute planarity score
    return eigenvalues(2) / (eigenvalues(0) + eigenvalues(1) + eigenvalues(2));
}


void removePointsFromCloud(PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &outliers) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::cout << cloud->size() << std::endl;

    extract.setInputCloud(cloud);
    extract.setIndices(outliers);
    extract.setNegative(true);
    std::cout << "using filter " << std::endl;
    extract.filter(*cloud);
    std::cout << cloud->size() << std::endl;
}


int removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg) {
    int deleted_voxels = 0;

    std::vector<LeafContainerT *> leaf_container_vector_arg;
    octree->serializeLeafs(leaf_container_vector_arg);

    for (auto leaf: leaf_container_vector_arg) {
        if (leaf->getSize() < min_points_per_voxel_arg) {
            PointT centroid;
            leaf->getCentroid(centroid);

            octree->deleteVoxelAtPoint(centroid);
            deleted_voxels++;
        }
    }
    return deleted_voxels;
}

int removeNonPlanarVoxels(Octree::Ptr &octree, double max_score) {
    int deleted_voxels = 0;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

#if DEBUG
    std::vector<double> planarity_scores;
#endif

    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);
    for (auto leaf: *leaf_container_vector_arg) {
        PointT centroid;
        leaf->getCentroid(centroid);
        pcl::Indices indices;
        leaf->getPointIndices(indices);
        auto score = computePlanarityScore(octree, indices, centroid);
#if DEBUG
        planarity_scores.push_back(score);
#endif
        if (score > max_score) {
            octree->deleteVoxelAtPoint(centroid);
            deleted_voxels++;
        }
    }
#if DEBUG
    std::cout << "planarity scores " << std::endl;
    for (auto score: planarity_scores) {
        std::cout << score << std::endl;
    }
#endif
    return deleted_voxels;
}

std::pair<Eigen::VectorXf, pcl::Indices>
refinePlane(PointCloudPtr &plane, double distance_threshold, double probability, int max_iter) {
    // Create a plane model
    pcl::SampleConsensusModelPlane<PointT>::Ptr
            model_p(new pcl::SampleConsensusModelPlane<PointT>(plane));

    // Create the RANSAC object and set its parameters
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(distance_threshold);
    ransac.setProbability(probability);
    ransac.setMaxIterations(max_iter);

    // Compute the model
    ransac.computeModel(1);

    // Get inliers
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);
//    ransac.refineModel(3.0, 1000);

    // Only keep the inliers -- Not needed
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(plane);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    // Get coefficients
    Eigen::VectorXf coefficients; //a,b,c,d where ax+by+cz+d=0
    ransac.getModelCoefficients(coefficients);
#if DEBUG
    std::cout << "coefficients " << coefficients << std::endl;
#endif
    return std::make_pair(coefficients, inliers->indices);
}


std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>
extractPlane(Octree::Ptr &octree) {
    // Get the list of leafs
    std::vector<LeafContainerT * > leaf_container_vector_arg;
    octree->serializeLeafs(leaf_container_vector_arg);

    // Storage for plane parameters and their indices in the point cloud
    std::vector<PlaneParam> params;
    std::vector<pcl::Indices> indice_vec;

#if MINIMAL_OUTPUT
    std::cout << "leafs " << leaf_container_vector_arg.size() << std::endl;
#endif
    for (auto leaf: leaf_container_vector_arg) {
        // Extract the points from the leaf (voxel)
        pcl::Indices indices;
        leaf->getPointIndices(indices);
        PointCloudPtr plane(new PointCloud);
        auto cloud = octree->getInputCloud();
        pcl::copyPointCloud(*cloud, indices, *plane);

        if (plane->size() < 3) {
            // Not enough points to create a plane
            continue;
        }

        // Define the plane
        auto refined_plane = refinePlane(plane, 0.01, 0.99, 5000);
        auto coefficients = refined_plane.first;

        // indices should be updated after refinement -- The plane should only be defined by the points in the plane
        auto refined_indices = refined_plane.second;
        for (auto &index: refined_indices) {
            index = indices[index];
        }

        // Check if the plane is valid and extract its parameters
        if (coefficients.size() < 4) continue;
        Eigen::Vector3f normal(coefficients(0), coefficients(1), coefficients(2));
        double distance = coefficients(3);

        if (distance < 0) {
            // Choose the normal in order for the distance to be positive
            normal = -normal;
            distance = -distance;
        }

        // check if the plane is already in the list
        bool new_plane = true;
        int i = 0;
        while (i < params.size() && new_plane) {
            auto param = params[i];
            if (param.first.isApprox(normal, 0.01) && std::abs(param.second - distance) < 0.01) {
                // Plane already exists
                new_plane = false;
            }
            i++;
        }

        if (new_plane) {
            params.emplace_back(normal, distance);
            indice_vec.push_back(refined_indices);
        }
    }
    return std::make_pair(params, indice_vec);
}