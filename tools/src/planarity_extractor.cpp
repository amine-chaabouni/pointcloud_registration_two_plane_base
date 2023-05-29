//
// Created by amine on 29.05.23.
//
#include "planarity_extractor.h"

double computePlanarityScore(Octree::Ptr &octree, LeafContainerT &leaf, PointT &centroid) {
    double planarity_score = 1.0;
    pcl::Indices indices;
    leaf.getPointIndices(indices);
    Eigen::Matrix3f matrice_covariance = Eigen::Matrix3f::Zero();
    for (auto indice: indices) {
        auto point = octree->getInputCloud()->at(indice);
        auto vec = point.getVector3fMap() - centroid.getVector3fMap();
        auto mat = vec * vec.transpose();
        matrice_covariance += mat;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(matrice_covariance);
    if (eigensolver.info() != Eigen::Success) abort();
    auto eigenvalues = eigensolver.eigenvalues();
    planarity_score = eigenvalues(2) / (eigenvalues(0) + eigenvalues(1) + eigenvalues(2));
    std::cout << "planarity score " << planarity_score << std::endl;
    return planarity_score;
}


void removePointsFromCloud(PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &inliers) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::cout << cloud->size() << std::endl;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    std::cout << "using filter " << std::endl;
    extract.filter(*cloud);
    std::cout << cloud->size() << std::endl;
}


int removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg) {
    int deleted_voxels = 0;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);

    for (auto leaf: *leaf_container_vector_arg) {
        if (leaf->getSize() < min_points_per_voxel_arg) {
            PointT centroid;
            leaf->getCentroid(centroid);

            pcl::Indices indices;
            leaf->getPointIndices(indices);
            inliers->indices.insert(inliers->indices.end(), indices.begin(), indices.end());

            octree->deleteVoxelAtPoint(centroid);
            deleted_voxels++;
        }
    }

//    removePointsFromCloud(cloud, inliers);
//    octree = std::make_shared<Octree>(toOctree(cloud, octree->getResolution()));
    return deleted_voxels;
}

int removeNonPlanarVoxels(Octree::Ptr &octree, double max_score) {
    int deleted_voxels = 0;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::vector<double> planarity_scores;

    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);
    for (auto leaf: *leaf_container_vector_arg) {
        PointT centroid;
        leaf->getCentroid(centroid);
        auto score = computePlanarityScore(octree, *leaf, centroid);
        planarity_scores.push_back(score);
        if (score > max_score) {
            pcl::Indices indices;
            leaf->getPointIndices(indices);
            inliers->indices.insert(inliers->indices.end(), indices.begin(), indices.end());
            octree->deleteVoxelAtPoint(centroid);
            deleted_voxels++;
#if DEBUG
            std::cout << "Deleted voxel with centroid " << centroid << " and score " << score << std::endl;
            std::cout << "leaf has " << indices.size() << " points" << std::endl;
#endif
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

std::pair<Eigen::VectorXf, pcl::Indices> refinePlane(PointCloudPtr &plane, double distance_threshold, double probability, int max_iter) {
    pcl::SampleConsensusModelPlane<PointT>::Ptr
            model_p(new pcl::SampleConsensusModelPlane<PointT>(plane));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(distance_threshold);
    ransac.setProbability(probability);
    ransac.setMaxIterations(max_iter);
    ransac.computeModel(1);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);
//    ransac.refineModel(3.0, 1000);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(plane);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);
    Eigen::VectorXf coefficients; //a,b,c,d where ax+by+cz+d=0
    ransac.getModelCoefficients(coefficients);
#if DEBUG
    std::cout << "coefficients " << coefficients << std::endl;
#endif
    return std::make_pair(coefficients, inliers->indices);
}


std::pair<std::vector<PlaneParam>, std::vector<pcl::Indices>>  extractPlane(Octree::Ptr &octree) {
    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);
    std::vector<PlaneParam> params;
    std::vector<pcl::Indices> indice_vec;
//    PointCloudPtr new_cloud(new PointCloud);
    std::cout << "leafs " << leaf_container_vector_arg->size() << std::endl;

    for (auto leaf: *leaf_container_vector_arg) {
        pcl::Indices indices;
        leaf->getPointIndices(indices);
        PointCloudPtr plane(new PointCloud);
        auto cloud = octree->getInputCloud();
        pcl::copyPointCloud(*cloud, indices, *plane);
        if(plane->size() < 3){
            std::cout << indices.size() << std::endl;
            continue;
        }

        auto refined_plane = refinePlane(plane);
        auto coefficients = refined_plane.first;
        // indices should be updated after refinement
//         indices = refined_plane.second;
        if(coefficients.size() == 0) continue;
        Eigen::Vector3f normal(coefficients(0), coefficients(1), coefficients(2));
        double distance = coefficients(3);

        if (distance < 0) {
            normal = -normal;
            distance = -distance;
        }

        params.emplace_back(normal, distance);
        indice_vec.push_back(indices);

//        new_cloud->insert(new_cloud->end(), plane->begin(), plane->end());
    }
    return std::make_pair(params, indice_vec);
}