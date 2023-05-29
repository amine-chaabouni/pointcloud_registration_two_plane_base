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
    planarity_score = eigenvalues(1) / (eigenvalues.sum());
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


int removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg, PointCloud::Ptr &cloud) {
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

    removePointsFromCloud(cloud, inliers);
    octree = std::make_shared<Octree>(toOctree(cloud, octree->getResolution()));
    return deleted_voxels;
}

int removeNonPlanarVoxels(Octree::Ptr &octree, double max_score, PointCloud::Ptr &cloud) {
    int deleted_voxels = 0;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);
    for (auto leaf: *leaf_container_vector_arg) {
        PointT centroid;
        leaf->getCentroid(centroid);
        auto score = computePlanarityScore(octree, *leaf, centroid);
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
    removePointsFromCloud(cloud, inliers);
    octree = std::make_shared<Octree>(toOctree(cloud, octree->getResolution()));
    return deleted_voxels;
}


void checkOctree(const Octree::Ptr &octree) {
    /* Check the voxels in an octree */
    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);
    auto cloud = octree->getInputCloud();
    for (auto leaf: *leaf_container_vector_arg) {
        PointT centroid;
        leaf->getCentroid(centroid);
        std::cout << "centroid = " << centroid << std::endl;
        pcl::Indices indices;
        leaf->getPointIndices(indices);
        std::cout << "indices size = " << indices.size() << std::endl;
        auto last_idx = leaf->getPointIndex();
        std::cout << "last index = " << last_idx << " and corresponding point is " << cloud->at(last_idx) << std::endl;
    }
}