//
// Created by amine on 29.05.23.
//
#include "utils.h"

PointCloudPtr loadPcd(const std::string &pcd_file_path) {
    /* Load a pcd file and vizualise it */
    // Create a point cloud object
    PointCloudPtr cloud(new PointCloud);
    // Load the pcd file
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud);
#if DEBUG
    std::cout << "Point Cloud Loaded" << cloud->size() << std::endl;
#endif
    return cloud;
}

Octree toOctree(const PointCloud::ConstPtr &cloud, double resolution) {
    /* Convert a point cloud to an octree */
    // Create an octree object
    Octree octree(resolution);
    // Add the point cloud to the octree
    octree.setInputCloud(cloud);
    // Add points from the cloud to the octree
    octree.addPointsFromInputCloud();
#if DEBUG
    std::cout << "Number of voxels in the octree: " << octree.getLeafCount() << std::endl;
#endif
    return octree;
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

double angleBetweenVectors(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {
    return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

void findRotationBetweenPlanes(const Eigen::MatrixXf &source_normals,
                               const Eigen::MatrixXf &target_normals,
                               std::shared_ptr<Eigen::Matrix3f> &R) {

    /* Find rotation supposing that it is around the z axis */
    // calculate SVD of normals
    Eigen::MatrixXf normals_matrix = source_normals * target_normals.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(normals_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

    int dim = source_normals.rows();

    if (dim == 2) {
        // For rotation only around z axis
        Eigen::Matrix2f rotation = svd.matrixV() * svd.matrixU().transpose();
        // Check if the matrix is a reflection
        if (rotation.determinant() < 0) {
            rotation = svd.matrixV() * (Eigen::Matrix2f() << 1, 0, 0, -1).finished() * svd.matrixU().transpose();
        }
        *R = Eigen::Matrix3f::Identity();
        R->block<2, 2>(0, 0) = rotation;
    } else {
        // Rotation in 3D
        *R = svd.matrixV() * svd.matrixU().transpose();
        // Check if the matrix is a reflection
        if (R->determinant() < 0) {
            *R = svd.matrixV() * (Eigen::Matrix3f() << 1, 0, 0, 0, 1, 0, 0, 0, -1).finished() *
                 svd.matrixU().transpose();
        }
    }
}

PointCloudPtr
transformTargetPointCloud(const PointCloud::ConstPtr &cloud, const Eigen::MatrixXf &transformation) {
    PointCloudPtr transformed_cloud(new PointCloud);
    transformed_cloud = cloud->makeShared();
    transformed_cloud->points.clear();

    for (auto point: cloud->points) {
        Eigen::Vector4f point_4d;
        point_4d << point.x, point.y, point.z, 1;
        auto transformed_point = transformation * point_4d;
        transformed_cloud->points.emplace_back(
                pcl::PointXYZ(transformed_point(0), transformed_point(1), transformed_point(2))
        );
    }
    return transformed_cloud;
}

PointCloudPtr
rotatePointCloud(const PointCloud::ConstPtr &cloud, const Eigen::Matrix3f &rotation) {
    PointCloudPtr rotated_cloud(new PointCloud);
    rotated_cloud = cloud->makeShared();
    rotated_cloud->points.clear();

    for (auto point: cloud->points) {
        Eigen::Vector3f point_3d;
        point_3d << point.x, point.y, point.z;
        auto transformed_point = rotation * point_3d;
        rotated_cloud->points.emplace_back(
                pcl::PointXYZ(transformed_point(0), transformed_point(1), transformed_point(2))
        );
    }
    return rotated_cloud;
}
