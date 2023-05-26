#include <iostream>
#include <memory>
#include <string>
// # include "point_cloud_loader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "octree_viewer.h"
#include "octree_voxel_container.h"
#include "octree_search_centroid.h"

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>


#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#define DEBUG 0
#define VISUALIZE 0

using PointT = pcl::PointXYZ;
using LeafContainerT = pcl::octree::OctreeVoxelContainer<PointT>;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using Octree = pcl::octree::OctreeVoxelBasedRegistration<PointT, LeafContainerT>;

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

void visualizePcd(const PointCloud::ConstPtr &cloud) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("pcd viewer");
    // Add the point cloud to the viewer
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "point cloud");
    // Set the background of the viewer to black
    viewer.setBackgroundColor(0, 0, 0);
    // Set the size of the point
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
    // Display the point cloud
#if VISUALIZE
    std::cout << "Spinning viewer in load_pcd" << std::endl;
    viewer.spin();
    while (!viewer.wasStopped()) {
        std::cout << "Viewer spun" << std::endl;
    }
    std::cout << "Viewer Stopped" << std::endl;
#endif
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

void visualizeOctree(const PointCloudPtr &cloud, double resolution) {
    OctreeViewer viewer{cloud, resolution};
}

int removeVoxelsWithLessThanXPoints(Octree::Ptr &octree, int min_points_per_voxel_arg) {
    int deleted_voxels = 0;
    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);

    for (auto leaf: *leaf_container_vector_arg) {
        if (leaf->getSize() < min_points_per_voxel_arg) {
            PointT centroid;
            leaf->getCentroid(centroid);
            octree->deleteVoxelAtPoint(centroid);
            deleted_voxels++;
        }
    }

    return deleted_voxels;
}

double computePlanarityScore(Octree::Ptr &octree, LeafContainerT &leaf, PointT &centroid){
    double planarity_score = 1.0;
    pcl::Indices indices;
    leaf.getPointIndices(indices);
    Eigen::Matrix3f matrice_covariance = Eigen::Matrix3f::Zero();
    for(auto indice: indices){
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

int removeNonPlanarVoxels(Octree::Ptr &octree, double max_score){
    int deleted_voxels = 0;
    std::shared_ptr<std::vector<LeafContainerT * >> leaf_container_vector_arg(new std::vector<LeafContainerT *>);
    octree->serializeLeafs(*leaf_container_vector_arg);
    for(auto leaf: *leaf_container_vector_arg){
        PointT centroid;
        leaf->getCentroid(centroid);
        auto score = computePlanarityScore(octree, *leaf, centroid);
        if(score > max_score){
            octree->deleteVoxelAtPoint(centroid);
            deleted_voxels++;
        }
    }
    return deleted_voxels;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <resolution>" << std::endl;
        std::cerr << "EXAMPLE: ./octreeVisu bun0.pcd 0.001" << std::endl;
        return -1;
    }
    std::string cloud_path(argv[1]);
    double resolution = std::atof(argv[2]);

    // Load the point cloud
    auto cloud = loadPcd(cloud_path);

    // Visualize the point cloud as octreeq
#if VISUALIZE
    visualizeOctree(cloud, resolution);
#endif



    // Convert the point cloud to an octree
    Octree octree = toOctree(cloud, resolution);
    auto octree_ptr = std::make_shared<Octree>(octree);

    std::cout << "Number of voxels = " << octree_ptr->getLeafCount() << std::endl;

    // Remove low populated voxels
    auto nb_removed = removeVoxelsWithLessThanXPoints(octree_ptr, 10);
    std::cout << "Number of removed voxels = " << nb_removed << std::endl;

    nb_removed = removeNonPlanarVoxels(octree_ptr, 0.03);
    std::cout << "Number of removed voxels = " << nb_removed << std::endl;

    std::cout << "Number of voxels = " << octree_ptr->getLeafCount() << std::endl;

#if VISUALIZE
    visualizeOctree(cloud, resolution);
#endif

#if DEBUG
    checkOctree(octree_ptr);
#endif


    return 0;
}
