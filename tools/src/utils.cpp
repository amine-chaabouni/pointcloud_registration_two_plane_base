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

void visualizeOctree(const PointCloudPtr &cloud, double resolution) {
    OctreeViewer viewer{cloud, resolution};
}

