//
// Created by amine on 30.05.23.
//

#include "visualization.h"


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
    viewer.close();

#endif
}

void visualizePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("pcd viewer");
    // Add the point cloud to the viewer
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "point cloud");
    // Set the background of the viewer to black
    viewer.setBackgroundColor(0, 0, 0);

    int i = 0;
    auto rate = 255 / planes.size();
    for (auto indices: planes) {
        ++i;
        PointCloudPtr plane_cloud(new PointCloud);
        for (auto idx: indices) {
            plane_cloud->push_back(cloud->at(idx));
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(plane_cloud, 255 - (i * rate), 0,
                                                                                  i * rate);
        auto name = "plane " + std::to_string(i);
        viewer.addPointCloud<pcl::PointXYZ>(plane_cloud, red_color, name);
    }

    // Set the size of the point
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
    // Display the point cloud
    viewer.spin();
    while (!viewer.wasStopped()) {
    }
    viewer.close();
}

void visualizeCorrespondences(const CompleteCloud& first_cloud, const CompleteCloud& second_cloud,
                              const std::vector<std::pair<int, int>>& optimal_correspondence) {
    std::vector<pcl::Indices> indices;
    auto first_cloud_indices = std::get<1>(first_cloud).second;
    for (auto corr: optimal_correspondence)
        indices.emplace_back(first_cloud_indices[corr.first]);
    visualizePlanesOnCloud(std::get<0>(first_cloud)->getInputCloud(), indices);

    indices.clear();
    auto second_cloud_indices = std::get<1>(second_cloud).second;
    for (auto corr: optimal_correspondence)
        indices.emplace_back(second_cloud_indices[corr.second]);
    visualizePlanesOnCloud(std::get<0>(second_cloud)->getInputCloud(), indices);
}

void visualizeOctree(const PointCloudPtr &cloud, const Octree::Ptr &octree) {
    OctreeViz viewer{cloud, octree};
}

void VisualizeTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud){
    // Visualize the result
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(first_cloud, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(first_cloud, red_color, "source_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(second_cloud, 0,0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(second_cloud, blue_color, "target_cloud");

    // Set the size of the point
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "target_cloud");
    // Display the point cloud
    viewer.spin();
    while (!viewer.wasStopped()) {
    }
    viewer.close();
}