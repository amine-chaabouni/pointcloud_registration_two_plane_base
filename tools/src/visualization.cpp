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


void addNormals(const PointT &point_on_plane, Eigen::Vector3f &normal, pcl::visualization::PCLVisualizer &viewer,
                const std::string &name) {
    PointT tip_of_arrow;
    tip_of_arrow.x = point_on_plane.x + normal(0);
    tip_of_arrow.y = point_on_plane.y + normal(1);
    tip_of_arrow.z = point_on_plane.z + normal(2);
    viewer.addArrow<PointT, PointT>(tip_of_arrow, point_on_plane, 1, 0, 0, false, name);
}

void preparePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes,
                          pcl::visualization::PCLVisualizer &viewer, const std::string &name, int r, int g, int b) {
    // Add the point cloud to the viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(cloud, r, g, b);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, red_color, name);
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
        pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(plane_cloud, 255 - (i * rate), 0,
                                                                           i * rate);
        auto plane_name = name + " plane " + std::to_string(i);
        viewer.addPointCloud<pcl::PointXYZ>(plane_cloud, red_color, plane_name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, plane_name);
    }

    // Set the size of the point
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
}

void prepareTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud,
                           pcl::visualization::PCLVisualizer &viewer) {
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(first_cloud, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(first_cloud, red_color, "source_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(second_cloud, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(second_cloud, blue_color, "target_cloud");

    // Set the size of the point
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "target_cloud");
}

void visualizeCorrespondences(const CompleteCloud &first_cloud, const CompleteCloud &second_cloud,
                              const std::vector<std::pair<int, int>> &optimal_correspondence) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("pcd viewer");

    std::vector<pcl::Indices> indices;
    auto first_cloud_indices = std::get<1>(first_cloud).second;
    for (auto corr: optimal_correspondence)
        indices.emplace_back(first_cloud_indices[corr.first]);
    preparePlanesOnCloud(std::get<0>(first_cloud)->getInputCloud(), indices, viewer, "first_cloud", 0, 255, 0);

    indices.clear();
    auto second_cloud_indices = std::get<1>(second_cloud).second;
    for (auto corr: optimal_correspondence)
        indices.emplace_back(second_cloud_indices[corr.second]);
    preparePlanesOnCloud(std::get<0>(second_cloud)->getInputCloud(), indices, viewer, "second_cloud", 120, 120, 0);

    // Display the point cloud
    viewer.spin();
    while (!viewer.wasStopped()) {
    }
    viewer.close();
}

void visualizeOctree(const PointCloudPtr &cloud, const Octree::Ptr &octree) {
    OctreeViz viewer{cloud, octree};
}

void visualizeTwoPointClouds(const PointCloud::ConstPtr &first_cloud, const PointCloud::ConstPtr &second_cloud) {
    // Visualize the result
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // Add the point cloud to the viewer
    prepareTwoPointClouds(first_cloud, second_cloud, viewer);
    // Display the point cloud
    viewer.spin();
    while (!viewer.wasStopped()) {
    }
    viewer.close();
}

void visualizeBases(const PointCloud::ConstPtr &first_cloud, const Planes &first_planes,
                    const PointCloud::ConstPtr &second_cloud, const Planes &second_planes) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("pcd viewer");

    auto first_indices = first_planes.second;
    auto second_indices = second_planes.second;
    // Add the point cloud to the viewer
    preparePlanesOnCloud(first_cloud, first_indices, viewer, "first_cloud", 0, 255, 0);
    preparePlanesOnCloud(second_cloud, second_indices, viewer, "second_cloud", 120, 120, 0);

    // Display normals
    auto point_on_plane = first_cloud->at(first_indices[0][0]);
    auto normal = first_planes.first[0].first;
    addNormals(point_on_plane, normal, viewer, "first_normal_first_cloud");

    point_on_plane = first_cloud->at(first_indices[1][0]);
    normal = first_planes.first[1].first;
    addNormals(point_on_plane, normal, viewer, "second_normal_first_cloud");

    point_on_plane = second_cloud->at(second_indices[0][0]);
    normal = second_planes.first[0].first;
    addNormals(point_on_plane, normal, viewer, "first_normal_second_cloud");

    point_on_plane = second_cloud->at(second_indices[1][0]);
    normal = second_planes.first[1].first;
    addNormals(point_on_plane, normal, viewer, "second_normal_second_cloud");

    // Display the point cloud
    viewer.spin();
    while (!viewer.wasStopped()) {
    }
    viewer.close();
}

void visualizePlanesOnCloud(const PointCloud::ConstPtr &cloud, const std::vector<pcl::Indices> &planes) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("pcd viewer");
    // Add the point cloud to the viewer
    preparePlanesOnCloud(cloud, planes, viewer, "point cloud", 255, 255, 255);
    // Display the point cloud
    viewer.spin();
    while (!viewer.wasStopped()) {
    }
    viewer.close();
}


void visualizeFinalResults(const CompleteCloud &source_cloud, const CompleteCloud &target_cloud,
                           const Correspondences &optimal_correspondence, const Eigen::Matrix4f &transformation,
                           double lidar_resolution) {
    // Visualize Correspondances
    visualizeCorrespondences(source_cloud, target_cloud, optimal_correspondence);

    auto first_octree_ptr = std::get<0>(source_cloud);
    auto first_planes = std::get<1>(source_cloud);
    auto first_bases = std::get<2>(source_cloud);

    auto second_octree_ptr = std::get<0>(target_cloud);
    auto second_planes = std::get<1>(target_cloud);
    auto second_bases = std::get<2>(target_cloud);

    PointCloudPtr transformed_cloud = transformTargetPointCloud(first_octree_ptr->getInputCloud(), transformation);
    visualizeTwoPointClouds(second_octree_ptr->getInputCloud(), transformed_cloud);

    // Visualize Correspondances
    Octree::Ptr transformed_octree_ptr = std::make_shared<Octree>(toOctree(transformed_cloud, lidar_resolution));
    CompleteCloud transformed_cloud_complete = std::make_tuple(transformed_octree_ptr, first_planes, first_bases);
    visualizeCorrespondences(transformed_cloud_complete, target_cloud, optimal_correspondence);
}