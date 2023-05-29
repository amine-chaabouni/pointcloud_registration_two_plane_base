//
// Created by amine on 26.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_OCTREE_VIEWER_H
#define VOXEL_BASED_REGISTRATION_OCTREE_VIEWER_H

#include <thread>

#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/filter.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

#include "octree_search_centroid.h"
#include "octree_voxel_container.h"

using namespace std::chrono_literals;

class OctreeViewer {
public:
    using Octree = pcl::octree::OctreeVoxelBasedRegistration<pcl::PointXYZ, pcl::octree::OctreeVoxelContainer<pcl::PointXYZ>>;
    OctreeViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const Octree::Ptr& input_octree);

private:
    //========================================================
    // PRIVATE ATTRIBUTES
    //========================================================
    //visualizer
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;

    pcl::visualization::PCLVisualizer viz;
    //original cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    //displayed_clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
    // cloud which contains the voxel center
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
    //octree
    Octree::Ptr octree;
    //level
    int displayedDepth;
    //bool to decide what should be display
    bool wireframe;
    bool show_cubes_, show_centroids_, show_original_points_;
    float point_size_;

    //========================================================
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *);

    void run();

    void showLegend();

    void update();

    void clearView();

    void showCubes(double voxelSideLen);

    void extractPointsAtLevel(int depth);

    bool IncrementLevel();

    bool DecrementLevel();
};


#endif //VOXEL_BASED_REGISTRATION_OCTREE_VIEWER_H
