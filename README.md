# pointcloud_registration_two_plane_base

Unofficial implementation of paper ["Pairwaise coarse registration of point clouds by traversing voxel-based 2 plane
bases"](https://www.tandfonline.com/doi/epdf/10.1080/01431161.2022.2130725?needAccess=true&role=button)

## Additional Changes to the paper

1. The same plane is only generated once to avoid redundant computation.
2. When computing the LCP score, the number of planes generated are taken into account. If a plane is generated twice,
   the LCP takes that into account.
3. When assigning correspondences between planes, the correspondence between the bases being processed is encouraged. If
   one of the bases is corresponded to the other, but not the other way around, we force the correspondence to be
   mutual.
4. This implementation aims at registering point clouds taken by sensors correctly oriented on the Z axis. As such, we
   force the rotation to be only on the Z axis.

### Justification of the changes

An ablation study has been conducted on a particular example (`data/data/0010`) to justify the changes. The results can
be found in `data/lidar_lidar/0_5-300-0_5-300-0_6/0010/ablation_study` and `data/lidar_rgbd/0_5-300-0_5-300-0_6/0010/ablation_study`. The ablation study shows that the changes made to the
paper improve the registration results.

## Repository structure

```
├── README.md
├── CMakeLists.txt
├── main.cpp
├── tools --> containing the classes and functions for tools used to process pointclouds, perform the registration and visualize the results
│   ├── CMakeLists.txt
│   ├── include
│   └── src
├── custom_lib --> containing the classes and functions for customized pcl objects
│   ├── CMakeLists.txt
│   ├── include
│   └── src
└── data --> containing the data for the registration
    ├── data
    │   ├── 0001
    │   │   ├── 0001_setup.png
    │   │   ├── lidar_0_0_*.pcd
    │   │   ├── lidar_0_1_*.pcd
    │   │   ├── rgbd_0_0_*.pcd
    │   │   └── rgbd_0_1_*.pcd
    │   ├── 0002
    │   ├── 0003
    │   ├── 0004
    │   ├── 0005
    │   ├── 0006
    │   ├── 0007
    │   ├── 0008
    │   ├── 0009
    │   └── 0010
    ├── lidar_lidar --> containing the lidar-lidar registration results
    └── lidar_rgbd --> containing the lidar-rgbd registration results
```

## Usage

```
./main <path_source_cloud> <path_target_cloud> <voxel_size_source_cloud> <minimum_nb_points_per_voxel_source> <voxel_size_target_cloud> <minimum_nb_points_per_voxel_target> <planarity_score>
```

Example:

```
./main data/data/0010/lidar_0_0* data/data/0010/rgbd_0_1* 0.5 300 0.5 300 0.6
```