//
// Created by amine on 26.05.23.
// from pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp
//

#include "octree_search_centroid.h"
#include <pcl/octree/impl/octree_pointcloud.hpp>

namespace pcl::octree{

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT, typename LeafContainerT, typename BranchContainerT>
    bool
    OctreeVoxelBasedRegistration<PointT, LeafContainerT, BranchContainerT>::
    getVoxelCentroidAtPoint(const PointT& point_arg, PointT& voxel_centroid_arg) const
    {
        OctreeKey key;
        LeafContainerT* leaf = NULL;

        // generate key
        genOctreeKeyforPoint(point_arg, key);

        leaf = this->findLeaf(key);
        if (leaf)
            leaf->getCentroid(voxel_centroid_arg);

        return (leaf != NULL);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT, typename LeafContainerT, typename BranchContainerT>
    uindex_t
    OctreeVoxelBasedRegistration<PointT, LeafContainerT, BranchContainerT>::
    getVoxelCentroids(
            typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::
            AlignedPointTVector& voxel_centroid_list_arg) const
    {
        OctreeKey new_key;

        // reset output vector
        voxel_centroid_list_arg.clear();
        voxel_centroid_list_arg.reserve(this->leaf_count_);

        getVoxelCentroidsRecursive(this->root_node_, new_key, voxel_centroid_list_arg);

        // return size of centroid vector
        return (voxel_centroid_list_arg.size());
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT, typename LeafContainerT, typename BranchContainerT>
    void
    OctreeVoxelBasedRegistration<PointT, LeafContainerT, BranchContainerT>::
    getVoxelCentroidsRecursive(
            const BranchNode* branch_arg,
            OctreeKey& key_arg,
            typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::
            AlignedPointTVector& voxel_centroid_list_arg) const
    {
        // iterate over all children
        for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
            // if child exist
            if (branch_arg->hasChild(child_idx)) {
                // add current branch voxel to key
                key_arg.pushBranch(child_idx);

                OctreeNode* child_node = branch_arg->getChildPtr(child_idx);

                switch (child_node->getNodeType()) {
                    case BRANCH_NODE: {
                        // recursively proceed with indexed child branch
                        getVoxelCentroidsRecursive(static_cast<const BranchNode*>(child_node),
                                                   key_arg,
                                                   voxel_centroid_list_arg);
                        break;
                    }
                    case LEAF_NODE: {
                        PointT new_centroid;

                        auto* container = static_cast<LeafNode*>(child_node);

                        container->getContainer().getCentroid(new_centroid);

                        voxel_centroid_list_arg.push_back(new_centroid);
                        break;
                    }
                    default:
                        break;
                }

                // pop current branch voxel from key
                key_arg.popBranch();
            }
        }
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT, typename LeafContainerT, typename BranchContainerT>
    int
    OctreeVoxelBasedRegistration<PointT, LeafContainerT, BranchContainerT>::
    removeVoxelsWithLessThanXPoints(int min_points_per_voxel_arg)
    {
        int deleted_voxels = 0;
        std::shared_ptr<std::vector< LeafContainerT* >> leaf_container_vector_arg(new std::vector< LeafContainerT* >);
        this->serializeLeafs(*leaf_container_vector_arg);

        for(auto leaf: *leaf_container_vector_arg){
            if(leaf->getSize() < min_points_per_voxel_arg){
                PointT centroid;
                leaf->getCentroid(centroid);
                this->deleteVoxelAtPoint(centroid);
                deleted_voxels++;
            }
        }
        return deleted_voxels;
    }

}