//
// Created by amine on 26.05.23.
//

#ifndef VOXEL_BASED_REGISTRATION_OCTREE_SEARCH_CENTROID_H
#define VOXEL_BASED_REGISTRATION_OCTREE_SEARCH_CENTROID_H

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/types.h>
#include "octree_voxel_container.h"

namespace pcl::octree {

    /** \brief @b Octree pointcloud search class
    * \note This class provides several methods for spatial neighbor search based on octree
    * structure
    * \tparam PointT type of point used in pointcloud
    * \ingroup octree
    * \author Amine Chaabouni
    */
    template<typename PointT,
            typename LeafContainerT = OctreeVoxelContainer<PointT>,
            typename BranchContainerT = OctreeContainerEmpty>
    class OctreeVoxelBasedRegistration
            : public OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT> {
    public:
        using Ptr = shared_ptr<OctreeVoxelBasedRegistration<PointT, LeafContainerT>>;
        using ConstPtr =
                shared_ptr<const OctreeVoxelBasedRegistration<PointT, LeafContainerT>>;

        using OctreeT = OctreePointCloud<PointT, LeafContainerT, BranchContainerT>;
        using LeafNode = typename OctreeT::LeafNode;
        using BranchNode = typename OctreeT::BranchNode;

        /** \brief OctreeVoxelBasedRegistration class constructor.
         * \param[in] resolution_arg octree resolution at lowest octree level
         */
        explicit OctreeVoxelBasedRegistration(const double resolution_arg)
                : OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>(resolution_arg) {}

        /** \brief Add DataT object to leaf node at octree key.
         * \param pointIdx_arg
         */
        void
        addPointIdx(const uindex_t pointIdx_arg) override {
            OctreeKey key;

            assert(pointIdx_arg < this->input_->size());

            const PointT &point = (*this->input_)[pointIdx_arg];

            // make sure bounding box is big enough
            this->adoptBoundingBoxToPoint(point);

            // generate key
            this->genOctreeKeyforPoint(point, key);

            // add point to octree at key
            LeafContainerT *container = this->createLeaf(key);
            container->addPoint(pointIdx_arg, point);
        }


        /** \brief Get centroid for a single voxel addressed by a PointT point.
         * \param[in] point_arg point addressing a voxel in octree
         * \param[out] voxel_centroid_arg centroid is written to this PointT reference
         * \return "true" if voxel is found; "false" otherwise
         */
        bool
        getVoxelCentroidAtPoint(const PointT &point_arg, PointT &voxel_centroid_arg) const;

        /** \brief Get centroid for a single voxel addressed by a PointT point from input
         * cloud.
         * \param[in] point_idx_arg point index from input cloud addressing a voxel in octree
         * \param[out] voxel_centroid_arg centroid is written to this PointT reference
         * \return "true" if voxel is found; "false" otherwise
         */
        inline bool
        getVoxelCentroidAtPoint(const index_t &point_idx_arg,
                                PointT &voxel_centroid_arg) const {
            // get centroid at point
            return (this->getVoxelCentroidAtPoint((*this->input_)[point_idx_arg],
                                                  voxel_centroid_arg));
        }

        /** \brief Get PointT vector of centroids for all occupied voxels.
         * \param[out] voxel_centroid_list_arg results are written to this vector of PointT
         * elements
         * \return number of occupied voxels
         */
        uindex_t
        getVoxelCentroids(
                typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::
                AlignedPointTVector &voxel_centroid_list_arg) const;

        /** \brief Recursively explore the octree and output a PointT vector of centroids for
         * all occupied voxels.
         * \param[in] branch_arg: current branch node
         * \param[in] key_arg: current key
         * \param[out] voxel_centroid_list_arg results are written to this vector of PointT
         * elements
         */
        void
        getVoxelCentroidsRecursive(
                const BranchNode *branch_arg,
                OctreeKey &key_arg,
                typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::
                AlignedPointTVector &voxel_centroid_list_arg) const;

        /** \brief Remove voxels that don't contain a minimum number of points.
        * \param[in] min_points_per_voxel_arg minimum number of points per voxel
        * \return number of removed voxels
        */
        int
        removeVoxelsWithLessThanXPoints(int min_points_per_voxel_arg);
    };

}
#endif //VOXEL_BASED_REGISTRATION_OCTREE_SEARCH_CENTROID_H
