//
// Created by amine on 26.05.23.
//
#pragma once

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/types.h>

#ifndef VOXEL_BASED_REGISTRATION_OCTREE_VOXEL_CONTAINER_H
#define VOXEL_BASED_REGISTRATION_OCTREE_VOXEL_CONTAINER_H

namespace pcl::octree {
    template<typename PointT>
    class OctreeVoxelContainer : public OctreeContainerPointIndices {
    public:
        /** \brief Class initialization. */
        OctreeVoxelContainer() { this->reset(); }

        /** \brief Empty class deconstructor. */
        ~OctreeVoxelContainer() override = default;

        /** \brief deep copy function */
        OctreeVoxelContainer *
        deepCopy() const override {
            return (new OctreeVoxelContainer(*this));
        }

        /** \brief Reset leaf container. */
        void
        reset() override {
            using namespace pcl::common;
            leafDataTVector_.clear();
        }

        void
        addPoint(index_t data_arg, const PointT& new_point) {
            using namespace pcl::common;

            leafDataTVector_.push_back(data_arg);
            point_sum_ += new_point;
        }


        // Part of the OctreePointCloudVoxelCentroidContainer class
        //////////////////////////////////////////////////////////////

        /** \brief Equal comparison operator - set to false
         */
        // param[in] OctreeVoxelContainer to compare with
        bool
        operator==(const OctreeContainerBase &) const override {
            return (false);
        }

        /** \brief Calculate centroid of voxel.
         * \param[out] centroid_arg the resultant centroid of the voxel
         */
        void
        getCentroid(PointT &centroid_arg) const {
            using namespace pcl::common;

            auto point_counter = this->getSize();

            if (point_counter) {
                centroid_arg = point_sum_;
                centroid_arg /= static_cast<float>(point_counter);
            }
            else {
                centroid_arg *= 0.0f;
            }
        }

    private:
        PointT point_sum_;

        ///////////////////////////////////////////////////////////

    };


}
#endif //VOXEL_BASED_REGISTRATION_OCTREE_VOXEL_CONTAINER_H
