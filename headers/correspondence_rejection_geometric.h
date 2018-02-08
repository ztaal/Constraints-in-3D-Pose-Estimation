// Copyright (c) 2013, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_GEOMETRIC_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_GEOMETRIC_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  namespace registration
  {
    /** \brief CorrespondenceRejectorGeometric implements a correspondence rejection method that exploits low-level and
      * pose-invariant geometric constraints between two point sets by comparing edge lengths with their indices.
      *
      * \author Martin Staal Steenberg
      */
    template <typename SourceT, typename TargetT>
    class PCL_EXPORTS CorrespondenceRejectorGeometric
    {

      public:
        typedef boost::shared_ptr<CorrespondenceRejectorGeometric> Ptr;
        typedef boost::shared_ptr<const CorrespondenceRejectorGeometric> ConstPtr;

        typedef pcl::PointCloud<SourceT> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<TargetT> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        /** \brief Empty constructor */
        CorrespondenceRejectorGeometric ()
        {
        }

        /** \brief Provide a source point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
          * \param[in] cloud a cloud containing XYZ data
          */
        inline void
        setInputSource (const PointCloudSourceConstPtr &cloud)
        {
          input_ = cloud;
        }

        /** \brief Provide a source point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
          * \param[in] cloud a cloud containing XYZ data
          */
        inline void
        setInputCloud (const PointCloudSourceConstPtr &cloud)
        {
          input_ = cloud;
        }

        /** \brief Provide a target point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
          * \param[in] target a cloud containing XYZ data
          */
        inline void
        setInputTarget (const PointCloudTargetConstPtr &target)
        {
          target_ = target;
        }

        /** \brief See if this rejector requires source points */
        bool
        requiresSourcePoints () const
        { return (true); }

        /** \brief Blob method for setting the source cloud */
        void
        setSourcePoints (pcl::PCLPointCloud2::ConstPtr cloud2)
        {
          PointCloudSourcePtr cloud (new PointCloudSource);
          fromPCLPointCloud2 (*cloud2, *cloud);
          setInputSource (cloud);
        }

        /** \brief See if this rejector requires a target cloud */
        bool
        requiresTargetPoints () const
        { return (true); }

        /** \brief Method for setting the target cloud */
        void
        setTargetPoints (pcl::PCLPointCloud2::ConstPtr cloud2)
        {
          PointCloudTargetPtr cloud (new PointCloudTarget);
          fromPCLPointCloud2 (*cloud2, *cloud);
          setInputTarget (cloud);
        }

        /** \brief Geometric rejection of a single polygon, indexed by two point index vectors
          * \param source_indices indices of polygon points in \ref input_
          * \param target_indices corresponding indices of polygon points in \ref target_
          * \return true if the longests, second longest, ..., etc. edge index of the target corresponds to equivalent index in the source
          */
        inline bool
        geometricConstraint (const std::vector<int>& source_indices, const std::vector<int>& target_indices)
        {
            unsigned int sampleSize = source_indices.size();
            std::vector<double> source_sides( sampleSize );
            std::vector<double> target_sides( sampleSize );
            for (unsigned int i = 0; i < sampleSize; i++) {
                source_sides.push_back( computeSquaredDistance( input_->points[source_indices[i]],
                                        input_->points[source_indices[(i + 1) % sampleSize]]) );
                target_sides.push_back( computeSquaredDistance(target_->points[target_indices[i]],
                                        target_->points[target_indices[(i + 1) % sampleSize]]) );
            }
            while ( source_sides.size() > 1 ) {
                int idx = std::distance( source_sides.begin(), std::max_element(source_sides.begin(), source_sides.end()) );
                int jdx = std::distance( target_sides.begin(), std::max_element(target_sides.begin(), target_sides.end()) );
                if ( idx != jdx ) {
                    return false;
                } else if ( source_sides.size() > 1 ) {
                    source_sides.erase( source_sides.begin() + idx );
                    target_sides.erase( target_sides.begin() + jdx );
                }
            }
            return true;
        }

        /** \brief Geometric rejection of a single polygon, indexed by two point index vectors
          * \param source_indices indices of polygon points in \ref input_
          * \param target_indices corresponding indices of polygon points in \ref target_
          * \return true if the longests, second longest, ..., etc. edge index of the target corresponds to equivalent index in the source
          */
        inline bool
        geometricConstraint2 (const std::vector<int>& source_indices, const std::vector<int>& target_indices)
        {
            unsigned int sampleSize = source_indices.size();
            std::vector<double> source_sides( sampleSize );
            std::vector<double> target_sides( sampleSize );
            for (unsigned int i = 0; i < sampleSize; i++) {
                source_sides.push_back( computeSquaredDistance( input_->points[source_indices[i]],
                                        input_->points[source_indices[(i + 1) % sampleSize]]) );
                target_sides.push_back( computeSquaredDistance(target_->points[target_indices[i]],
                                        target_->points[target_indices[(i + 1) % sampleSize]]) );
            }
            if ( sampleSize == 3 ) {
                if ( source_sides[0] == source_sides[1] && source_sides[1] == source_sides[2] ) {
                    return true;
                } else if ( source_sides[0] == source_sides[1] ) {
                    source_sides.erase( source_sides.begin() );
                    target_sides.erase( target_sides.begin() );
                } else if ( source_sides[1] == source_sides[2] ) {
                    source_sides.erase( source_sides.begin() + 1 );
                    target_sides.erase( target_sides.begin() + 1 );
                } else if ( source_sides[0] == source_sides[2] ) {
                    source_sides.erase( source_sides.begin() );
                    target_sides.erase( target_sides.begin() );
                }
            }
            while ( source_sides.size() > 1 ) {
                int idx = std::distance( source_sides.begin(), std::max_element(source_sides.begin(), source_sides.end()) );
                int jdx = std::distance( target_sides.begin(), std::max_element(target_sides.begin(), target_sides.end()) );
                if ( idx != jdx ) {
                    return false;
                } else if ( source_sides.size() > 1 ) {
                    source_sides.erase( source_sides.begin() + idx );
                    target_sides.erase( target_sides.begin() + jdx );
                }
            }
            return true;
        }

      protected:
        /** \brief Squared Euclidean distance between two points using the members x, y and z
          * \param p1 first point
          * \param p2 second point
          * \return squared Euclidean distance
          */
        inline float
        computeSquaredDistance (const SourceT& p1, const TargetT& p2)
        {
          const float dx = p2.x - p1.x;
          const float dy = p2.y - p1.y;
          const float dz = p2.z - p1.z;

          return (dx*dx + dy*dy + dz*dz);
        }

        /** \brief The input point cloud dataset */
        PointCloudSourceConstPtr input_;

        /** \brief The input point cloud dataset target */
        PointCloudTargetConstPtr target_;
    };
  }
}

#endif    // PCL_REGISTRATION_CORRESPONDENCE_REJECTION_GEOMETRIC_H_
