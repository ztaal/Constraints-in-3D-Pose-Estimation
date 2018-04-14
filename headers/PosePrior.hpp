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

#ifndef COVIS_POSE_PRIOR_H
#define COVIS_POSE_PRIOR_H

// Covis
#include <covis/covis.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Point and feature types
typedef pcl::PointCloud<PointT> CloudT;

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class PosePrior
         * @brief Base class for estimating poses
         *
         * This class is used to estimate poses on table tops using pose priors
         *
         * @author Martin Staal Steenberg
         */
        class posePrior {
            public:
                /**
                 * Constructor: set default parameters:
                 *   - inlierThreshold (@ref setInlierThreshold()): 10
                 *   - icpIterations (@ref setIcpIterations()): 100
                 *   - occlusionReasoning (@ref setOcclusionReasoning()): false
                 *   - viewAxis (@ref setViewAxis()): 0
                 *   - verbose (@ref setPrerejection()): false
                 */
                posePrior() :
                    inlierThreshold(10),
                    // icpIterations(50), // Hintertoisser // BEST 50
                    icpIterations(100), // Tejani 100
                    occlusionReasoning(false),
                    viewAxis(0),
                    verbose(false) {}

                /// empty destructor
                ~posePrior() {};

                /**
                 * Set source cloud
                 * @param point cloud
                 */
                inline void setSource( pcl::PointCloud<PointT>::Ptr _source ) {
                    COVIS_ASSERT(_source);
                    source = _source;
                }

                /**
                 * Set target cloud
                 * @param point cloud
                 */
                inline void setTarget( pcl::PointCloud<PointT>::Ptr _target ) {
                    COVIS_ASSERT(_target);
                    target = _target;
                }

                /**
                 * Set correspondence between source and target cloud
                 * @param correspondence
                 */
                inline void setCorrespondences( core::Correspondence::VecPtr _corr ) {
                    COVIS_ASSERT(_corr);
                    bool allEmpty = true;
                    for(size_t i = 0; i < _corr->size(); ++i) {
                        if (!(*_corr)[i].empty()) {
                            allEmpty = false;
                            break;
                        }
                    }
                    COVIS_ASSERT_MSG(!allEmpty, "All empty correspondences input to RANSAC!");
                    corr = _corr;
                }

                /**
                 * Set source centroid distance
                 * @param distance
                 */
                inline void setSrcCentroidDist( float _srcCentroidDist ) {
                    COVIS_ASSERT( _srcCentroidDist > 0 );
                    srcCentroidDist = _srcCentroidDist;
                }

                /**
                 * Set Euclidean inlier threshold
                 * @param inlier threshold
                 */
                inline void setInlierThreshold( float _inlierThreshold ) {
                    COVIS_ASSERT(_inlierThreshold > 0);
                    inlierThreshold = _inlierThreshold;
                }

                /**
                 * Set number of Iterative Closest Point iterations
                 * @param iterations
                 */
                inline void setIcpIterations( int _icpIterations ) {
                    COVIS_ASSERT( _icpIterations > 0 );
                    icpIterations = _icpIterations;
                }

                /**
                 * Set occlusion reasoning flag
                 * @param occlusionReasoning flag
                 */
                inline void setOcclusionReasoning( bool _occlusionReasoning ) {
                    occlusionReasoning = _occlusionReasoning;
                }

                /**
                 * Set the view axis - default z
                 * @param veiw axis [0, 1, 2]
                 */
                inline void setViewAxis( int _viewAxis ) {
                    viewAxis = _viewAxis;
                }

                /**
                 * Set modelIndex
                 * @param index [1, 2, 3, 4, 5, 6]
                 */
                inline void setModelIndex( int _modelIndex ) {
                    modelIndex = _modelIndex;
                }

                /**
                 * Set verbose flag for printing
                 * @param verbose flag
                 */
                inline void setVerbose( bool _verbose ) {
                    verbose = _verbose;
                }

                /**
                 * Run pose prior estimation
                 * @return best detection, if any was found - this can be verified directly in a boolean expression:
                 * @code
                 * covis::core::Detection d = estimate();
                 * if(d)
                 *   // Do something...
                 * @endcode
                 */
                core::Detection estimate();

            private:
                /// Source point cloud
                pcl::PointCloud<PointT>::Ptr source;

                /// Target point cloud
                pcl::PointCloud<PointT>::Ptr target;

                /// Correspondences
                core::Correspondence::VecPtr corr;

                /// Distance to the cloest point from source centroid
                double srcCentroidDist;

                // double lower_plane_dist_threshold = 5
                // double upper_plane_dist_threshold = 150
                // double plane_inlier_threshold = 0.15
                // double plane_rejection_threshold = 0.5
                // bool constraint_1 = true
                // dobule constraint_1_threshold = 0
                // bool constraint_2 = true
                // dobule constraint_2_threshold = 0.8
                // bool constraint_3 = true
                // dobule constraint_3_threshold = 1.2
                // bool constraint_4 = true
                // dobule constraint_4_threshold = 0.2
                // bool constraint_5 = true
                // dobule constraint_5_threshold = 2

                /// Euclidean inlier threshold
                float inlierThreshold;

                /// Iterative Closest Point Iterations
                float icpIterations;

                /// Enable removal of occluded points
                bool occlusionReasoning;

                /// Specify which of the three sensor axes points in the viewing direction - typically this is the z-axis
                int viewAxis;

                /// Verbose flag
                bool verbose;

                /// Model index used for correcting axis
                int modelIndex;

                /**
                * Compute the ortogonal basis from the plane normal
                * @param plane coefficients
                * @return ortogonal frame
                */
                inline Eigen::Matrix3f ortogonal_basis(pcl::ModelCoefficients::Ptr coefficients)
                {
                    // Create ortogonal basis
                    double a = coefficients->values[0];
                    double b = coefficients->values[1];
                    double c = coefficients->values[2];
                    Eigen::Vector3f z(a, b, c);
                    Eigen::Vector3f y(0, -c, b);
                    Eigen::Vector3f x = y.cross(z);
                    Eigen::Matrix3f frame;
                    frame.col(0) = x;
                    frame.col(1) = y;
                    frame.col(2) = z;

                    return frame;
                }

                /**
                * Compute median of vector
                * @param vector
                */
                inline double
                median( std::vector<double> v )
                {
                    std::sort(v.begin(), v.end());
                    v.erase(v.begin() + int(v.size() / 6), v.begin() + v.size());
                    size_t size = v.size();

                    if (size % 2 == 0)
                        return (v[size / 2 - 1] + v[size / 2]) / 2;

                    return v[size / 2];
                }
        };
    }
}

#endif
