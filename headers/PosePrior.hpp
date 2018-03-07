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
                 *   - inlierThreshold (@ref setInlierThreshold()): 5
                 *   - inlierFraction (@ref setInlierFraction()): 0.05
                 *   - occlusionReasoning (@ref setOcclusionReasoning()): false
                 *   - viewAxis (@ref setViewAxis()): 0
                 *   - verbose (@ref setPrerejection()): false
                 */
                posePrior() :
                    inlierThreshold(5),
                    inlierFraction(0.05),
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
                 * Set Euclidean inlier threshold
                 * @param inlier threshold
                 */
                inline void setInlierThreshold( float _inlierThreshold ) {
                    COVIS_ASSERT(_inlierThreshold > 0);
                    inlierThreshold = _inlierThreshold;
                }

                /**
                 * Set required inlier fraction (must be in [0,1])
                 * @param inlier fraction
                 */
                inline void setInlierFraction( float _inlierFraction ) {
                    COVIS_ASSERT(_inlierFraction >= 0 && _inlierFraction <= 1);
                    inlierFraction = _inlierFraction;
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

                /// Euclidean inlier threshold
                float inlierThreshold;

                /// Visibility or required fraction of inliers of the source points [0,1]
                float inlierFraction;

                /// Enable removal of occluded points
                bool occlusionReasoning;

                /// Specify which of the three sensor axes points in the viewing direction - typically this is the z-axis
                int viewAxis;

                /// Verbose flag
                bool verbose;

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
                    Eigen::Vector3f x = z.cross(y);
                    Eigen::Matrix3f frame;
                    frame.col(0) = x;
                    frame.col(1) = y;
                    frame.col(2) = z;

                    return frame;
                }
        };
    }
}

#endif
