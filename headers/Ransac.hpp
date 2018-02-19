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

#ifndef COVIS_RANSAC_BENCHMARK_H
#define COVIS_RANSAC_BENCHMARK_H

// Covis
#include <covis/covis.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// Geometric prerejector
#include "../headers/correspondence_rejection_geometric.h"
#include "../headers/correspondence_rejection_poly.h"
// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Point and feature types
typedef pcl::PointCloud<PointT> CloudT;

// Rejection statistics TODO maybe move to benchmark class
struct binaryClassification
{
    int tp = 0; // True-positive (correct accept)
    int tn = 0; // True-negative (correct reject)
    int fp = 0; // False-positive (reject positive)
    int fn = 0; // False-negative (accept negative)
};

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class Ransac
         * @brief Base class for estimating poses
         *
         * This class is used to estimate poses using ransac
         *
         * @author Martin Staal Steenberg
         */
        class ransac {
            public:
                /**
                 * Constructor: set default parameters:
                 *   - sampleSize (@ref setSampleSize()): 3
                 *   - iterations (@ref setIterations()): 10000
                 *   - prerejectionSimilarity (@ref setPrerejectionSimilarity()): 0.9
                 *   - inlierThreshold (@ref setInlierThreshold()): 5
                 *   - inlierFraction (@ref setInlierFraction()): 0.05
                 *   - reestimatePose (@ref setReestimatePose()): true
                 *   - fullEvaluation (@ref setFullEvaluation()): false
                 *   - occlusionReasoning (@ref setOcclusionReasoning()): false
                 *   - occlusionRemoval (@ref setOcclusionRemoval()): false
                 *   - viewAxis (@ref setViewAxis()): 0
                 *   - verbose (@ref setPrerejection()): false
                 */
                ransac() :
                    sampleSize(3),
                    iterations(10000),
                    prerejectionSimilarity(0.9),
                    inlierThreshold(5),
                    inlierFraction(0.05),
                    reestimatePose(true),
                    fullEvaluation(false),
                    occlusionReasoning(false),
                    occlusionRemoval(false),
                    viewAxis(0),
                    verbose(false) {}

                /// empty destructor
                ~ransac() {};

                /**
                 * Set source cloud
                 * @param point cloud
                 */
                inline void setSource( pcl::PointCloud<PointT>::Ptr _source ) {
                    source = _source;
                }

                /**
                 * Set target cloud
                 * @param point cloud
                 */
                inline void setTarget( pcl::PointCloud<PointT>::Ptr _target ) {
                    target = _target;
                }

                /**
                 * Set correspondence between source and target cloud
                 * @param correspondence
                 */
                inline void setCorrespondences( core::Correspondence::VecPtr _corr ) {
                    corr = _corr;
                }

                /**
                 * Set sample size
                 * @param sample size
                 */
                inline void setSampleSize( size_t _sampleSize ) {
                    sampleSize = _sampleSize;
                }

                /**
                 * Set number of RANSAC iterations
                 * @param iterations
                 */
                inline void setIterations( size_t _iterations ) {
                    iterations = _iterations;
                }

                /**
                 * Set Euclidean inlier threshold
                 * @param inlier threshold
                 */
                inline void setInlierThreshold( float _inlierThreshold ) {
                    inlierThreshold = _inlierThreshold;
                }

                /**
                 * Set required inlier fraction (must be in [0,1])
                 * @param inlier fraction
                 */
                inline void setInlierFraction( float _inlierFraction ) {
                    inlierFraction = _inlierFraction;
                }

                /**
                 * Set the re-estimation flag for the pose
                 * In the standard RANSAC formulation, the model is re-estimated using the consensus set, however this
                 * will decrease the speed of the algorithm. If you set this to false, you can expect higher speed at
                 * the expense of potential loss of accuracy in the poses.
                 * @param reestimate pose flag
                 */
                inline void setReestimatePose( bool _reestimatePose ) {
                    reestimatePose = _reestimatePose;
                }

                /**
                 * Enable full evaluation, i.e. evaluation of inliers by point correspondence search between the source
                 * and the target model.
                 * @param full evaluation flag
                 */
                inline void setFullEvaluation( bool _fullEvaluation ) {
                    fullEvaluation = _fullEvaluation;
                }

                /**
                 * Set the prerejection similarity threshold
                 * @param prerejection similarity threshold in [0,1]
                 */
                inline void setPrerejectionSimilarity( float _prerejectionSimilarity ) {
                    prerejectionSimilarity = _prerejectionSimilarity;
                }

                /**
                 * Enable removal of occluded points - be sure to only use this flag if you have target scenes with a well-defined viewpoint
                 * By default, this class assumes the third (z) axis to be the view axis. If this is not the case for your data,
                 * use @ref setViewAxis() to change this
                 * @param occlusionReasoning occlusion
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
                 * Set the dissimilarity prerejection flag
                 * @param prerejection_d
                 */
                void setPrerejectionD( bool _prerejection_d ) {
                    prerejection_d = _prerejection_d;
                }

                /**
                 * Set the geometric prerejection flag
                 * @param prerejection_g
                 */
                void setPrerejectionG( bool _prerejection_g ) {
                    prerejection_g = _prerejection_g;
                }

                /**
                 * Set the geometric prerejection 2 flag
                 * @param prerejection_g2
                 */
                void setPrerejectionG2( bool _prerejection_g2 ) {
                    prerejection_g2 = _prerejection_g2;
                }

                /**
                 * Set the correction flag
                 * @param correction
                 */
                void setCorrection( bool _correction ) {
                    correction = _correction;
                }

                /**
                 * Run RANSAC
                 * @return best detection, if any was found - this can be verified directly in a boolean expression:
                 * @code
                 * covis::core::Detection d = estimate();
                 * if(d)
                 *   // Do something...
                 * @endcode
                 */
                core::Detection estimate();

                /**
                 * TODO FIX
                 * @return best detection, if any was found - this can be verified directly in a boolean expression:
                 */
                core::Detection posePriors();

                 /**
                  * Benchmark prerejection methods and returns a binaryClassification
                  * containing information about the quality of the prerejection method
                  * @param ground truth pose
                  * @return binaryClassification
                  */
                std::vector<binaryClassification> benchmark( Eigen::Matrix4f ground_truth );

                /**
                 * Benchmark the correction method
                 * @param ground truth pose
                 */
                void benchmark_correction( Eigen::Matrix4f ground_truth );

            private:
                /// Source point cloud
                pcl::PointCloud<PointT>::Ptr source;

                /// Target point cloud
                pcl::PointCloud<PointT>::Ptr target;

                /// Correspondences
                core::Correspondence::VecPtr corr;

                /// Sample size
                size_t sampleSize;

                /// Number of ransac iterations
                size_t iterations;

                /// If prerejection is set to true, use this polygonal similarity threshold
                float prerejectionSimilarity;

                /// Euclidean inlier threshold
                float inlierThreshold;

                /// Visibility or required fraction of inliers of the source points [0,1]
                float inlierFraction;

                /// If set to true (default), a pose is re-estimated if it has enough inliers
                bool reestimatePose;

                /// Full evaluation flag
                bool fullEvaluation;

                /// Enable removal of occluded points
                bool occlusionReasoning;

                /// Occlusion removal flag
                bool occlusionRemoval;

                /// Specify which of the three sensor axes points in the viewing direction - typically this is the z-axis
                int viewAxis;

                /// Verbose flag
                bool verbose;

                /// Prereject dissimilarity flag
                bool prerejection_d = false;

                /// Prereject geometric flag
                bool prerejection_g = false;

                /// Prereject geometric 2 flag
                bool prerejection_g2 = false;

                /// Correction method flag
                bool correction = false;
        };
    }
}

#endif
