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

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Point and feature types
typedef pcl::PointCloud<PointT> CloudT;

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

                ransac() {};

                /// empty destructor
                ~ransac() {};

                ///
                inline void setSource( pcl::PointCloud<PointT>::Ptr _source ) {
                    source = _source;
                }

                ///
                inline void setTarget( pcl::PointCloud<PointT>::Ptr _target ) {
                    target = _target;
                }

                ///
                inline void setCorrespondences( core::Correspondence::VecPtr _corr ) {
                    corr = _corr;
                }

                ///
                inline void setIterations( size_t _iterations ) {
                    iterations = _iterations;
                }

                ///
                inline void setSampleSize( size_t _sampleSize ) {
                    sampleSize = _sampleSize;
                }

                ///
                inline void setInlierThreshold( float _inlierThreshold ) {
                    inlierThreshold = _inlierThreshold;
                }

                ///
                inline void setInlierFraction( float _inlierFraction ) {
                    inlierFraction = _inlierFraction;
                }

                ///
                inline void setReestimatePose( bool _reestimatePose ) {
                    reestimatePose = _reestimatePose;
                }

                ///
                inline void setFullEvaluation( bool _fullEvaluation ) {
                    fullEvaluation = _fullEvaluation;
                }

                ///
                inline void setPrerejectionSimilarity( float _prerejectionSimilarity ) {
                    prerejectionSimilarity = _prerejectionSimilarity;
                }

                ///
                inline void setOcclusionReasoning( bool _occlusionReasoning ) {
                    occlusionReasoning = _occlusionReasoning;
                }

                ///
                inline void setViewAxis( int _viewAxis ) {
                    viewAxis = _viewAxis;
                }

                ///
                inline void setVerbose( bool _verbose ) {
                    verbose = _verbose;
                }

                void setPrerejectionD( bool _prerejectionD );
                void setPrerejectionG( bool _prerejectionG );
                void setPrerejectionL( bool _prerejectionL );
                void setPrerejectionA( bool _prerejectionA );

                core::Detection estimate();

            private:
                // Ransac variables
                pcl::PointCloud<PointT>::Ptr source;
                pcl::PointCloud<PointT>::Ptr target;
                core::Correspondence::VecPtr corr;
                size_t iterations = 10000;
                size_t sampleSize = 3;
                bool prerejection_d = false;
                bool prerejection_g = false;
                bool prerejection_l = false;
                bool prerejection_a = false;
                float prerejectionSimilarity = 0.9;
                float inlierThreshold = 5;
                float inlierFraction = 0.05;
                bool reestimatePose = true;
                bool fullEvaluation = false;
                bool occlusionReasoning = false;
                bool occlusionRemoval = false;
                int viewAxis = 0;
                bool verbose = false;

                // Feature matching
                float resolution = 1;
                float objectScale = 1;
                float far = -1;
                float radiusNormal = 5;
                float resolutionQuery = 5;
                float resolutionTarget = 5;
                float radiusFeature = 25;
                float cutoff = 50;
                std::string feature = "si";
        };
    }
}

#endif
