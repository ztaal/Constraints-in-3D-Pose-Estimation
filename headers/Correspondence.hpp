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

#ifndef COVIS_CORRESPONDENCE_H
#define COVIS_CORRESPONDENCE_H

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
         * @class correspondences
         * @brief Base class for computing correspondences
         *
         * This class is used to compute correspondences between a target and source mesh
         *
         * @author Martin Staal Steenberg
         */
        class correspondence {
            public:
                /**
                 * Constructor: set default parameters:
                 *   - resolution (@ref setResolution()): 1
                 *   - objectScale (@ref setObjectScale()): 1
                 *   - far (@ref setFar()): -1
                 *   - radiusNormal (@ref setRadiusNormal()): 5
                 *   - resolutionQuery (@ref setResolutionQuery()): 5
                 *   - resolutionTarget (@ref setResolutionTarget()): 5
                 *   - radiusFeature (@ref setRadiusFeature()): 25
                 *   - cutoff (@ref setCutoff()): 50
                 *   - feature (@ref setFeature()): si
                 *   - verbose (@ref setVerbose()): false
                 */
                correspondence() :
                    resolution(1),
                    objectScale(1),
                    far(-1),
                    radiusNormal(5),
                    resolutionQuery(5),
                    resolutionTarget(5),
                    radiusFeature(25),
                    cutoff(50),
                    feature("si"),
                    verbose(false) {}

                /// empty destructor
                ~correspondence() {};

                /**
                 * Set resolution
                 * @param resolution
                 */
                inline void setResolution(bool _resolution) {
                    resolution = _resolution;
                }

                /**
                 * Set object scale
                 * @param objectScale
                 */
                inline void setObjectScale(bool _objectScale) {
                    objectScale = _objectScale;
                }

                /**
                 * Set far
                 * @param far
                 */
                inline void setFar(bool _far) {
                    far = _far;
                }

                /**
                 * Set radiusNormal
                 * @param radiusNormal
                 */
                inline void setRadiusNormal(bool _radiusNormal) {
                    radiusNormal = _radiusNormal;
                }

                /**
                 * Set resolutionQuery
                 * @param resolutionQuery
                 */
                inline void setResolutionQuery(bool _resolutionQuery) {
                    resolutionQuery = _resolutionQuery;
                }

                /**
                 * Set resolutionTarget
                 * @param resolutionTarget
                 */
                inline void setResolutionTarget(bool _resolutionTarget) {
                    resolutionTarget = _resolutionTarget;
                }

                /**
                 * Set radiusFeature
                 * @param radiusFeature
                 */
                inline void setRadiusFeature(bool _radiusFeature) {
                    radiusFeature = _radiusFeature;
                }

                /**
                 * Set cutoff %
                 * @param cutoff
                 */
                inline void setCutoff(bool _cutoff) {
                    cutoff = _cutoff;
                }

                /**
                 * Set feature type
                 * @param feature
                 */
                inline void setFeature(std::string _feature) {
                    feature = _feature;
                }

                /**
                 * Set verbose flag for printing
                 * @param verbose flag
                 */
                inline void setVerbose( bool _verbose ) {
                    verbose = _verbose;
                }

                /** TODO FIX
                 * Set verbose flag for printing
                 * @param verbose flag
                 */
                covis::core::Correspondence::VecPtr compute( std::string query, std::string target );

            private:

                /// Resolution of scene and object clouds
                float resolution;

                /// Scale of the object
                float objectScale;

                /// Depth of which target points are considered
                float far;

                /// Normal estimation radius in mr
                float radiusNormal;

                /// Resolution of query features in mr
                float resolutionQuery;

                /// Resolution of target features in mr
                float resolutionTarget;

                /// Feature estimation radius
                float radiusFeature;

                /// Cutoff % for the best correspondences for RANSAC based on feature distance
                float cutoff;

                /// Feature used for feature matching
                std::string feature;

                /// Verbose flag
                bool verbose;
        };
    }
}

#endif
