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

#ifndef COVIS_BENCHMARK_TEJANI_H
#define COVIS_BENCHMARK_TEJANI_H

// Covis
#include <covis/covis.h>

// Timer
#include "../headers/Timer.hpp"

// Pose Prior
#include "../headers/PosePrior.hpp"

// Ransac
#include "../headers/Ransac.hpp"

// YML loader
#include "../headers/yml_loader.hpp"

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Point and feature types
typedef pcl::PointCloud<PointT> CloudT;

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class Benchmark Tejani
         * @brief Base class for benchmarking the tejani dataset
         *
         * This class is used to benchmark the tejani dataset
         *
         * @author Martin Staal Steenberg
         */
        class Benchmark_Tejani {
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
                Benchmark_Tejani() :
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

                /// Empty destructor
                virtual ~Benchmark_Tejani() {}

                /**
                 * Set root path of data set
                 * @param root path
                 */
                inline void setRootPath( std::string _rootPath ) {
                    rootPath = _rootPath;
                }

                /**
                 * Set directory contaning the objects
                 * @param object directory
                 */
                inline void setObjectDir( std::string _objDir ) {
                    objDir = _objDir;
                }

                /**
                 * Set directory contaning the scenes
                 * @param scene directory
                 */
                inline void setSceneDir( std::string _sceneDir ) {
                    sceneDir = _sceneDir;
                }

                /**
                 * Set name of file containing the ground truth poses
                 * @param pose file
                 */
                inline void setPoseFile( std::string _poseFile ) {
                    poseFile = _poseFile;
                }

                /**
                 * Set name of file containing the benchmark indices
                 * @param benchmark file
                 */
                inline void setBenchmarkFile( std::string _benchmarkFile ) {
                    benchmarkFile = _benchmarkFile;
                }

                /**
                 * Set extenstion of the object files
                 * @param file extenstion
                 */
                inline void setObjExt( std::string _objExt ) {
                    objExt = _objExt;
                }

                 /**
                 * Set extenstion of the scene files
                 * @param file extenstion
                 */
                inline void setSceneExt( std::string _sceneExt ) {
                    sceneExt = _sceneExt;
                }

                /**
                 * Set resolution
                 * @param resolution
                 */
                inline void setResolution(float _resolution) {
                    resolution = _resolution;
                }

                /**
                 * Set object scale
                 * @param objectScale
                 */
                inline void setObjectScale(float _objectScale) {
                    objectScale = _objectScale;
                }

                /**
                 * Set far
                 * @param far
                 */
                inline void setFar(float _far) {
                    far = _far;
                }

                /**
                 * Set radiusNormal
                 * @param radiusNormal
                 */
                inline void setRadiusNormal(float _radiusNormal) {
                    radiusNormal = _radiusNormal;
                }

                /**
                 * Set resolutionQuery
                 * @param resolutionQuery
                 */
                inline void setResolutionQuery(float _resolutionQuery) {
                    resolutionQuery = _resolutionQuery;
                }

                /**
                 * Set resolutionTarget
                 * @param resolutionTarget
                 */
                inline void setResolutionTarget(float _resolutionTarget) {
                    resolutionTarget = _resolutionTarget;
                }

                /**
                 * Set radiusFeature
                 * @param radiusFeature
                 */
                inline void setRadiusFeature(float _radiusFeature) {
                    radiusFeature = _radiusFeature;
                }

                /**
                 * Set cutoff %
                 * @param cutoff
                 */
                inline void setCutoff(size_t _cutoff) {
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
                 * @param verbose verbose flag
                 */
                inline void setVerbose(bool _verbose) {
                    verbose = _verbose;
                }

                /**
                 * Benchmarks posePrior on the specified data set
                 * Running multiple instance of benchmark will store each individual benchmark
                 * To see the result of the benchmark call @ref @printResults()
                 * @param posePrior instance, benchmark name
                 */
                void run( class posePrior *instance, std::string _funcName );

                /**
                 * Benchmarks ransac on the specified data set
                 * Running multiple instance of benchmark will store each individual benchmark
                 * To see the result of the benchmark call @ref @printResults()
                 * @param ransac instance, benchmark name
                 */
                void run( class ransac *instance, std::string _funcName );

                /**
                 * Print results of the benchmarks
                 * call @ref run() before calling this
                 */
                void printResults();

                /**
                 * Saves results of the benchmarks to .txt files
                 * call @ref run() before calling this
                 * @param save path
                 */
                void saveResults( std::string _path );

                /**
                 * Saves poses of the benchmarks to .yml files
                 * call @ref run() before calling this
                 * @param save path
                 */
                void savePoses( std::string _path );

                /**
                 * Clear results of the benchmarks
                 */
                inline void clearResults() {
                    results.clear();
                };

            private:

                /// Struct contaning all benchmark results
                struct Result
                {
                    std::string name;
                    std::vector<double> avgDistance;
                    std::vector<double> medianDistance;
                    std::vector<covis::core::Detection> d;
                    std::vector<double> time;
                    std::vector<double> translationDist;
                    std::vector<double> angle;
                    std::vector<bool> failed;
                    std::vector<Eigen::Matrix4f> poses;
                    std::vector<std::string> sceneLabels;
                    std::string objectLabel;
                };

                /// Root path
                std::string rootPath;

                /// Object directory
                std::string objDir;

                /// Scene directory
                std::string sceneDir;

                /// Pose path
                std::string poseFile;

                /// Benchmark path
                std::string benchmarkFile;

                /// Object extensions
                std::string objExt;

                /// Scene extensions
                std::string sceneExt;

                /// Init flag, used to run @ref initBenchmark() once when @ref benchmark() is first called
                boost::once_flag flagInit = BOOST_ONCE_INIT;

                /// Object features
                feature::MatrixT objectFeat;

                /// Object cloud
                CloudT::Ptr objectCloud;

                /// Distance to the cloest point from centroid on the object
                double centroidDist;

                /// Scene meshes
                std::vector<covis::util::DatasetLoader::ModelPtr> sceneMesh;

                /// Scene cloud
                CloudT::Ptr sceneCloud;

                /// Ground truth poses
                std::vector<std::vector<Eigen::Matrix4f> > poses;

                /// Results of the benchmarks
                std::vector<Result> results;

                /// Scene labels
                std::vector<std::string> sceneLabels;

                /// Object labels
                std::string objectLabel;

                /// Object index
                int objectIndex;

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

                // Feature radius
                float frad;

                /// Cutoff % for the best correspondences for RANSAC based on feature distance
                size_t cutoff;

                /// Feature used for feature matching
                std::string feature;

                /// Verbose flag
                bool verbose;

                /**
                 * Initialized benchmark by loadning the data set and computing correspondences
                 */
                void initialize();

                /**
                 * Load data set with grount truth poses
                 * @param pointer to objectMesh
                 * @param pointer to sceneMesh
                 * @param pointer to poses
                 */
                void loadData(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                                std::vector<util::DatasetLoader::ModelPtr> *sceneMesh,
                                std::vector<std::vector<Eigen::Matrix4f> > *poses);

                /**
                 * Compute features of the object
                 * @param pointer to objectMesh
                 */
                void computeObjFeat(util::DatasetLoader::ModelPtr *objectMesh);

                /**
                * Compute correspondences of a scene
                * @param pointer to sceneMesh
                * @return correspondences
                */
                covis::core::Correspondence::VecPtr computeCorrespondence(util::DatasetLoader::ModelPtr *sceneMesh);

                /**
                * Compute the euclidean distance between the translation of two Matrix4f
                * @param matrix 1
                * @param matrix 2
                * @return distance
                */
                inline double
                norm( Eigen::Matrix4f p1, Eigen::Matrix4f p2 )
                {
                    PointT gtP;
                    gtP.x = p1(0,3);
                    gtP.y = p1(1,3);
                    gtP.z = p1(2,3);
                    PointT poseP;
                    poseP.x = p2(0,3);
                    poseP.y = p2(1,3);
                    poseP.z = p2(2,3);
                    return pcl::euclideanDistance(gtP, poseP);
                }

                /**
                * Compute median of vector
                * @param vector
                */
                inline double
                median( std::vector<double> v )
                {
                    size_t size = v.size();
                    std::sort(v.begin(), v.end());

                    if (size % 2 == 0)
                        return (v[size / 2 - 1] + v[size / 2]) / 2;

                    return v[size / 2];
                }
        };
    }
}

#endif
