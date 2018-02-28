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

#ifndef COVIS_BENCHMARK_H
#define COVIS_BENCHMARK_H

// Covis
#include <covis/covis.h>

// Timer
#include "../headers/Timer.hpp"

// Ransac
#include "../headers/Ransac.hpp"

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Point and feature types
typedef pcl::PointCloud<PointT> CloudT;

namespace covis {
    namespace detect {
        /**
         * @ingroup detect
         * @class Benchmark
         * @brief Base class for benchmarking ransac
         *
         * This class is used to benchmark ransac on an entire data set and print information such as:
         * Total time, avg time, failed percentage, avg RMSE, avg penalty and avg inlier fraction
         *
         * @author Martin Staal Steenberg
         */
        class Benchmark {
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
                Benchmark() :
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
                virtual ~Benchmark() {}

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
                 * Set directory contaning the ground truth poses
                 * @param pose directory
                 */
                inline void setPoseDir( std::string _poseDir ) {
                    poseDir = _poseDir;
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
                 * Set extenstion of the pose files
                 * @param file extenstion
                 */
                inline void setPoseExt( std::string _poseExt ) {
                    poseExt = _poseExt;
                }

                /**
                 * Set pose seperator
                 * @param pose seperator
                 */
                inline void setPoseSep( std::string _poseSep ) {
                    poseSep = _poseSep;
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
                 * Set benchmarkPrerejection flag
                 * @param benchmarkPrerejection
                 */
                inline void setBenchmarkPrerejection(bool _benchmarkPrerejection) {
                    benchmarkPrerejection = _benchmarkPrerejection;
                }

                /**
                 * Set the correction flag
                 * @param correction
                 */
                inline void setCorrection( bool _correction ) {
                    correction = _correction;
                }

                /**
                 * Generates a new seed for the benchmark
                 */
                inline void generateNewSeed() {
                    seed = std::time(0);
                }

                /**
                 * Benchmarks a function on the specified data set
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
                 * Print results of the prerejection benchmarks
                 * call @ref run() before calling this
                 */
                void printPrerejectionResults();

                /**
                 * Clear results of the benchmarks
                 */
                inline void clearResults() {
                    results.clear();
                };

            private:

                /// struct contaning all benchmark results
                struct Result
                {
                    std::string name;
                    std::vector<std::vector<double> > avgDistance;
                    std::vector<std::vector<double> > medianDistance;
                    std::vector<std::vector<covis::core::Detection> > d;
                    std::vector<std::vector<binaryClassification> > prerejectionStats;
                    std::vector<std::vector<double> > time;
                };

                /// Generated seed used in ransac
                size_t seed;

                /// root path
                std::string rootPath;

                /// object directory
                std::string objDir;

                /// scene directory
                std::string sceneDir;

                /// pose directory
                std::string poseDir;

                /// object extensions
                std::string objExt;

                /// scene extensions
                std::string sceneExt;

                /// pose extensions
                std::string poseExt;

                /// pose seperator
                std::string poseSep;

                /// init flag, used to run @ref initBenchmark() once when @ref benchmark() is first called
                boost::once_flag flagInit = BOOST_ONCE_INIT;

                /// object cloud
                std::vector<CloudT::Ptr> objectCloud;

                /// scene cloud
                std::vector<CloudT::Ptr> sceneCloud;

                /// ground truth poses
                std::vector<std::vector<Eigen::Matrix4f> > poses;

                /// mask designating which of the objects are present in the scene
                std::vector<std::vector<bool> > objectMask;

                /// correspondences
                std::vector< std::vector<covis::core::Correspondence::VecPtr> > correspondences;

                /// results of the benchmarks
                std::vector<Result> results;

                /// object labels
                std::vector<std::string> objectLabels;

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
                size_t cutoff;

                /// Feature used for feature matching
                std::string feature;

                /// Verbose flag
                bool verbose;

                /// Prerejection benchmark flag
                bool benchmarkPrerejection;

                /// Correction method flag
                bool correction;

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
                * Compute correspondences of each object with each scene
                * @param pointer to objectMesh
                * @param pointer to sceneMesh
                */
                void computeCorrespondence(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                                            std::vector<util::DatasetLoader::ModelPtr> *sceneMesh);

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
