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

#include "../headers/Benchmark.hpp"

using namespace covis::detect;

void Benchmark::loadData(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                        std::vector<util::DatasetLoader::ModelPtr> *sceneMesh,
                        std::vector<std::vector<Eigen::Matrix4f> > *poses)
{
    // Load dataset
    util::DatasetLoader dataset(
            this->rootPath,
            this->objDir,
            this->sceneDir,
            this->poseDir,
            this->objExt,
            this->sceneExt,
            this->poseExt,
            this->poseSep
    );

    // dataset.setRegexObject(po.getValue("object-regex"));
    // dataset.setRegexScene(po.getValue("scene-regex"));
    dataset.parse();

    if(this->verbose)
        COVIS_MSG(dataset);

    (*poses).resize(dataset.size());

    // Load object cloud, scene clouds and GT poses
    *objectMesh = dataset.getObjects();
    for(size_t i = 0; i < dataset.size(); ++i) {
        util::DatasetLoader::SceneEntry scene = dataset.at(i);
        (*sceneMesh).push_back(scene.scene);
        for(size_t j = 0; j < scene.poses.size(); ++j) {
            (*poses)[i].push_back( scene.poses[j] );
        }
    }
    COVIS_ASSERT(!objectMesh->empty() && !sceneMesh->empty());
    this->objectLabels = dataset.getObjectLabels();
}

void Benchmark::computeCorrespondence(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                                    std::vector<util::DatasetLoader::ModelPtr> *sceneMesh)
{
    std::vector<CloudT::Ptr> objectSurf, sceneSurf;

    // Surfaces and normals
    const bool resolutionInput = (this->resolution > 0.0f);
    if(!resolutionInput)
        this->resolution = 0;
    float diag = 0;
    for(size_t i = 0; i < objectMesh->size(); ++i) {
        if(!resolutionInput)
            this->resolution += detect::computeResolution((*objectMesh)[i]) * this->objectScale;
        diag += detect::computeDiagonal((*objectMesh)[i]) * this->objectScale;
        if(objectScale != 1)
            (*objectMesh)[i] = filter::scale((*objectMesh)[i], this->objectScale);
    }
    if(!resolutionInput)
        this->resolution /= float( objectMesh->size() );
    diag /= float(objectMesh->size());

    const float nrad =
            this->radiusNormal > 0.0 ?
            this->radiusNormal * this->resolution :
            2 * this->resolution;

    // Features and matching
    const float resQuery =
            this->resolutionQuery > 0.0 ?
            this->resolutionQuery * this->resolution :
            5 * this->resolution;
    const float resTarget =
            this->resolutionTarget > 0.0 ?
            this->resolutionTarget * this->resolution :
            5 * this->resolution;
    const float frad =
            this->radiusFeature > 0.0 ?
            this->radiusFeature * this->resolution :
            25 * this->resolution;
    const size_t cutoff = this->cutoff;
    COVIS_ASSERT(cutoff > 0 && cutoff <= 100);

    // Preprocess
    objectSurf.resize( objectMesh->size() );
    for(size_t i = 0; i < objectMesh->size(); ++i) {
        objectSurf[i] = filter::preprocess<PointT>((*objectMesh)[i], 1, true, this->far, this->resolution, nrad, false,
                                                  false, false);
        COVIS_ASSERT( !objectSurf[i]->empty() );
    }

    sceneSurf.resize(sceneMesh->size());
    for(size_t i = 0; i < sceneMesh->size(); ++i) {
        sceneSurf[i] = filter::preprocess<PointT>((*sceneMesh)[i], 1, true, this->far, this->resolution, nrad, false,
                                                  false, false);
        COVIS_ASSERT(!sceneSurf[i]->empty());
    }

    // Generate feature points
    this->objectCloud.resize(objectSurf.size());
    for(size_t i = 0; i < objectSurf.size(); ++i) {
        this->objectCloud[i] = filter::downsample<PointT>(objectSurf[i], resQuery);

        COVIS_ASSERT(!this->objectCloud[i]->empty());
    }

    this->sceneCloud.resize(sceneSurf.size());
    for(size_t i = 0; i < sceneSurf.size(); ++i) {
        this->sceneCloud[i] = filter::downsample<PointT>(sceneSurf[i], resTarget);

        COVIS_ASSERT(!this->sceneCloud[i]->empty());
    }

    // Compute features
    std::vector<feature::MatrixT> objectFeat, sceneFeat;
    objectFeat.resize(objectSurf.size());
    for(size_t i = 0; i < objectSurf.size(); ++i) {
        objectFeat[i] = feature::computeFeature<PointT>(feature, this->objectCloud[i], objectSurf[i], frad);
    }

    sceneFeat.resize(sceneSurf.size());
    for(size_t i = 0; i < sceneSurf.size(); ++i) {
        sceneFeat[i] = feature::computeFeature<PointT>(feature, this->sceneCloud[i], sceneSurf[i], frad);
    }

     // Match features
    this->correspondences.resize(sceneFeat.size());
    for (size_t i = 0; i < sceneFeat.size(); i++) {
        this->correspondences[i].resize(objectFeat.size());
        for (size_t j = 0; j < objectFeat.size(); j++) {
            this->correspondences[i][j] = detect::computeRatioMatches(objectFeat[j], sceneFeat[i]);

            // Sort correspondences and cutoff at <cutoff> %
            if(cutoff < 100) {
                covis::core::sort(*this->correspondences[i][j]);
                this->correspondences[i][j]->resize(this->correspondences[i][j]->size() * cutoff / 100);
            }
        }
    }
}

void Benchmark::initialize()
{
    if(this->verbose)
        printf("Loading data\n");
    std::vector<util::DatasetLoader::ModelPtr> objectMesh, sceneMesh;
    loadData( &objectMesh, &sceneMesh, &this->poses );
    computeCorrespondence( &objectMesh, &sceneMesh );
    this->seed = std::time(0);
}

void Benchmark::run( class ransac *instance, std::string funcName )
{
    // Call init if it has not been called before
    boost::call_once([this]{initialize();}, this->flagInit);

    // Seed all Benchmarks with same seed
    covis::core::randgen.seed( this->seed );

    Result result;

    // Benchmark
    {
        printf( "Benchmarking %s: \n", funcName.c_str() );
        std::vector<std::vector<covis::core::Detection> > d( sceneCloud.size() );
        std::vector<std::vector<double> > time( sceneCloud.size() );

        covis::core::ProgressDisplay pd( sceneCloud.size(), true );

        // Start timer
        covis::core::Timer t;

        // Run through scenes and estimate pose of each object
        for ( size_t i = 0; i < sceneCloud.size(); i++, ++pd ) {
            d[i].resize( objectCloud.size() );
            time[i].resize( objectCloud.size() );
            t.intermediate();
            for ( size_t j = 0; j < objectCloud.size(); j++ ) {
                instance->setSource( this->objectCloud[j] );
                instance->setTarget( this->sceneCloud[i] );
                instance->setCorrespondences( this->correspondences[i][j] );
                d[i][j] = instance->estimate();
                time[i][j] = t.intermediate();

                // if ( this->benchmark )
                result.prerejectionStats.push_back( instance->benchmark( this->poses[i][j] ) );

                if (this->verbose) {
                    COVIS_MSG( d[i][j].pose );
                    visu::showDetection<PointT>( this->objectCloud[j], this->sceneCloud[i], d[i][j].pose );
                }

            }
        }
        result.d = d;
        result.time = time;
        result.name = funcName;
        result.totalTime = t.seconds();
    }
    // Store results of the Benchmark
    this->results.push_back( result );
}

void Benchmark::printResults()
{
    // Sanity checks
    COVIS_ASSERT_MSG( this->results.size() > 0,
        "Run Benchmark() atleast once before calling printResults()." );

    // Header
    printf( "\n\n\n\033[1m%67s\033[m\n", "BENCHMARK RESULTS" );
    printf( "\033[1m%20s%15s%15s%10s(%%)%15s%15s%20s\033[m\n", "Function Name   ",
        "Total Time", "Avg Time", "Failed", "Avg RMSE", "Avg Penalty", "Avg InlierFrac" );

    // Data
    std::vector<int> objSuccessful( objectCloud.size() );
    std::vector<double> avgObjTime( objectCloud.size() );
    std::vector<double> avgObjRMSE( objectCloud.size() );
    std::vector<double> avgObjPenalty( objectCloud.size() );
    std::vector<double> avgObjInliers( objectCloud.size() );

    // Excecution speed and error
    for( auto &result : this->results ) {
        // Calculate averages
        double avgRMSE = 0, avgInliers = 0, avgPenalty = 0;
        int successful = 0;
        for ( unsigned int i = 0; i < objectCloud.size(); i++ ) {
            objSuccessful[i] = 0;
            double objTime = 0, objRMSE = 0, objInliers = 0, objPenalty = 0;
            for ( unsigned int j = 0; j < sceneCloud.size(); j++ ) {
                if ( result.d[j][i] ) {
                    objTime += result.time[j][i];
                    objRMSE += result.d[j][i].rmse;
                    objPenalty += result.d[j][i].penalty;
                    objInliers += result.d[j][i].inlierfrac;
                    objSuccessful[i]++;
                }
            }
            avgRMSE += objRMSE;
            avgPenalty += objPenalty;
            avgInliers += objInliers;
            successful += objSuccessful[i];
            avgObjTime[i] = objTime;
            avgObjRMSE[i] = objRMSE / objSuccessful[i];
            avgObjPenalty[i] = objPenalty / objSuccessful[i];
            avgObjInliers[i] = objInliers / objSuccessful[i];
        }
        int totalIterations = sceneCloud.size() * objectCloud.size();
        avgRMSE = avgRMSE / successful;
        avgInliers = avgInliers / successful;
        avgPenalty = avgPenalty / successful;
        double avgTime = result.totalTime / successful;
        double failed = totalIterations - successful;
        double failedPercent = (failed / totalIterations) * 100;

        // Print information
        printf("%s\n",std::string(115, '-').c_str());
        printf( " \033[1m%-20s%14.4f%15.4f%13.1f%15.5f%15.4f%20.4f\033[m\n",
            result.name.c_str(), result.totalTime, avgTime, failedPercent,
            avgRMSE, avgPenalty, avgInliers );

        // Print information about each object
        for ( unsigned int i = 0; i < objectCloud.size(); i++ ) {
            double objFailed = sceneCloud.size() - objSuccessful[i];
            double objFailedPercent = (objFailed / sceneCloud.size()) * 100;
            printf( "\033[3m%15s\033[m%20.4f%15.4f%13.1f%15.5f%15.4f%20.4f\n",
                objectLabels[i].c_str(), avgObjTime[i],
                avgObjTime[i] / objSuccessful[i], objFailedPercent,
                avgObjRMSE[i], avgObjPenalty[i], avgObjInliers[i] );
        }
    }
    printf("%s\n\n\n",std::string(115,'-').c_str());
}

void Benchmark::printPrerejectionResults()
{

    int numPrerejction = this->results[0].prerejectionStats[0].size();
    std::vector<int> tp( numPrerejction ), tn( numPrerejction ), fp( numPrerejction ), fn( numPrerejction );

    for( auto &result : this->results ) {
        for ( auto &prerejectionStat : result.prerejectionStats ) {
            for ( int k = 0; k < numPrerejction; k++ ) {
                tp[k] += prerejectionStat[k].tp;
                tn[k] += prerejectionStat[k].tn;
                fp[k] += prerejectionStat[k].fp;
                fn[k] += prerejectionStat[k].fn;
            }
        }
    }

    // Prerejection information
    printf( "\n\033[1m%67s\033[m\n", "PREREJECTION RESULTS" );
    for ( int i = 0; i < numPrerejction; i++ ) {
        printf( " \033[1m%-13s%d\033[m\n", "Prerejection ", i );
        printf("%s\n",std::string(115, '-').c_str());
        printf( " \033[1m%-20s%20s%20s\033[m\n",
            "", "Accecpt", "Reject" );
        printf( " \033[1m%-20s%20d%20d\033[m\n",
            "Positive outcome", tp[i], fp[i] );
        printf( " \033[1m%-20s%20d%20d\033[m\n",
            "Negative outcome", fn[i], tn[i] );
        printf( "%s\n\n\n",std::string(115, '-').c_str() );
    }
}
