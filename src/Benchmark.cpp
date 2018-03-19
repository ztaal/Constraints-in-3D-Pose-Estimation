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

    dataset.parse();

    if(this->verbose)
        COVIS_MSG(dataset);

    (*poses).resize(dataset.size());

    // Load object cloud, scene clouds and GT poses
    *objectMesh = dataset.getObjects();
    for(size_t i = 0; i < dataset.size(); ++i) {
        util::DatasetLoader::SceneEntry scene = dataset.at(i);
        (*sceneMesh).push_back(scene.scene);
        this->objectMask.push_back(scene.objectMask);
        for( auto &pose : scene.poses ) {
            (*poses)[i].push_back( pose );
        }
    }
    COVIS_ASSERT( !objectMesh->empty() && !sceneMesh->empty() && !poses->empty() );
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
    for(size_t i = 0; i < objectMesh->size(); ++i) {
        if(!resolutionInput)
            this->resolution += detect::computeResolution((*objectMesh)[i]) * this->objectScale;
        if(objectScale != 1)
            (*objectMesh)[i] = filter::scale((*objectMesh)[i], this->objectScale);
    }
    if(!resolutionInput)
        this->resolution /= float( objectMesh->size() );

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
    COVIS_ASSERT(this->cutoff > 0 && this->cutoff <= 100);

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
        objectFeat[i] = feature::computeFeature<PointT>(this->feature, this->objectCloud[i], objectSurf[i], frad);
    }

    sceneFeat.resize(sceneSurf.size());
    for(size_t i = 0; i < sceneSurf.size(); ++i) {
        sceneFeat[i] = feature::computeFeature<PointT>(this->feature, this->sceneCloud[i], sceneSurf[i], frad);
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
                this->correspondences[i][j]->resize(this->correspondences[i][j]->size() * this->cutoff / 100);
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
    generateNewSeed();
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
        std::vector<std::vector<double> > time( this->sceneCloud.size() );
        std::vector<std::vector<double> > avgDistance( this->sceneCloud.size() );
        std::vector<std::vector<double> > medianDistance( this->sceneCloud.size() );
        std::vector<std::vector<covis::core::Detection> > d( this->sceneCloud.size() );

        covis::core::ProgressDisplay pd( this->sceneCloud.size(), true );

        // Start timer
        covis::core::Timer t;

        // Run through scenes and estimate pose of each object
        for ( size_t i = 0; i < this->sceneCloud.size(); i++, ++pd ) {
            d[i].resize( this->objectCloud.size() );
            time[i].resize( this->objectCloud.size() );
            avgDistance[i].resize( this->objectCloud.size() );
            medianDistance[i].resize( this->objectCloud.size() );
            t.intermediate();

            for ( size_t j = 0; j < this->objectCloud.size(); j++ ) {
                if ( !objectMask[i][j] ) // skip objects thats are not in the scene
                    continue;

                instance->setSource( this->objectCloud[j] );
                instance->setTarget( this->sceneCloud[i] );
                instance->setCorrespondences( this->correspondences[i][j] );
                d[i][j] = instance->estimate();
                time[i][j] = t.intermediate();

                if ( this->benchmarkPrerejection )
                    result.prerejectionStats.push_back( instance->benchmark( this->poses[i][j] ) );

                if ( this->correction )
                    instance->estimate_correction( this->poses[i][j] );
                    // instance->estimate_correction2( this->poses[i][j] );

                if (d[i][j]) {
                    // Calculate distance from GT
                    CloudT gtCloud = *this->objectCloud[j];
                    CloudT poseCloud = *this->objectCloud[j];

                    covis::core::transform( poseCloud, d[i][j].pose );
                    covis::core::transform( gtCloud, this->poses[i][j] );

                    std::vector<double> distance;
                    for ( auto corr : *this->correspondences[i][j] )
                    distance.push_back( pcl::euclideanDistance(poseCloud[corr.query], gtCloud[corr.query]) );

                    medianDistance[i][j] = this->median( distance );

                    for ( auto &n : distance )
                    avgDistance[i][j] += n;
                    avgDistance[i][j] = avgDistance[i][j] / distance.size();

                    if (this->verbose) {
                        COVIS_MSG( d[i][j].pose );
                        visu::showDetection<PointT>( this->objectCloud[j], this->sceneCloud[i], d[i][j].pose );
                    }
                }
            }
        }
        result.d = d;
        result.time = time;
        result.name = funcName;
        result.avgDistance = avgDistance;
        result.medianDistance = medianDistance;
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
    printf( "\n\n\n\033[1m%90s\033[m\n", "BENCHMARK RESULTS" );
    printf( "\033[1m%20s%15s%15s%10s(%%)%17s%20s%15s%15s%20s%23s\033[m\n", "Function Name   ",
        "Total Time", "Avg Time", "Failed", "Avg Corr Dist", "Median Corr Dist",
        "Avg RMSE", "Avg Penalty", "Avg InlierFrac", "Stddev InlierFrac" );

    // Data
    std::vector<int> objAttempt( this->objectCloud.size() );
    std::vector<int> objSuccessful( this->objectCloud.size() );
    std::vector<double> avgObjTime( this->objectCloud.size() );
    std::vector<double> avgObjRMSE( this->objectCloud.size() );
    std::vector<double> avgObjPenalty( this->objectCloud.size() );
    std::vector<double> avgObjInliers( this->objectCloud.size() );
    std::vector<double> avgObjMedianDist( this->objectCloud.size() );
    std::vector<double> avgObjDist( this->objectCloud.size() );
    std::vector<double> stddevObjInliers( this->objectCloud.size() );

    // Excecution speed and error
    for( auto &result : this->results ) {
        // Calculate averages
        int successful = 0, totalIterations = 0;
        double avgRMSE = 0, avgInliers = 0, avgPenalty = 0;
        double avgDist = 0, avgMedianDist = 0, totalTime = 0;

        for ( unsigned int i = 0; i < this->objectCloud.size(); i++ ) {
            objAttempt[i] = 0;
            objSuccessful[i] = 0;
            double objRMSE = 0, objInliers = 0, objPenalty = 0;
            double objDist = 0, objMedianDist = 0;
            for ( unsigned int j = 0; j < this->sceneCloud.size(); j++ ) {
                totalIterations += objectMask[j][i];
                objAttempt[i] += objectMask[j][i];
                if ( result.d[j][i] && objectMask[j][i] ) {
                    avgObjTime[i] += result.time[j][i];
                    totalTime += result.time[j][i];
                    objRMSE += result.d[j][i].rmse;
                    objPenalty += result.d[j][i].penalty;
                    objInliers += result.d[j][i].inlierfrac;
                    objDist += result.avgDistance[j][i];
                    objMedianDist += result.medianDistance[j][i];
                    objSuccessful[i]++;
                }
            }
            avgRMSE += objRMSE;
            avgPenalty += objPenalty;
            avgInliers += objInliers;
            avgDist += objDist;
            avgMedianDist += objMedianDist;
            successful += objSuccessful[i];
            avgObjRMSE[i] = objRMSE / objSuccessful[i];
            avgObjPenalty[i] = objPenalty / objSuccessful[i];
            avgObjInliers[i] = objInliers / objSuccessful[i];
            avgObjDist[i] = objDist / objSuccessful[i];
            avgObjMedianDist[i] = objMedianDist / objSuccessful[i];
        }

        avgRMSE = avgRMSE / successful;
        avgInliers = avgInliers / successful;
        avgPenalty = avgPenalty / successful;
        avgDist = avgDist / successful;
        avgMedianDist = avgMedianDist / successful;
        double avgTime = totalTime / successful;
        double failed = totalIterations - successful;
        double failedPercent = (failed / totalIterations) * 100;

        // Calculate standard deviation
        double dist = 0;
        for ( unsigned int i = 0; i < this->objectCloud.size(); i++ ) {
            double objDist = 0;
            for ( unsigned int j = 0; j < this->sceneCloud.size(); j++ ) {
                if ( result.d[j][i] && objectMask[j][i] ) {
                    objDist += pow(result.d[j][i].inlierfrac - avgObjInliers[i], 2);
                    dist += pow(result.d[j][i].inlierfrac - avgInliers, 2);
                }
            }
            stddevObjInliers[i] = sqrt(objDist / objSuccessful[i]);
        }
        double stddevInliers = sqrt(dist / successful);

        // Print information
        printf("%s\n",std::string(160, '-').c_str());
        printf( " \033[1m%-20s%14.4f%15.4f%13.1f%17f%20f%15.5f%15.4f%20.4f%23.4f\033[m\n",
            result.name.c_str(), totalTime, avgTime, failedPercent, avgDist,
            avgMedianDist, avgRMSE, avgPenalty, avgInliers, stddevInliers );

        // Print information about each object
        for ( unsigned int i = 0; i < this->objectCloud.size(); i++ ) {
            double objFailed = objAttempt[i] - objSuccessful[i];
            double objFailedPercent = (objFailed / objAttempt[i]) * 100;
            printf( "\033[3m%15s\033[m%20.4f%15.4f%13.1f%17f%20f%15.5f%15.4f%20.4f%23.4f\n",
                objectLabels[i].c_str(), avgObjTime[i],
                avgObjTime[i] / objSuccessful[i], objFailedPercent, avgObjDist[i], avgObjMedianDist[i],
                avgObjRMSE[i], avgObjPenalty[i], avgObjInliers[i], stddevObjInliers[i] );
        }
    }
    printf("%s\n\n\n",std::string(160,'-').c_str());
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

void Benchmark::saveResults( std::string path )
{
    // Sanity checks
    COVIS_ASSERT_MSG( this->results.size() > 0,
        "Run Benchmark() atleast once before calling saveResults( std::string )." );

    for( auto &result : this->results ) {
        ofstream file;
        file.open( path + result.name + ".txt" );
        file << result.name << "\n";
        file << "Object,Time,Failed,RMSE,Penalty,InlierFrac,Avg Dist,Median Dist\n";

        for ( unsigned int i = 0; i < this->objectCloud.size(); i++ ) {
            for ( unsigned int j = 0; j < this->sceneCloud.size(); j++ ) {
                if ( objectMask[j][i] ) {
                    if ( result.d[j][i] ) {
                        file << objectLabels[i] << ",";
                        file << result.time[j][i] << ",";
                        file << "0,";
                        file << result.d[j][i].rmse << ",";
                        file << result.d[j][i].penalty << ",";
                        file << result.d[j][i].inlierfrac << ",";
                        file << result.avgDistance[j][i] << ",";
                        file << result.medianDistance[j][i] << "\n";
                    } else {
                        file << "-,1,-,-,-,-,-\n";
                    }
                }
            }
        }
        file.close();
    }
}
