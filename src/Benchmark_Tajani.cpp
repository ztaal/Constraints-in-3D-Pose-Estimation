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

#include "../headers/Benchmark_Tejani.hpp"

using namespace covis::detect;

void Benchmark_Tejani::loadData(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                                std::vector<util::DatasetLoader::ModelPtr> *sceneMesh,
                                std::vector<std::vector<Eigen::Matrix4f> > *poses)
{
    // Load dataset
    util::DatasetLoader dataset(
            this->rootPath,
            this->objDir,
            this->sceneDir,
            "",
            this->objExt,
            this->sceneExt,
            "",
            ""
    );
    dataset.parse();

    if(this->verbose)
        COVIS_MSG(dataset);

    // Load object cloud, scene clouds and GT poses
    *objectMesh = dataset.getObjects();
    for(size_t i = 0; i < dataset.size(); ++i) {
        util::DatasetLoader::SceneEntry scene = dataset.at(i);
        (*sceneMesh).push_back(scene.scene);
    }
    util::yml_loader yml( this->rootPath + posePath );
    yml.load( poses );
    COVIS_ASSERT( !objectMesh->empty() && !sceneMesh->empty() && !poses->empty() );
    this->objectLabels = dataset.getObjectLabels();
}

void Benchmark_Tejani::computeObjFeat(util::DatasetLoader::ModelPtr *objectMesh)
{

    // Surfaces and normals
    const bool resolutionInput = (this->resolution > 0.0f);
    if(!resolutionInput)
        this->resolution = detect::computeResolution(*objectMesh) * this->objectScale;
    if(objectScale != 1)
        *objectMesh = filter::scale(*objectMesh, this->objectScale);

    const float nrad =
            this->radiusNormal > 0.0 ?
            this->radiusNormal * this->resolution :
            2 * this->resolution;

    // Features and matching
    const float resQuery =
            this->resolutionQuery > 0.0 ?
            this->resolutionQuery * this->resolution :
            5 * this->resolution;
    const float frad =
            this->radiusFeature > 0.0 ?
            this->radiusFeature * this->resolution :
            25 * this->resolution;

    // Preprocess
    CloudT::Ptr objectSurf = filter::preprocess<PointT>(*objectMesh, 1, true, this->far, this->resolution,
                                            nrad, false, false, false);
    COVIS_ASSERT(!objectSurf->empty());

    // Generate feature points
    this->objectCloud = filter::downsample<PointT>(objectSurf, resQuery);
    COVIS_ASSERT(!this->objectCloud->empty());

    // Compute features
    this->objectFeat = feature::computeFeature<PointT>(this->feature, this->objectCloud, objectSurf, frad);
}

covis::core::Correspondence::VecPtr Benchmark_Tejani::computeCorrespondence(util::DatasetLoader::ModelPtr *sceneMesh)
{
    // Surfaces and normals
    const float nrad =
            this->radiusNormal > 0.0 ?
            this->radiusNormal * this->resolution :
            2 * this->resolution;

    // Features and matching
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
    CloudT::Ptr sceneSurf = filter::preprocess<PointT>(*sceneMesh, 1, true, this->far,  this->resolution,
                                            nrad, false, false, false);
    COVIS_ASSERT(!sceneSurf->empty());

    // Generate feature points
    this->sceneCloud = filter::downsample<PointT>(sceneSurf, resTarget);
    COVIS_ASSERT(!this->sceneCloud->empty());

    // Compute features
    feature::MatrixT sceneFeat = feature::computeFeature<PointT>(this->feature, this->sceneCloud, sceneSurf, frad);

    // Match features
    covis::core::Correspondence::VecPtr correspondences = detect::computeRatioMatches(this->objectFeat, sceneFeat);

    // Sort correspondences and cutoff at <cutoff> %
    if(this->cutoff < 100) {
        covis::core::sort(*correspondences);
        correspondences->resize(correspondences->size() * this->cutoff / 100);
    }
    return correspondences;
}

void Benchmark_Tejani::initialize()
{
    if(this->verbose)
        printf("Loading data\n");
    std::vector<covis::util::DatasetLoader::ModelPtr> objectMesh;
    loadData( &objectMesh, &this->sceneMesh, &this->poses );
    computeObjFeat( &objectMesh[0] );
}

void Benchmark_Tejani::run( class posePrior *instance, std::string funcName )
{
    // Call init if it has not been called before
    boost::call_once([this]{initialize();}, this->flagInit);

    // Instantiate result struct
    Result result;

    // Benchmark
    {
        printf( "Benchmarking %s: \n", funcName.c_str() );
        std::vector<double> time( this->sceneMesh.size() );
        std::vector<double> avgDistance( this->sceneMesh.size() );
        std::vector<double> medianDistance( this->sceneMesh.size() );
        std::vector<double> translationDist( this->sceneMesh.size() );
        std::vector<double> angle( this->sceneMesh.size() );
        std::vector<bool> failed( this->sceneMesh.size() );
        std::vector<covis::core::Detection> d( this->sceneMesh.size() );

        covis::core::ProgressDisplay pd( this->sceneMesh.size(), true );
        instance->setSource( this->objectCloud );

        // Start timer
        covis::core::Timer t;

        // Run through scenes and estimate pose of each object
        for ( size_t i = 0; i < this->sceneMesh.size(); i++, ++pd ) {
            covis::core::Correspondence::VecPtr correspondence = computeCorrespondence(&this->sceneMesh[i]);

            instance->setTarget( this->sceneCloud );
            instance->setCorrespondences( correspondence );

            // Run pose estimation
            t.intermediate();
            d[i] = instance->estimate();
            time[i] = t.intermediate();

            if (d[i]) {
                // Calculate distance from GT
                CloudT gtCloud = *this->objectCloud;
                CloudT poseCloud = *this->objectCloud;

                // Find gt pose closest to estimated pose
                int poseIndex = 0;
                double shortestDist = std::numeric_limits<double>::max();
                for ( size_t j = 0; j < this->poses[i].size(); j++ ) {
                    // Find distance between translation
                    double dist = norm( this->poses[i][j], d[i].pose );
                    if (dist < shortestDist) {
                        shortestDist = dist;
                        poseIndex = j;
                    }
                }
                covis::core::transform( poseCloud, d[i].pose );
                covis::core::transform( gtCloud, this->poses[i][poseIndex] );

                std::vector<double> distance;
                for ( auto corr : *correspondence )
                    distance.push_back( pcl::euclideanDistance(poseCloud[corr.query], gtCloud[corr.query]) );

                medianDistance[i] = this->median( distance );

                for ( auto &n : distance )
                    avgDistance[i] += n;
                avgDistance[i] = avgDistance[i] / distance.size();

                // Find distance between translations
                translationDist[i] = norm( this->poses[i][poseIndex], d[i].pose );

                // Find angle between z-axis
                Eigen::Vector3f poseTranslation = d[i].pose.block<3,1>(0, 2);
                Eigen::Vector3f gtTranslation = this->poses[i][poseIndex].block<3,1>(0,2);
                angle[i] = atan2( (gtTranslation.cross(poseTranslation)).norm(), gtTranslation.dot(poseTranslation) );

                // Check if pose is good or bad
                if ( translationDist[i] > 30 || angle[i] > 0.15) {
                    failed[i] = true;
                } else {
                    failed[i] = false;
                }

                // if ( this->verbose ) {
                if ( translationDist[i] > 30 || angle[i] > 0.15 || this->verbose ) { // 0.2
                // if ( translationDist[i] > 50 || angle[i] > 0.1 || this->verbose ) {
                    std::cout << "\nScene " << i << "\n";
                    std::cout << "Distance: " << translationDist[i] << '\n';
                    std::cout << "Angle: " << angle[i] << '\n';
                    COVIS_MSG( d[i].pose );
                    visu::showDetection<PointT>( this->objectCloud, this->sceneCloud, d[i].pose );
                }
            } else {
                std::cout << "\nScene " << i << " Failed!\n";
            }
        }
        result.d = d;
        result.time = time;
        result.name = funcName;
        result.avgDistance = avgDistance;
        result.medianDistance = medianDistance;
        result.translationDist = translationDist;
        result.angle = angle;
        result.failed = failed;
    }
    // Store results of the Benchmark
    this->results.push_back( result );
}

void Benchmark_Tejani::printResults()
{
    // Sanity checks
    COVIS_ASSERT_MSG( this->results.size() > 0,
        "Run Benchmark() atleast once before calling printResults()." );

    // Header
    printf( "\n\n\n\033[1m%105s\033[m\n", "BENCHMARK RESULTS" );
    printf( "\033[1m%20s%15s%13s%10s(%%)%17s%20s%15s%14s%14s%15s%18s%20s\033[m\n", "Function Name   ",
        "Total Time", "Avg Time", "Failed", "Avg Corr Dist", "Median Corr Dist", "Avg T Dist",
        "Avg T Angle", "Avg RMSE", "Avg Penalty", "Avg InlierFrac", "Stddev InlierFrac" );

    // Excecution speed and error
    for( auto &result : this->results ) {
        // Calculate averages
        int successful = 0;
        double avgRMSE = 0, avgInliers = 0, avgPenalty = 0, avgTime = 0, avgDist = 0;
        double avgMedianDist = 0, avgTransDist = 0, avgAngle = 0, totalTime = 0;
        for ( unsigned int i = 0; i < this->sceneMesh.size(); i++ ) {
            totalTime += result.time[i];
            if ( result.d[i] && !result.failed[i] ) {
                avgRMSE += result.d[i].rmse;
                avgInliers += result.d[i].inlierfrac;
                avgPenalty += result.d[i].penalty;
                avgTime += result.time[i];
                avgDist += result.avgDistance[i];
                avgMedianDist += result.medianDistance[i];
                avgTransDist += result.translationDist[i];
                avgAngle += result.angle[i];
                successful++;
            }
        }

        avgRMSE = avgRMSE / successful;
        avgInliers = avgInliers / successful;
        avgPenalty = avgPenalty / successful;
        avgTime = avgTime / successful;
        avgDist = avgDist / successful;
        avgMedianDist = avgMedianDist / successful;
        avgTransDist = avgTransDist / successful;
        avgAngle = avgAngle / successful;
        double failed = this->sceneMesh.size() - successful;
        double failedPercent = (failed / this->sceneMesh.size()) * 100;

        // Calculate standard deviation
        double dist = 0;
        for ( unsigned int i = 0; i < this->sceneMesh.size(); i++ ) {
            if ( result.d[i] && !result.failed[i] ) {
                dist += pow(result.d[i].inlierfrac - avgInliers, 2);
            }
        }
        double stddevInliers = sqrt(dist / successful);

        // Print information
        printf("%s\n",std::string(195, '-').c_str());
        printf( " \033[1m%-20s%14.4f%13.4f%13.1f%17.4f%20.4f%15.4f%14.4f%14.5f%15.4f%18.4f%20.4f\033[m\n",
            result.name.c_str(), totalTime, avgTime, failedPercent, avgDist, avgMedianDist,
            avgTransDist, avgAngle, avgRMSE, avgPenalty, avgInliers, stddevInliers );
    }
    printf("%s\n\n\n",std::string(195,'-').c_str());
}

void Benchmark_Tejani::saveResults( std::string path ) // TODO Add missing variables
{
    // Sanity checks
    COVIS_ASSERT_MSG( this->results.size() > 0,
        "Run Benchmark() atleast once before calling saveResults( std::string )." );

    for( auto &result : this->results ) {
        ofstream file;
        file.open( path + result.name + ".txt" );
        file << result.name << "\n";
        file << "Object,Time,Failed,RMSE,Penalty,InlierFrac,Avg Dist,Median Dist\n";

        for ( unsigned int i = 0; i < this->sceneMesh.size(); i++ ) {
            if ( result.d[i] && !result.failed[i] ) {
                file << objectLabels[i] << ",";
                file << result.time[i] << ",";
                file << "0,";
                file << result.d[i].rmse << ",";
                file << result.d[i].penalty << ",";
                file << result.d[i].inlierfrac << ",";
                file << result.avgDistance[i] << ",";
                file << result.medianDistance[i] << "\n";
            } else {
                file << "-,1,-,-,-,-,-\n";
            }
        }
        file.close();
    }
}
