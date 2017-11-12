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

#include "../headers/Ransac.hpp"

using namespace covis::detect;

covis::core::Detection ransac::estimate()
{
    // detect::PointSearch<PointT>::Ptr _search;
    covis::core::Detection::Vec _allDetections;

    // Sanity checks
    COVIS_ASSERT(this->source && this->target && this->corr);
    bool allEmpty = true;
    for(size_t i = 0; i < this->corr->size(); ++i) {
        if (!(*this->corr)[i].empty()) {
            allEmpty = false;
            break;
        }
    }
    COVIS_ASSERT_MSG(!allEmpty, "All empty correspondences input to RANSAC!");
    COVIS_ASSERT(this->sampleSize >= 3);
    COVIS_ASSERT(this->iterations > 0);
    COVIS_ASSERT(this->inlierThreshold > 0);
    COVIS_ASSERT(this->inlierFraction >= 0 && this->inlierFraction <= 1);

    // Instantiate pose sampler
    covis::detect::PoseSampler<PointT> poseSampler;
    poseSampler.setSource(this->source);
    poseSampler.setTarget(this->target);
    std::vector<int> sources( this->sampleSize );
    std::vector<int> targets( this->sampleSize );

    detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(this->target));
    fe->setOcclusionReasoning( !this->occlusionReasoning );
    fe->setViewAxis( this->viewAxis );
    if( this->occlusionReasoning )
        fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
    else
        fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);

    // Instantiate fit evaluator
    // if(!fe) // Not even created
    // fe.reset(new covis::detect::FitEvaluation<PointT>);
    fe->setInlierThreshold( this->inlierThreshold );
    // if(!fe->getSearch()) { // Not initialized with a search object
    //     if(_search && _search->getTarget() == this->target) // Search object provided to this, and consistent
    //         fe->setSearch(_search);
    //     else // Nothing provided, set target for indexing
    // }
    fe->setTarget(this->target);

    // Instantiate polygonal prerejector
    pcl::registration::CorrespondenceRejectorPoly<PointT,PointT> poly;
    poly.setInputSource( this->source );
    poly.setInputTarget( this->target );
    poly.setSimilarityThreshold( this->prerejectionSimilarity );

    // Instantiate geometric prerejector
    pcl::registration::CorrespondenceRejectorGeometric<PointT,PointT> geom;
    geom.setInputSource( this->source );
    geom.setInputTarget( this->target );

    // Output detection(s)
    core::Detection result;
    _allDetections.clear();
    if( this->corr->size() < this->sampleSize )
        return result;

    // Start main loop
    for(size_t i = 0; i < this->iterations; ++i) {

        // Create a sample from data
        const core::Correspondence::Vec maybeInliers =
                poseSampler.sampleCorrespondences(*this->corr, this->sampleSize);
        for(size_t j = 0; j < this->sampleSize; ++j) {
            sources[j] = maybeInliers[j].query;
            targets[j] = maybeInliers[j].match[0];
        }

        // Prerejection dissimilarity
        if ( this->prerejection_d ) {
            if( !poly.thresholdPolygon( sources, targets ) )
                continue;
        }

        // Prerejection geometric
        if ( this->prerejection_g ) {
            if( !geom.geometricConstraint( sources, targets ) )
                continue;
        }

        // Sample a pose model
        Eigen::Matrix4f pose = poseSampler.transformation( sources, targets );

        // Find consensus set
        fe->update( this->source, pose, this->corr ); // Using full models

        // If number of inliers (consensus set) is high enough
        if( fe->inlierFraction() >= this->inlierFraction ) {
            // Reestimate pose using consensus set
            if(fe->inliers() >= this->sampleSize) {
                pose = poseSampler.transformation(fe->getInliers());

                // Evaluate updated model
                fe->update(this->source, pose); // Using full models
            }

            // Add to the list of all detections
            _allDetections.push_back(core::Detection(pose,
                                                     fe->rmse(),
                                                     fe->penalty(),
                                                     fe->inlierFraction(),
                                                     fe->outlierFraction())
            );

            // Update result if updated model is the best so far
            if(fe->penalty() < result.penalty) {
                result.pose = pose;
                result.rmse = fe->rmse();
                result.inlierfrac = fe->inlierFraction();
                result.outlierfrac = fe->outlierFraction();
                result.penalty = fe->penalty();
            }
        }
    }

    // Collect translations and scores for all detections and perform NMS
    if(!_allDetections.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans(new pcl::PointCloud<pcl::PointXYZ>(_allDetections.size(), 1));
        std::vector<float> scores(_allDetections.size());
        for(size_t i = 0; i < _allDetections.size(); ++i) {
            trans->points[i].x = _allDetections[i].pose(0,3);
            trans->points[i].y = _allDetections[i].pose(1,3);
            trans->points[i].z = _allDetections[i].pose(2,3);
            scores[i] = 1 - _allDetections[i].penalty;
        }

        // Perform NMS
        filter::NonMaximumSuppression<pcl::PointXYZ> nms(0.2 * detect::computeDiagonal<PointT>(this->source), 0.1);
        nms.setScores(scores);
        nms.filter(trans);
        const std::vector<bool>& keep = nms.getMask();
        _allDetections = core::mask(_allDetections, keep);
    }

    // COVIS_MSG(result);
    return result;
}

std::vector<binaryClassification> ransac::benchmark( Eigen::Matrix4f ground_truth )
{
    // detect::PointSearch<PointT>::Ptr _search;
    covis::core::Detection::Vec _allDetections;

    // Sanity checks
    COVIS_ASSERT(this->source && this->target && this->corr);
    bool allEmpty = true;
    for(size_t i = 0; i < this->corr->size(); ++i) {
        if (!(*this->corr)[i].empty()) {
            allEmpty = false;
            break;
        }
    }
    COVIS_ASSERT_MSG(!allEmpty, "All empty correspondences input to RANSAC!");
    COVIS_ASSERT(this->sampleSize >= 3);
    COVIS_ASSERT(this->iterations > 0);

    // Instantiate pose sampler
    covis::detect::PoseSampler<PointT> poseSampler;
    poseSampler.setSource(this->source);
    poseSampler.setTarget(this->target);
    std::vector<int> sources( this->sampleSize );
    std::vector<int> targets( this->sampleSize );

    // Instantiate polygonal prerejector
    pcl::registration::CorrespondenceRejectorPoly<PointT,PointT> poly;
    poly.setInputSource( this->source );
    poly.setInputTarget( this->target );
    poly.setSimilarityThreshold( this->prerejectionSimilarity );

    // Instantiate geometric prerejector
    pcl::registration::CorrespondenceRejectorGeometric<PointT,PointT> geom;
    geom.setInputSource( this->source );
    geom.setInputTarget( this->target );

    // Prerejection statistics
    std::vector<binaryClassification> results;
    binaryClassification dissimilarity;
    binaryClassification geometric;

    // Start main loop
    for(size_t i = 0; i < this->iterations; ++i) {

        bool rejectDissimilarity = false;
        bool rejectGeometric = false;

        // Create a sample from data
        const core::Correspondence::Vec maybeInliers =
                poseSampler.sampleCorrespondences(*this->corr, this->sampleSize);
        for(size_t j = 0; j < this->sampleSize; ++j) {
            sources[j] = maybeInliers[j].query;
            targets[j] = maybeInliers[j].match[0];
        }

        // Prerejection dissimilarity
        {
            if( !poly.thresholdPolygon(sources, targets) )
                rejectDissimilarity = true;
        }

        // Prerejection geometric
        if ( this->prerejection_g ) {
            if( !geom.geometricConstraint( sources, targets ) )
                rejectGeometric = true;
        }

        // Sample a pose model
        Eigen::Matrix4f pose = poseSampler.transformation( sources, targets );

        // Calculate distance between pose and ground truth
        CloudT poseCloud, gtCloud;
        for (size_t j = 0; j < this->sampleSize; j++) {
            poseCloud.push_back(this->source->points[sources[j]]);
            gtCloud.push_back(this->source->points[sources[j]]);
        }

        covis::core::transform(poseCloud, pose);
        covis::core::transform(gtCloud, ground_truth);
        Eigen::Vector4f poseCentroid, gtCentroid;
        pcl::compute3DCentroid(poseCloud, poseCentroid);
        pcl::compute3DCentroid(gtCloud, gtCentroid);
        pcl::PointXYZ poseTranslation( poseCentroid(0), poseCentroid(1), poseCentroid(2) );
        pcl::PointXYZ gtTranslation( gtCentroid(0), gtCentroid(1), gtCentroid(2) );
        auto distance = pcl::euclideanDistance( poseTranslation, gtTranslation );

        // Determine if rejections were correct
        if ( distance < 1 ) {
            if ( !rejectDissimilarity ) dissimilarity.tp++; else dissimilarity.fp++;
            if ( !rejectGeometric ) geometric.tp++; else geometric.fp++;
        } else {
            if ( rejectDissimilarity ) dissimilarity.tn++; else dissimilarity.fn++;
            if ( rejectGeometric ) geometric.tn++; else geometric.fn++;
        }
    }
    results.push_back(dissimilarity);
    results.push_back(geometric);
    return results;
}
