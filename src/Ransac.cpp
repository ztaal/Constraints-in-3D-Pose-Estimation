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

#include <fstream>
using namespace covis::detect;

covis::core::Detection ransac::estimate()
{
    covis::core::Detection::Vec _allDetections;

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
    fe->setInlierThreshold( this->inlierThreshold );
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

    // Correspondence
    std::vector<covis::core::Correspondence> correspondences = *this->corr;

    // Output detection(s)
    core::Detection result;
    _allDetections.clear();
    if( correspondences.size() < this->sampleSize )
        return result;

    // Start main loop
    for(size_t i = 0; i < this->iterations; ++i) {

        // Create a sample from data
        std::vector<size_t> idx = covis::core::randidx( correspondences.size(), this->sampleSize );
        covis::core::Correspondence::Vec samples = covis::core::extract( correspondences, idx );

        for(size_t j = 0; j < this->sampleSize; ++j) {
            sources[j] = samples[j].query;
            targets[j] = samples[j].match[0];
        }

        // Prerejection dissimilarity
        if ( this->prerejection_d ) {
            if( !poly.thresholdPolygon( sources, targets ) ) {
                continue;
            }
        }

        // Prerejection geometric
        if ( this->prerejection_g ) {
            if( !geom.geometricConstraint( sources, targets ) ) {
                continue;
            }
        } else if ( this->prerejection_g2 ) { // TODO run test with this and then remove
            if( !geom.geometricConstraint2( sources, targets ) ) {
                continue;
            }
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

    return result;
}


/* Ransac with correction (2 steps: sort good from bad then normal ransac) */
covis::core::Detection ransac::estimate_correction()
{
    // detect::PointSearch<PointT>::Ptr _search;
    covis::core::Detection::Vec _allDetections;

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
    fe->setInlierThreshold( this->inlierThreshold );
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

    // Remove redundant correspondence
    std::vector<covis::core::Correspondence> correspondences = *this->corr;
    std::vector<covis::core::Correspondence> accepted;

    // Output detection(s)
    core::Detection result;
    _allDetections.clear();

    // Start main loop
    for(size_t i = 0; i < this->iterations; ++i) {

        // Create a sample from data
        std::vector<size_t> idx = covis::core::randidx( correspondences.size(), this->sampleSize );
        covis::core::Correspondence::Vec samples = covis::core::extract( correspondences, idx );
        for(size_t j = 0; j < this->sampleSize; ++j) {
            sources[j] = samples[j].query;
            targets[j] = samples[j].match[0];
        }

        // Prerejection dissimilarity
        if ( this->prerejection_d ) {
            if( !poly.thresholdPolygon( sources, targets ) ) {
                continue;
            }
        }

        // Prerejection geometric
        if ( this->prerejection_g ) {
            if( !geom.geometricConstraint( sources, targets ) ) {
                continue;
            }
        }

        // Sample a pose model
        Eigen::Matrix4f pose = poseSampler.transformation( sources, targets );

        // Find consensus set
        fe->update( this->source, pose, this->corr ); // Using full models

        // If number of inliers (consensus set) is high enough
        if( fe->inlierFraction() >= this->inlierFraction ) {
            // Add points to accepted vector
            if ( this->correction ) {
                for (size_t j = 0; j < this->sampleSize; j++) {
                    accepted.push_back(correspondences[idx[j]]);
                    correspondences[idx[j]] = correspondences.back();
                    correspondences.pop_back();
                }
            }

        }
    }

    if (accepted.size() < 3)
        return result;

    // Loop through all permutations of accepted correspondeces
    for(size_t i = 0; i < accepted.size() * 3; ++i) {
        // Create a sample from data
        std::vector<size_t> idx2 = covis::core::randidx( accepted.size(), this->sampleSize );
        covis::core::Correspondence::Vec samples2 = covis::core::extract( accepted, idx2 );
        for(size_t j = 0; j < this->sampleSize; ++j) {
            sources[j] = samples2[j].query;
            targets[j] = samples2[j].match[0];
        }

        // Prerejection dissimilarity
        if ( this->prerejection_d ) {
            if( !poly.thresholdPolygon( sources, targets ) ) {
                continue;
            }
        }

        // Prerejection geometric
        if ( this->prerejection_g ) {
            if( !geom.geometricConstraint( sources, targets ) ) {
                continue;
            }
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
    return result;
}


/* Ransac with correction (voting scheme and threshold) */
covis::core::Detection ransac::estimate_correction2()
{
    // detect::PointSearch<PointT>::Ptr _search;
    covis::core::Detection::Vec _allDetections;

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
    fe->setInlierThreshold( this->inlierThreshold );
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

    // Remove redundant correspondence
    std::vector<covis::core::Correspondence> correspondences = *this->corr;
    std::vector<covis::core::Correspondence> rejected;
    std::vector<int> corr_votes( correspondences.size() );
    int original_size = correspondences.size();
    double inliers = original_size * 0.3;

    // Output detection(s)
    core::Detection result;
    _allDetections.clear();


    // Start main loop
    for(size_t i = 0; i < this->iterations; ++i) {

        // Calculate threshold
        double median1 = median( corr_votes );
        double F = original_size / (double)(correspondences.size() - inliers);
        double threshold = median1 + F;

        // Create a sample from data
        std::vector<size_t> idx = covis::core::randidx( correspondences.size(), this->sampleSize );
        covis::core::Correspondence::Vec samples = covis::core::extract( correspondences, idx );
        for(size_t j = 0; j < this->sampleSize; ++j) {
            sources[j] = samples[j].query;
            targets[j] = samples[j].match[0];
        }

        // Prerejection dissimilarity
        if ( this->prerejection_d ) {
            if( !poly.thresholdPolygon( sources, targets ) ) {
                if ( this->correction ) {
                    int erased = 0;
                    for (size_t j = 0; j < this->sampleSize; j++) {
                        corr_votes[idx[j]] += 1;
                        if (corr_votes[idx[j]] > threshold) {
                            rejected.push_back(correspondences[idx[j] - erased]);
                            corr_votes.erase(corr_votes.begin() + idx[j] - erased);
                            correspondences.erase(correspondences.begin() + idx[j] - erased);
                            erased++;
                        }
                    }
                }
                continue;
            }
        }

        // Prerejection geometric
        if ( this->prerejection_g ) {
            if( !geom.geometricConstraint( sources, targets ) ) {
                if ( this->correction ) {
                    int erased = 0;
                    for (size_t j = 0; j < this->sampleSize; j++) {
                        corr_votes[idx[j]] += 1;
                        if (corr_votes[idx[j]] > threshold) {
                            rejected.push_back(correspondences[idx[j] - erased]);
                            corr_votes.erase(corr_votes.begin() + idx[j] - erased);
                            correspondences.erase(correspondences.begin() + idx[j] - erased);
                            erased++;
                        }
                    }
                }
                continue;
            }
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

                // Set to 0 if it was the best match found
                if ( this->correction ) {
                    for (size_t j = 0; j < this->sampleSize; j++) {
                        corr_votes[idx[j]] = 0;
                    }
                }
            }

            // Subtract if it was within the inlier fraction
            if ( this->correction ) {
                for (size_t j = 0; j < this->sampleSize; j++) {
                    corr_votes[idx[j]] -= 1;
                }
            }

        } else if ( this->correction ) { // If inliers are too low
            int erased = 0;
            for (size_t j = 0; j < this->sampleSize; j++) {
                corr_votes[idx[j]] += 1;
                if (corr_votes[idx[j]] > threshold) {
                    rejected.push_back(correspondences[idx[j] - erased]);
                    corr_votes.erase(corr_votes.begin() + idx[j] - erased);
                    correspondences.erase(correspondences.begin() + idx[j] - erased);
                    erased++;
                }
            }
        }
    }

    return result;
}
