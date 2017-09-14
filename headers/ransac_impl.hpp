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

#ifndef COVIS_DETECT_RANSAC_IMPL_HPP
#define COVIS_DETECT_RANSAC_IMPL_HPP

// Own
#include "ransac.h"
#include "pose_sampler.h"
#include "../core/progress_display.h"
#include "../filter/non_maximum_suppression.h"

// PCL
#include <pcl/registration/correspondence_rejection_poly.h>

namespace covis {
    namespace detect {
        template<typename PointT>
        core::Detection Ransac<PointT>::estimate() {
            // Sanity checks
            COVIS_ASSERT(_source && _target && _correspondences);
            bool allEmpty = true;
            for(size_t i = 0; i < _correspondences->size(); ++i) {
                if (!(*_correspondences)[i].empty()) {
                    allEmpty = false;
                    break;
                }
            }
            COVIS_ASSERT_MSG(!allEmpty, "All empty correspondences input to RANSAC!");
            COVIS_ASSERT(_sampleSize >= 3);
            COVIS_ASSERT(_iterations > 0);
            COVIS_ASSERT(_inlierThreshold > 0);
            COVIS_ASSERT(_inlierFraction >= 0 && _inlierFraction <= 1);
            if(_prerejection)
                COVIS_ASSERT(_prerejectionSimilarity >= 0.0f && _prerejectionSimilarity <= 1.0f);
            
            // Instantiate pose sampler
            PoseSampler<PointT> poseSampler;
            poseSampler.setSource(_source);
            poseSampler.setTarget(_target);
            std::vector<int> sources(_sampleSize);
            std::vector<int> targets(_sampleSize);
            
            
 
            // Weighted sampling only supported for Boost >= 1.47
#if BOOST_VERSION >= 104700
            // Instantiate distribution
            boost::random::discrete_distribution<size_t,double> corrDist;
            std::vector<double> corrQuality; // We set the quality to the negative feature distance
            if(_weightedSampling) {
                // Sum up all qualities
                corrQuality.resize(_correspondences->size());
                double sum = 0.0;
                for(size_t i = 0; i < _correspondences->size(); ++i) {
                    corrQuality[i] = -(*_correspondences)[i].distance[0];
                    sum += corrQuality[i];
                }
                // Normalize to get probabilities
                if(sum > 1e-5f) {
                    for(size_t i = 0; i < _correspondences->size(); ++i)
                        corrQuality[i] = corrQuality[i] / sum; // Highest probability for lowest distances
                } else { // All zeros
                    for(size_t i = 0; i < _correspondences->size(); ++i)
                        corrQuality[i] = 1.0 / double(_correspondences->size());
                }
                // Initialize distribution
                corrDist = boost::random::discrete_distribution<size_t,double>(corrQuality);
            }
#else
            // Warn if the user has tried to enable weighted sampling
            if(_weightedSampling)
                COVIS_MSG_WARN("Weighted sampling only supported for Boost >= 1.47!");
#endif           
            
            
            // Instantiate fit evaluator
            if(!_fitEvaluation) // Not even created
                _fitEvaluation.reset(new FitEvaluation<PointT>);
            _fitEvaluation->setInlierThreshold(_inlierThreshold);
            if(!_fitEvaluation->getSearch()) { // Not initialized with a search object
                if(_search && _search->getTarget() == _target) // Search object provided to this, and consistent
                    _fitEvaluation->setSearch(_search);
                else // Nothing provided, set target for indexing
                    _fitEvaluation->setTarget(_target);
            }
            
            // Instantiate polygonal prerejector
            pcl::registration::CorrespondenceRejectorPoly<PointT,PointT> poly;
            poly.setInputSource(_source);
            poly.setInputTarget(_target);
            poly.setSimilarityThreshold(_prerejectionSimilarity);
            
            // Output detection(s)
            core::Detection result;
            _allDetections.clear();
            if(_correspondences->size() < _sampleSize)
                return result;
            
            // Verbosity
            core::ProgressDisplay::Ptr pd;
            if(_verbose)
                pd.reset(new core::ProgressDisplay(_iterations, true));
            
            // Start main loop
            for(size_t i = 0; i < _iterations; ++i) {
                // Verbosity
                if(_verbose)
                    ++(*pd);

                // Create a sample from data
#if BOOST_VERSION >= 104700
                if(_weightedSampling) { // Weighted
                    // TODO: Sample without replacement instead
                    for(size_t j = 0; j < _sampleSize; ++j) {
                        core::Correspondence c = (*_correspondences)[corrDist(core::randgen)];
                        while(c.empty())
                            c = (*_correspondences)[corrDist(core::randgen)];
                        sources[j] = c.query;
                        targets[j] = c.match[0];
                        
                    }
                } else { // Uniform
                    const core::Correspondence::Vec maybeInliers =
                            poseSampler.sampleCorrespondences(*_correspondences, _sampleSize);
                    for(size_t j = 0; j < _sampleSize; ++j) {
                        sources[j] = maybeInliers[j].query;
                        targets[j] = maybeInliers[j].match[0];
                    }
                }
#else
                const core::Correspondence::Vec maybeInliers =
                        poseSampler.sampleCorrespondences(*_correspondences, _sampleSize);
                for(size_t j = 0; j < _sampleSize; ++j) {
                    sources[j] = maybeInliers[j].query;
                    targets[j] = maybeInliers[j].match[0];
                }
#endif
                
                // Apply prerejection
                if(_prerejection)
                    if(!poly.thresholdPolygon(sources, targets))
                        continue;
                
                // Sample a pose model
                Eigen::Matrix4f pose = poseSampler.transformation(sources, targets);

                // Find consensus set
                if(_fullEvaluation)
                    _fitEvaluation->update(_source, pose); // Using full models 
                else
                    _fitEvaluation->update(_source, pose, _correspondences); // Using input correspondences

                // If number of inliers (consensus set) is high enough
                if(_fitEvaluation->inlierFraction() >= _inlierFraction) {
                    // Reestimate pose using consensus set
                    if(_reestimatePose && _fitEvaluation->inliers() >= _sampleSize) {
                        pose = poseSampler.transformation(_fitEvaluation->getInliers());
                        
                        // Evaluate updated model
                        if(_fullEvaluation)
                            _fitEvaluation->update(_source, pose); // Using full models 
                        else
                            _fitEvaluation->update(_source, pose, _correspondences); // Using input correspondences
                    }
                    
                    // Add to the list of all detections
                    _allDetections.push_back(core::Detection(pose,
                                                             _fitEvaluation->rmse(),
                                                             _fitEvaluation->penalty(),
                                                             _fitEvaluation->inlierFraction(),
                                                             _fitEvaluation->outlierFraction())
                    );
                    
                    // Update result if updated model is the best so far (TODO: Allow for using other criteria here)
                    if(_fitEvaluation->penalty() < result.penalty) {
                        result.pose = pose;
                        result.rmse = _fitEvaluation->rmse();
                        result.inlierfrac = _fitEvaluation->inlierFraction();
                        result.outlierfrac = _fitEvaluation->outlierFraction();
                        result.penalty = _fitEvaluation->penalty();
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
                filter::NonMaximumSuppression<pcl::PointXYZ> nms(0.2 * computeDiagonal<PointT>(_source), 0.1);
                nms.setScores(scores);
                nms.filter(trans);
                const std::vector<bool>& keep = nms.getMask();
                _allDetections = core::mask(_allDetections, keep);
            }
            
            return result;
        }
    }
}
#endif
