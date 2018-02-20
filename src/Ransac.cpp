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

inline double median( std::vector<int> scores )
{
    size_t size = scores.size();
    std::sort(scores.begin(), scores.end());

    if (size % 2 == 0)
        return (scores[size / 2 - 1] + scores[size / 2]) / 2;

    return scores[size / 2];
}

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

    // Remove redundant correspondence
    std::vector<covis::core::Correspondence> correspondences = *this->corr;
    std::vector<int> corr_votes( correspondences.size() );
    int original_size = correspondences.size();
    double inliers = original_size * 0.1;

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
        } else if ( this->prerejection_g2 ) {
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


Eigen::Matrix3f ortogonal_basis(pcl::ModelCoefficients::Ptr coefficients)
{
    // Create ortogonal basis
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    Eigen::Vector3f z(a, b, c);
    Eigen::Vector3f y(0, -c, b);
    Eigen::Vector3f x = z.cross(y);
    Eigen::Matrix3f frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = z;

    return frame;
}

covis::core::Detection ransac::posePriors()
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

    // Initialize return variables
    core::Detection result;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f> pose_vector;
    covis::core::Correspondence::Vec correspondences = *this->corr;
    covis::core::sort(correspondences);

    // Find largest plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(10);
    seg.setInputCloud(this->target);
    seg.segment(*inliers, *coefficients);

    // Create ortogonal basis
    Eigen::Matrix3f target_frame = ortogonal_basis(coefficients);

    // Compute surface normals for the two matched correspondence
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setRadiusSearch(15);
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr source_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud (this->source);
    ne.compute (*source_normals);

    // Loop over correspondences
    // for( size_t i = 0; i < 1; i++ ) {
    for( size_t i = 0; i < correspondences.size(); i++ ) {

        // int source_corr = 1248;
        // int target_corr = 1063;
        int source_corr = correspondences[i].query;
        int target_corr = correspondences[i].match[0];
        pose = Eigen::Matrix4f::Identity();

        // Rotate source to fit target plane
        Eigen::Affine3f rotation = Eigen::Affine3f::Identity();
        rotation.rotate(target_frame);

        // Subtract the correspondence to rotate around it
        Eigen::Vector4f corrPoint(this->source->points[source_corr].x,
                                    this->source->points[source_corr].y,
                                    this->source->points[source_corr].z,
                                    1);
        rotation.translation() = corrPoint.head<3>() - (target_frame * corrPoint.head<3>());

        // Apply transformation
        Eigen::Affine3f translation = Eigen::Affine3f::Identity();
        translation.translation() << this->target->points[target_corr].x - this->source->points[source_corr].x,
                                        this->target->points[target_corr].y - this->source->points[source_corr].y,
                                        this->target->points[target_corr].z - this->source->points[source_corr].z;
        Eigen::Matrix4f transformation = (translation * rotation).matrix();
        pose *= transformation;

        // Compute surface normals for the two matched correspondence
        ne.setInputCloud (this->target);
        ne.compute (*target_normals);

        // Project normals onto plane
        Eigen::Vector4f source_normal(this->source->points[source_corr].normal_x, this->source->points[source_corr].normal_y, this->source->points[source_corr].normal_z, 0);
        Eigen::Vector3f target_normal(this->target->points[target_corr].normal_x, this->target->points[target_corr].normal_y, this->target->points[target_corr].normal_z);
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        source_normal = transformation.inverse().transpose() * source_normal; // Transform normal
        Eigen::Vector3f source_projected = source_normal.head<3>() - source_normal.head<3>().dot(plane_normal) * plane_normal;
        Eigen::Vector3f target_projected = target_normal - target_normal.dot(plane_normal) * plane_normal;

        // Find rotation between projected normals (Rodrigues' rotation formula)
        // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        // https://gist.github.com/peteristhegreat/3b76d5169d7b9fc1e333
        Eigen::Vector3f v = source_projected.cross(target_projected);
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f ssc(3, 3);
        ssc << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        Eigen::Matrix3f R = I + ssc + (ssc * ssc) * (1 - source_projected.dot(target_projected)) / (v.norm() * v.norm());
        Eigen::Affine3f projected_rotation(R);

        // Subtract the correspondence to rotate around it
        corrPoint = transformation * corrPoint;
        projected_rotation.translation() = corrPoint.head<3>() - (projected_rotation * corrPoint.head<3>());

        // Apply rotation
        Eigen::Affine3f translation2 = Eigen::Affine3f::Identity();
        Eigen::Matrix4f projected_transformation = (translation2 * projected_rotation).matrix();
        pose = projected_transformation * pose;
        // visu::showDetection<PointT>( this->source, this->target, pose );
        pose_vector.push_back(pose);
    }

    pcl::PointCloud<PointT>::Ptr results( new pcl::PointCloud<PointT>() );
    for (size_t i = 0; i < pose_vector.size(); i++) {
        PointT point;
        point.x = pose_vector[i](0,3);
        point.y = pose_vector[i](1,3);
        point.z = pose_vector[i](2,3);
        results->push_back(point);
    }
    // visu::showDetection<PointT>( results, this->target, pose );

    // Creates the visualization object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0, "global");

    viewer->addPointCloud<PointT> (results, "sample cloud");
    viewer->addPointCloud<PointT> (this->target, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(results, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(this->target, 0, 0, 255);
    viewer->updatePointCloud(results, red, "sample cloud");
    viewer->updatePointCloud(this->target, blue, "sample cloud2");

    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    result.pose = pose;
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
        double distance = 0;
        for (size_t i = 0; i < poseCloud.size(); i++) {
            double point_dist = pcl::euclideanDistance(poseCloud[i], gtCloud[i]);
            if (point_dist > distance)
                distance = point_dist;
        }

        // Determine if rejections were correct
        if ( distance < 5 ) {
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


/* Ransac with correction (2 steps: sort good from bad then normal ransac) */
void ransac::benchmark_correction( Eigen::Matrix4f ground_truth )
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
        return;

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
}



/* Ransac with correction (voting scheme and threshold) */
// void ransac::benchmark_correction( Eigen::Matrix4f ground_truth )
// {
//     // detect::PointSearch<PointT>::Ptr _search;
//     covis::core::Detection::Vec _allDetections;
//
//     // Sanity checks
//     COVIS_ASSERT(this->source && this->target && this->corr);
//     bool allEmpty = true;
//     for(size_t i = 0; i < this->corr->size(); ++i) {
//         if (!(*this->corr)[i].empty()) {
//             allEmpty = false;
//             break;
//         }
//     }
//     COVIS_ASSERT_MSG(!allEmpty, "All empty correspondences input to RANSAC!");
//     COVIS_ASSERT(this->sampleSize >= 3);
//     COVIS_ASSERT(this->iterations > 0);
//     COVIS_ASSERT(this->inlierThreshold > 0);
//     COVIS_ASSERT(this->inlierFraction >= 0 && this->inlierFraction <= 1);
//
//     // Instantiate pose sampler
//     covis::detect::PoseSampler<PointT> poseSampler;
//     poseSampler.setSource(this->source);
//     poseSampler.setTarget(this->target);
//     std::vector<int> sources( this->sampleSize );
//     std::vector<int> targets( this->sampleSize );
//
//     detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(this->target));
//     fe->setOcclusionReasoning( !this->occlusionReasoning );
//     fe->setViewAxis( this->viewAxis );
//     if( this->occlusionReasoning )
//         fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
//     else
//         fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);
//
//     // Instantiate fit evaluator
//     fe->setInlierThreshold( this->inlierThreshold );
//     fe->setTarget(this->target);
//
//     // Instantiate polygonal prerejector
//     pcl::registration::CorrespondenceRejectorPoly<PointT,PointT> poly;
//     poly.setInputSource( this->source );
//     poly.setInputTarget( this->target );
//     poly.setSimilarityThreshold( this->prerejectionSimilarity );
//
//     // Instantiate geometric prerejector
//     pcl::registration::CorrespondenceRejectorGeometric<PointT,PointT> geom;
//     geom.setInputSource( this->source );
//     geom.setInputTarget( this->target );
//
//     // Remove redundant correspondence
//     std::vector<covis::core::Correspondence> correspondences = *this->corr;
//     std::vector<covis::core::Correspondence> rejected;
//     std::vector<int> corr_votes( correspondences.size() );
//     int original_size = correspondences.size();
//     double inliers = original_size * 0.3;
//
//     // Output detection(s)
//     core::Detection result;
//     _allDetections.clear();
//
//     // Start main loop
//     for(size_t i = 0; i < this->iterations; ++i) {
//
//         // Calculate threshold
//         double median1 = median( corr_votes );
//         double F = original_size / (double)(correspondences.size() - inliers);
//         double threshold = median1 + F;
//
//         // Create a sample from data
//         std::vector<size_t> idx = covis::core::randidx( correspondences.size(), this->sampleSize );
//         covis::core::Correspondence::Vec samples = covis::core::extract( correspondences, idx );
//         for(size_t j = 0; j < this->sampleSize; ++j) {
//             sources[j] = samples[j].query;
//             targets[j] = samples[j].match[0];
//         }
//
//         // Prerejection dissimilarity
//         if ( this->prerejection_d ) {
//             if( !poly.thresholdPolygon( sources, targets ) ) {
//                 if ( this->correction ) {
//                     int erased = 0;
//                     for (size_t j = 0; j < this->sampleSize; j++) {
//                         corr_votes[idx[j]] += 1;
//                         if (corr_votes[idx[j]] > threshold) {
//                             rejected.push_back(correspondences[idx[j] - erased]);
//                             corr_votes.erase(corr_votes.begin() + idx[j] - erased);
//                             correspondences.erase(correspondences.begin() + idx[j] - erased);
//                             erased++;
//                         }
//                     }
//                 }
//                 continue;
//             }
//         }
//
//         // Prerejection geometric
//         if ( this->prerejection_g ) {
//             if( !geom.geometricConstraint( sources, targets ) ) {
//                 if ( this->correction ) {
//                     int erased = 0;
//                     for (size_t j = 0; j < this->sampleSize; j++) {
//                         corr_votes[idx[j]] += 1;
//                         if (corr_votes[idx[j]] > threshold) {
//                             rejected.push_back(correspondences[idx[j] - erased]);
//                             corr_votes.erase(corr_votes.begin() + idx[j] - erased);
//                             correspondences.erase(correspondences.begin() + idx[j] - erased);
//                             erased++;
//                         }
//                     }
//                 }
//                 continue;
//             }
//         }
//
//         // Sample a pose model
//         Eigen::Matrix4f pose = poseSampler.transformation( sources, targets );
//
//         // Find consensus set
//         fe->update( this->source, pose, this->corr ); // Using full models
//
//         // If number of inliers (consensus set) is high enough
//         if( fe->inlierFraction() >= this->inlierFraction ) {
//             // Reestimate pose using consensus set
//             if(fe->inliers() >= this->sampleSize) {
//                 pose = poseSampler.transformation(fe->getInliers());
//
//                 // Evaluate updated model
//                 fe->update(this->source, pose); // Using full models
//             }
//
//             // Add to the list of all detections
//             _allDetections.push_back(core::Detection(pose,
//                                                      fe->rmse(),
//                                                      fe->penalty(),
//                                                      fe->inlierFraction(),
//                                                      fe->outlierFraction())
//             );
//
//             // Update result if updated model is the best so far
//             if(fe->penalty() < result.penalty) {
//                 result.pose = pose;
//                 result.rmse = fe->rmse();
//                 result.inlierfrac = fe->inlierFraction();
//                 result.outlierfrac = fe->outlierFraction();
//                 result.penalty = fe->penalty();
//
//                 // Set to 0 if it was the best match found
//                 if ( this->correction ) {
//                     for (size_t j = 0; j < this->sampleSize; j++) {
//                         corr_votes[idx[j]] = 0;
//                     }
//                 }
//             }
//             // else if ( this->correction ) { // If inliers are too low
//             //     int erased = 0;
//             //     for (size_t j = 0; j < this->sampleSize; j++) {
//             //         corr_votes[idx[j]] += 1;
//             //         // if (corr_votes[idx[j]] > threshold) {
//             //         //     rejected.push_back(correspondences[idx[j] - erased]);
//             //         //     corr_votes.erase(corr_votes.begin() + idx[j] - erased);
//             //         //     correspondences.erase(correspondences.begin() + idx[j] - erased);
//             //         //     erased++;
//             //         // }
//             //     }
//             // }
//
//             // Subtract if it was within the inlier fraction
//             if ( this->correction ) {
//                 for (size_t j = 0; j < this->sampleSize; j++) {
//                     corr_votes[idx[j]] -= 1;
//                 }
//             }
//
//         }
//         else if ( this->correction ) { // If inliers are too low
//             int erased = 0;
//             for (size_t j = 0; j < this->sampleSize; j++) {
//                 corr_votes[idx[j]] += 1;
//                 if (corr_votes[idx[j]] > threshold) {
//                     rejected.push_back(correspondences[idx[j] - erased]);
//                     corr_votes.erase(corr_votes.begin() + idx[j] - erased);
//                     correspondences.erase(correspondences.begin() + idx[j] - erased);
//                     erased++;
//                 }
//             }
//         }
//     }
//
//     // Get points from accepted and rejected correspondences
//     CloudT accepted_source, accepted_target;
//     for (size_t i = 0; i < correspondences.size(); i++) {
//         accepted_source.push_back(this->source->points[correspondences[i].query]);
//         accepted_target.push_back(this->target->points[correspondences[i].match[0]]);
//     }
//
//     CloudT rejected_source, rejected_target;
//     for (size_t i = 0; i < rejected.size(); i++) {
//         rejected_source.push_back(this->source->points[rejected[i].query]);
//         rejected_target.push_back(this->target->points[rejected[i].match[0]]);
//     }
//
//     // Transform using ground truth
//     covis::core::transform(accepted_source, ground_truth);
//     covis::core::transform(rejected_source, ground_truth);
//
//     // Find distances
//     std::vector<double> distance_accepted( correspondences.size() );
//     for (size_t i = 0; i < correspondences.size(); i++) {
//         double dist = pcl::euclideanDistance(accepted_source[i], accepted_target[i]);
//         // Floor to nearest half
//         dist = std::floor((dist * 2) + 0.5) / 2;
//         distance_accepted.push_back( dist );
//     }
//     std::sort (distance_accepted.begin(), distance_accepted.end());
//
//     std::vector<double> distance_rejected( rejected.size() );
//     for (size_t i = 0; i < rejected.size(); i++) {
//         double dist = pcl::euclideanDistance(rejected_source[i], rejected_target[i]);
//         // Floor to nearest half
//         dist = std::floor((dist * 2) + 0.5) / 2;
//         distance_rejected.push_back( dist );
//     }
//     std::sort (distance_rejected.begin(), distance_rejected.end());
//
//     // Write to file
//     ofstream accepted_file;
//     accepted_file.open ("../correction_accepted.txt");
//     for (auto &dist : distance_accepted)
//         accepted_file << dist << "\n";
//     accepted_file.close();
//
//     ofstream rejected_file;
//     rejected_file.open ("../correction_rejected.txt");
//     for (auto &dist : distance_rejected)
//         rejected_file << dist << "\n";
//     rejected_file.close();
// }
