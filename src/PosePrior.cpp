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

#include "../headers/PosePrior.hpp"

using namespace covis::detect;

covis::core::Detection posePrior::estimate()
{
    // Instantiate fit evaluator
    detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(this->target));
    fe->setOcclusionReasoning( !this->occlusionReasoning );
    fe->setViewAxis( this->viewAxis );
    if( this->occlusionReasoning )
        fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
    else
        fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);

    fe->setInlierThreshold( this->inlierThreshold );
    fe->setTarget( this->target );

    // Instantiate variables
    core::Detection result;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f> pose_vector;

    // Instantiate kd tree
    pcl::KdTree<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
    tree->setInputCloud(this->target);

    // Find centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*this->source, centroid);

    // Find table
    Eigen::Vector4f plane_normal;
    CloudT::Ptr tmp (new CloudT);
    pcl::copyPointCloud(*this->target, *tmp);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    while(true) {
        // Find largest plane
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000); // 1000
        seg.setDistanceThreshold(10);
        seg.setInputCloud(tmp);
        seg.segment(*inliers, *coefficients);

        if (coefficients->values[2] > 0) {
            coefficients->values[0] = -1 * coefficients->values[0];
            coefficients->values[1] = -1 * coefficients->values[1];
            coefficients->values[2] = -1 * coefficients->values[2];
            coefficients->values[3] = -1 * coefficients->values[3];
        }

        Eigen::Vector4f normal(coefficients->values[0],
                                coefficients->values[1],
                                coefficients->values[2],
                                coefficients->values[3]);

        // Loop over all correspondences in the target
        int pointsOnPlane = 0;
        for (size_t i = 0; i < this->corr->size(); i++) {
            PointT corrPoint = this->target->points[(*this->corr)[i].match[0]];
            Eigen::Vector4f point(corrPoint.x, corrPoint.y, corrPoint.z, 1);
            double dist = pcl::pointToPlaneDistance(corrPoint, normal);
            if (dist > 5 && dist < 150) // TODO Add threshold variables
                pointsOnPlane++;
        }
        // std::cout << "pointsOnPlane: " << pointsOnPlane << '\n';
        // std::cout << "Threshold: " << this->corr->size() * 0.15 << '\n';

        if (pointsOnPlane > this->corr->size() * 0.15) { // TODO Add threshold variable
            plane_normal = normal;
            break;
        } else { // Remove plane
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(tmp);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*tmp);
        }
        // visu::showDetection<PointT>( this->source, tmp, pose );

        // Check if no plane was found
        if (tmp->size() < this->target->size() * 0.2)
            return result;
    }

    // Create ortogonal basis
    Eigen::Matrix3f target_frame = ortogonal_basis(coefficients);

    // Find max z value of the source cloud
    PointT min_pt, max_pt;
    pcl::getMinMax3D( *this->source, min_pt, max_pt );

    // Loop over correspondences
    for( size_t i = 0; i < this->corr->size(); i++ ) {
        int source_corr = (*this->corr)[i].query;
        int target_corr = (*this->corr)[i].match[0];
        pose = Eigen::Matrix4f::Identity();

        // Constraint1: ignore point if it is below the plane
        PointT tgtPoint = this->target->points[target_corr];
        double tgt_dist = pcl::pointToPlaneDistanceSigned( tgtPoint, plane_normal );
        if (tgt_dist < 0)
        	continue;

        // Constraint2: if translation result in the object being below the plane
        // double src_dist = fabs(this->source->points[source_corr].z) + max_pt.z;
        // if ( fabs(tgt_dist) < src_dist * 0.8 ) // 0.8
        //     continue;

        // Prerejection3: if translation result in the object being above the plane
        // if ( fabs(tgt_dist) > src_dist ) // 0.8
        //     continue;

        // Rotate source to fit target plane
        Eigen::Vector4f corrPoint(this->source->points[source_corr].x,
                                    this->source->points[source_corr].y,
                                    this->source->points[source_corr].z,
                                    1);
        Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
        transformation.rotate(target_frame);

        // Translation
        transformation.translation() << this->target->points[target_corr].x - this->source->points[source_corr].x,
                                        this->target->points[target_corr].y - this->source->points[source_corr].y,
                                        this->target->points[target_corr].z - this->source->points[source_corr].z;

        // Subtract the correspondence to rotate around the correspondence
        Eigen::Affine3f correction = Eigen::Affine3f::Identity();
        correction.translation() = corrPoint.head<3>() - (target_frame * corrPoint.head<3>());
        transformation = correction * transformation;

        // Apply transformation
        pose = transformation.matrix() * pose;

        // Project normals onto plane
        Eigen::Vector4f source_normal(this->source->points[source_corr].normal_x,
                                        this->source->points[source_corr].normal_y,
                                        this->source->points[source_corr].normal_z, 0);
        Eigen::Vector3f target_normal(this->target->points[target_corr].normal_x,
                                        this->target->points[target_corr].normal_y,
                                        this->target->points[target_corr].normal_z);
        source_normal = transformation.matrix().inverse().transpose() * source_normal; // Transform normal
        Eigen::Vector3f source_projected = (source_normal.head<3>() - source_normal.head<3>().dot(plane_normal.head<3>()) * plane_normal.head<3>()).normalized();
        Eigen::Vector3f target_projected = (target_normal - target_normal.dot(plane_normal.head<3>()) * plane_normal.head<3>()).normalized();

        // Find rotation between projected normals (Rodrigues' rotation formula)
        // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        // https://gist.github.com/peteristhegreat/3b76d5169d7b9fc1e333
        Eigen::Vector3f v = source_projected.cross(target_projected);
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f ssc(3, 3);
        ssc << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        Eigen::Matrix3f R = I + ssc + (ssc * ssc) * (1 - source_projected.dot(target_projected)) / (v.norm() * v.norm());
        Eigen::Affine3f projected_transformation(R);

        // Subtract the correspondence to rotate around the correspondence
        corrPoint = transformation.matrix() * corrPoint;
        projected_transformation.translation() = corrPoint.head<3>() - (projected_transformation * corrPoint.head<3>());

        // Apply rotation
        pose = projected_transformation.matrix() * pose;

        // Constraint3: Reject pose if it is below the plane
        double pose_dist = plane_normal.dot( pose.block<4,1>(0, 3) );
        if ( pose_dist < max_pt.z * 0.8 ) // 0.8
            continue;

        // Constraint4: Reject pose if it is above the plane
        if ( pose_dist > max_pt.z * 1.2 ) // 1.2
            continue;

        // Constraint5: Find angel between normals and reject pose if it is too large
        source_normal = projected_transformation.matrix().inverse().transpose() * source_normal; // Transform normal
        double angle = atan2( (source_normal.head<3>().cross(target_normal)).norm(), source_normal.head<3>().dot(target_normal) );
        // if (angle > 0.05)
        // if (angle > 0.1) // Best so far
        // if (angle > 0.18)
        if (angle > 0.2) // Second best
        // if (angle > 0.3)
        // if (angle > 0.5)
        // if (angle > 1)
            continue;

        // Find consensus set
        fe->update( this->source, pose, this->corr ); // Using full models

        // If number of inliers (consensus set) is high enough
        if( fe->inlierFraction() >= this->inlierFraction ) {

            // Find closest point
            PointT point;
            point.x = pose(0,3);
            point.y = pose(1,3);
            point.z = pose(2,3);
            std::vector<int> nn_indices(1);
            std::vector<float> nn_dists(1);
            tree->nearestKSearch(point, 1, nn_indices, nn_dists);
            double tgtCentroidDist = sqrt(nn_dists[0]);

            // If closest point is too far away reject pose
            if ( tgtCentroidDist > this->srcCentroidDist * 2 )
                continue;

            pose_vector.push_back(pose);

            // Update result if updated model is the best so far
            if(fe->inlierFraction() > result.inlierfrac && !pose.isZero(0)) {
                // if(fe->penalty() < result.penalty && !pose.isZero(0)) {
                result.pose = pose;
                result.rmse = fe->rmse();
                result.inlierfrac = fe->inlierFraction();
                result.outlierfrac = fe->outlierFraction();
                result.penalty = fe->penalty();
            }
        }

    }

    bool show = false;
    // show = true;
    if (show == true) {
        // Visualize translations
        pcl::PointCloud<PointT>::Ptr results( new pcl::PointCloud<PointT>() );
        for (size_t i = 0; i < pose_vector.size(); i++) {
            PointT point;
            point.x = pose_vector[i](0,3);
            point.y = pose_vector[i](1,3);
            point.z = pose_vector[i](2,3);
            results->push_back(point);
        }

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
    }

    return result;
}
