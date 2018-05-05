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

void rgb2hsv( double R, double G, double B, double *_H, double *_S, double *_V )
{
    double m = std::min(R, std::min(G, B));
    double M = std::max(R, std::max(G, B));
    double C = M - m;
    double H = 0;
    if (M == R)
        H = 60 * (G - B) / C + 360;
    else if (M == G)
        H = 60 * (B - R) / C + 360;
    else if (M == B)
        H = 60 * (R - G) / C + 360;

    double V = M;
    // double L = 0.5 * (M + m);
    double Sv = 0;
    if (C != 0)
        Sv = C / M;
    // double Sl = 0;
    // if (L <= 0.5 && C != 0)
    //     Sl = C / (M + m);
    // else if (L > 0.5 && C != 0)
    //     Sl = C / (2 - M + m);

    *_H = H;
    *_S = Sv;
    *_V = V;
}


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

    // Find max z value of the source cloud
    PointT min_pt, max_pt;
    pcl::getMinMax3D( *this->source, min_pt, max_pt );
    double maxDist = max_pt.z - min_pt.z;

    // Correction of pose due to the models not being proberbly aligned with the axis TODO remove when correct models are used
    Eigen::Affine3f correctionT = Eigen::Affine3f::Identity();
    Eigen::Vector3f unitX = Eigen::Vector3f::UnitX();
    Eigen::Vector3f unitY = Eigen::Vector3f::UnitY();
    Eigen::Vector3f unitZ = Eigen::Vector3f::UnitZ();
    double thetaX = 0, thetaY = 0, thetaZ = 0;
    if (this->dataset == "tejani") {
        if (this->modelIndex == 1 || this->modelIndex == 3)
            thetaX = -3;
        else if (this->modelIndex == 5)
            thetaX = -10;

    } else if (this->dataset == "t-less") {
        if (this->sequence == 1) {
            if (this->modelIndex == 25)
                thetaX = 90;
            else if (this->modelIndex == 29)
                thetaX = 86;
        } else if (this->sequence == 3) {
            if (this->modelIndex == 5 || this->modelIndex == 8 || this->modelIndex == 11 || this->modelIndex == 12 )
                thetaX = 90;
            else if (this->modelIndex == 18)
                thetaX = 180;
        } else if (this->sequence == 4) {
            if (this->modelIndex == 8)
                thetaX = 180;
        } else if (this->sequence == 5) {
            if (this->modelIndex == 4)
                thetaX = -75;
            else if (this->modelIndex == 9)
                thetaX = -68;
        } else if (this->sequence == 6) {
            if (this->modelIndex == 6)
                thetaX = 90;
            else if (this->modelIndex == 7 || this->modelIndex == 11)
                thetaX = 180;
        } else if (this->sequence == 7) {
            if (this->modelIndex == 18) {
                thetaZ = 30;
                thetaX = 14;
            }
        } else if (this->sequence == 8) {
            if (this->modelIndex == 22)
                thetaY = -90;
            else if (this->modelIndex == 23)
                thetaX = 90;
        } else if (this->sequence == 10) {
            if (this->modelIndex == 20)
                thetaY = 90;
            else if (this->modelIndex == 21)
                thetaY = -90;
            else if (this->modelIndex == 24)
                thetaY = 90;
        } else if (this->sequence == 13) {
            if (this->modelIndex == 19)
                thetaY = 90;
            else if (this->modelIndex == 20)
                thetaX = 90;
        } else if (this->sequence == 14) {
            if (this->modelIndex == 19 || this->modelIndex == 20 )
                thetaY = 90;
        } else if (this->sequence == 15) {
            if (this->modelIndex == 26)
                thetaX = 90;
        } else if (this->sequence == 16) {
            if (this->modelIndex == 13)
                thetaX = 86;
            else if (this->modelIndex == 14)
                thetaX = 85;
            else if (this->modelIndex == 14)
                thetaX = 85;
        } else if (this->sequence == 19) {
            if (this->modelIndex == 13 || this->modelIndex == 15 || this->modelIndex == 16 ||
                this->modelIndex == 17 || this->modelIndex == 18)
                thetaX = 180;
            else if (this->modelIndex == 14)
                thetaX = 85;
        } else if (this->sequence == 20) {
                thetaX = 85;
        }
    }
    if (thetaZ != 0)
        correctionT = Eigen::AngleAxisf(thetaZ * M_PI / 180, unitZ) * correctionT;
    if (thetaY != 0)
        correctionT = Eigen::AngleAxisf(thetaY * M_PI / 180, unitY) * correctionT;
    if (thetaX != 0)
        correctionT = Eigen::AngleAxisf(thetaX * M_PI / 180, unitX) * correctionT;

    // Instantiate kd tree
    pcl::KdTree<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
    pcl::KdTree<PointT>::Ptr srcTree (new pcl::KdTreeFLANN<PointT>);
    tree->setInputCloud(this->target);
    srcTree->setInputCloud(this->source);

    // Find centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*this->source, centroid);

    // Find table
    Eigen::Vector4f plane_normal;
    CloudT::Ptr tmp (new CloudT);
    pcl::copyPointCloud(*this->target, *tmp);
    CloudT::Ptr feCloud (new CloudT);
    pcl::copyPointCloud(*this->target, *feCloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    // std::cout << "Dist: " << this->srcCentroidDist << '\n';

    while(true) {
        // Find largest plane
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1500); // 1000
        // seg.setMaxIterations(1000); // Tejani // 1000
        seg.setDistanceThreshold(10); // Tejani // 10
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
            // if (dist > 5 && dist < 200) // Tejani TODO Add threshold variables
            // if (dist > 50 && dist < 250) // Hintertoisser TODO Add threshold variables
            if (dist > 15 && dist < 300) // T-less TODO Add threshold variables
                pointsOnPlane++;
        }
        // std::cout << "\npointsOnPlane: " << pointsOnPlane << '\n';
        // std::cout << "Threshold: " << this->corr->size() * 0.18 << '\n';
        // if (pointsOnPlane > this->corr->size() * 0.25) { // Tejani TODO Add threshold variable
        // if (pointsOnPlane > this->corr->size() * 0.15) { // Hintertoisser TODO Add threshold variable
        if (pointsOnPlane > this->corr->size() * 0.15) { // T-less TODO Add threshold variable
            plane_normal = normal;
            break;
        } else { // Remove plane
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(tmp);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*tmp);
        }

        // Check if no plane was found
        if (tmp->size() < this->target->size() * 0.3)  // TODO add variable
            return result;
    }

    // Create ortogonal basis
    Eigen::Matrix3f target_frame = ortogonal_basis(coefficients);

    // Remove correspondences that are below the plane
    for( auto it = this->corr->begin(); it != this->corr->end(); ) {
        int target_corr = (*it).match[0];
        PointT tgtPoint = this->target->points[target_corr];
        double tgt_dist = pcl::pointToPlaneDistanceSigned( tgtPoint, plane_normal );
        // if (tgt_dist < 0)
        if (tgt_dist < 0 || tgt_dist > maxDist * 1.2)
            it = this->corr->erase(it);
        else
            ++it;
    }

    double avgColorDist = 0;
    double colorCount = 0;

    // Loop over correspondences
    for( size_t i = 0; i < this->corr->size(); i++ ) {
        int source_corr = (*this->corr)[i].query;
        int target_corr = (*this->corr)[i].match[0];
        pose = correctionT * Eigen::Matrix4f::Identity();

        // // Constraint0: ignore point if it is below the plane
        // PointT srcPoint = this->source->points[source_corr];
        // PointT tgtPoint = this->target->points[target_corr];
        // double tgt_dist = pcl::pointToPlaneDistanceSigned( tgtPoint, plane_normal );
        // // if (tgt_dist < 0)
        // // if (tgt_dist < tgt_dist * -0.1) // -0.1
        // // 	continue;

        // Constraint1: ignore point if it is below the plane
        PointT srcPoint = this->source->points[source_corr];
        PointT tgtPoint = this->target->points[target_corr];
        // double tgt_dist = pcl::pointToPlaneDistanceSigned( tgtPoint, plane_normal );
        // if (tgt_dist < this->inlierThreshold)
        // if (tgt_dist < 10)
        // if (tgt_dist < tgt_dist * -0.1) // -0.1
        	// continue;

        // Rotate source to fit target plane
        Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
        transformation.rotate(target_frame);

        // Translation
        Eigen::Vector4f corrPoint(srcPoint.x, srcPoint.y, srcPoint.z, 1);
        transformation.translation() << tgtPoint.x - srcPoint.x,
                                        tgtPoint.y - srcPoint.y,
                                        tgtPoint.z - srcPoint.z;

        // Subtract the correspondence to rotate around the correspondence
        Eigen::Affine3f correction = Eigen::Affine3f::Identity();
        correction.translation() = corrPoint.head<3>() - (target_frame * corrPoint.head<3>());
        transformation = correction * transformation;

        // Apply transformation
        pose = transformation.matrix() * pose;

        // Project normals onto plane
        Eigen::Vector4f source_normal(srcPoint.normal_x, srcPoint.normal_y, srcPoint.normal_z, 0);
        Eigen::Vector3f target_normal(tgtPoint.normal_x, tgtPoint.normal_y, tgtPoint.normal_z);
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

        // Constraint2: Reject pose if it is below the plane
        double pose_dist = plane_normal.dot( pose.block<4,1>(0, 3) );
        // if ( pose_dist < (maxDist/2) * 0.8 ) // Tejani // TODO add variable
        // if ( pose_dist < (maxDist/2) * 0.8 ) // Hinterstoisser TODO add variable // 0.8 BEST
        if ( pose_dist < (maxDist/2) * 0.8 ) // T-less // 0.8 BEST
            continue;

        // Constraint3: Reject pose if it is above the plane
        // if ( pose_dist > (maxDist/2) * 1.2 ) // Tejani // TODO add variable
        // if ( pose_dist > (maxDist/2) * 1.2 ) // Hinterstoisser // 1.2 BEST
        // if ( pose_dist > (maxDist/2) * 1.1 ) // T-less // 1.2 BEST
            // continue;

        // Constraint4: Find angel between normals and reject pose if it is too large
        source_normal = projected_transformation.matrix().inverse().transpose() * source_normal; // Transform normal
        double angle = atan2( (source_normal.head<3>().cross(target_normal)).norm(), source_normal.head<3>().dot(target_normal) );
        // if (angle > 0.5) // TODO add variable // 0.5 BEST
        // if (angle > 0.05) // 0.2  // TODO add variable
            // continue;

        // Constraint4.5: Find angel between plane normal and source normal and reject pose if it is too large
        double planeAngle = atan2( (plane_normal.head<3>().cross(target_normal)).norm(), plane_normal.head<3>().dot(target_normal) );
        // if (planeAngle < 0.3) // TODO add variable // 0.5 BEST
        // if (angle > 0.05) // 0.2  // TODO add variable
            // continue;

       // Find closest point
        PointT point;
        point.x = pose(0,3);
        point.y = pose(1,3);
        point.z = pose(2,3);
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree->nearestKSearch(point, 1, nn_indices, nn_dists);
        double tgtCentroidDist = sqrt(nn_dists[0]);
        // Constraint5: If closest point is too far away reject pose
        // if ( tgtCentroidDist < this->srcCentroidDist * 0.5 ) // TODO add variable // 0.5 BEST
        // if ( tgtCentroidDist < this->srcCentroidDist * 0.5 ) // Hinterstoisser // TODO add variable // 0.5 BEST
        if ( tgtCentroidDist < this->srcCentroidDist * 0.8 ) // Hinterstoisser // TODO add variable // 0.5 BEST
        // if ( tgtCentroidDist > this->srcCentroidDist * 3 || tgtCentroidDist < this->srcCentroidDist * 0.5 ) // TODO add variable
        // if ( tgtCentroidDist > this->srcCentroidDist * 2 || tgtCentroidDist < this->srcCentroidDist * 0.5 ) // Tejani TODO add variable
            continue;

        // // Constraint6: Color
        // int numberOfPoints = 5; // Hintertoisser // Best 20
        // std::vector<double> tgtRGB = {0, 0, 0};
        // std::vector<double> srcRGB = {0, 0, 0};
        // std::vector<int> tgtIndices(numberOfPoints), srcIndices(numberOfPoints);
        // std::vector<float> tgtDists(numberOfPoints), srcDists(numberOfPoints);
        // tree->nearestKSearch(tgtPoint, numberOfPoints, tgtIndices, tgtDists);
        // srcTree->nearestKSearch(srcPoint, numberOfPoints, srcIndices, srcDists);
        // for (int i = 0; i < numberOfPoints; i++) {
        //     PointT tgtColorPoint = this->target->points[tgtIndices[i]];
        //     PointT srcColorPoint = this->source->points[srcIndices[i]];
        //     tgtRGB[0] += int(tgtColorPoint.r);
        //     tgtRGB[1] += int(tgtColorPoint.g);
        //     tgtRGB[2] += int(tgtColorPoint.b);
        //     srcRGB[0] += int(srcColorPoint.r);
        //     srcRGB[1] += int(srcColorPoint.g);
        //     srcRGB[2] += int(srcColorPoint.b);
        // }
        // tgtRGB[0] /= numberOfPoints;
        // tgtRGB[1] /= numberOfPoints;
        // tgtRGB[2] /= numberOfPoints;
        // srcRGB[0] /= numberOfPoints;
        // srcRGB[1] /= numberOfPoints;
        // srcRGB[2] /= numberOfPoints;
        // double colorDist = sqrt(pow(srcRGB[0] - tgtRGB[0], 2) + pow(srcRGB[1] - tgtRGB[1], 2) + pow(srcRGB[2] - tgtRGB[2], 2));
        // avgColorDist += colorDist;
        // colorCount++;
        // // std::cout << "colorDist: " << colorDist << "\tthreshold: " << avgColorDist/colorCount * 0.8 << '\n';
        // if (colorDist > avgColorDist/colorCount * 0.8)
        //     continue;

        // Find consensus set
        fe->update( this->source, pose, this->corr ); // Using full models

        pose_vector.push_back(pose);

        // Update result if updated model is the best so far
        // double prevInlierFrac = fe->inlierFraction();
        if(fe->inlierFraction() > result.inlierfrac && !pose.isZero(0)) {
            // // Constraint 8?
            // Eigen::Vector3f tgtPointEigen(tgtPoint.x, tgtPoint.y, tgtPoint.z);
            // Eigen::Vector4f srcNormal(srcPoint.normal_x, srcPoint.normal_y, srcPoint.normal_z, 1);
            // srcNormal = pose.inverse().transpose() * srcNormal;
            // Eigen::Vector3f dirVec = srcNormal.head<3>().cross(plane_normal.head<3>());
            // Eigen::Vector3f newPoseVec = pose.block<3,1>(0, 3) + (15 * dirVec);
            // Eigen::Matrix4f newPose = pose;
            // newPose(0,3) = newPoseVec[0];
            // newPose(1,3) = newPoseVec[1];
            // newPose(2,3) = newPoseVec[2];
            // fe->update( this->source, newPose, this->corr );
            // if (prevInlierFrac < fe->inlierFraction() * 1.1)
            //     continue;
            // result.inlierfrac = prevInlierFrac;

            result.pose = pose;
            result.rmse = fe->rmse();
            result.inlierfrac = fe->inlierFraction();
            result.outlierfrac = fe->outlierFraction();
            result.penalty = fe->penalty();
        }
    }

    // // Iterative Closest Point (refine pose)
    // if (result) {
    //     pcl::IterativeClosestPoint<PointT, PointT> icp;
    //     // pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
    //     icp.setMaximumIterations( this->icpIterations );
    //     icp.setInputSource( this->source );
    //     icp.setInputTarget( this->target );
    //     // icp.setMaxCorrespondenceDistance( 8 );
    //     icp.setMaxCorrespondenceDistance( this->inlierThreshold );
    //     CloudT tmp;
    //     icp.align( tmp, result.pose );
    //     if(icp.hasConverged()) {
    //         result.pose = icp.getFinalTransformation();
    //         result.rmse = icp.getFitnessScore();
    //     } else {
    //         result.clear();
    //     }
    // }

    // Visualize translations
    bool show = false;
    // show = true;
    if (show == true) {
        pcl::PointCloud<PointT>::Ptr results( new pcl::PointCloud<PointT>() );
        // for (size_t i = 0; i < this->corr->size(); i++) {
        //     PointT corrPoint = this->target->points[(*this->corr)[i].match[0]];
        //     Eigen::Vector4f point(corrPoint.x, corrPoint.y, corrPoint.z, 1);
        //     double dist = pcl::pointToPlaneDistance(corrPoint, plane_normal);
        //    // if (dist > 2 * this->inlierThreshold)
        //     results->push_back(corrPoint);
        // }
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
        viewer->addPointCloud<PointT> (feCloud, "sample cloud2");
        // viewer->addPointCloud<PointT> (this->target, "sample cloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> red(results, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(feCloud, 0, 0, 255);
        // pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(this->target, 0, 0, 255);
        viewer->updatePointCloud(results, red, "sample cloud");
        viewer->updatePointCloud(feCloud, blue, "sample cloud2");
        // viewer->updatePointCloud(this->target, blue, "sample cloud2");

        while (!viewer->wasStopped ()) {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    return result;
}
