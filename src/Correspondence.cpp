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

#include "../headers/Correspondence.hpp"

using namespace covis::detect;

void correspondence::compute( std::string query, std::string target )
{
    // Load models
    pcl::PolygonMesh::Ptr queryMesh(new pcl::PolygonMesh);
    pcl::PolygonMesh::Ptr targetMesh(new pcl::PolygonMesh);
    util::load( query, *queryMesh);
    util::load( target, *targetMesh);

    // Surfaces and normals
    const bool resolutionInput = (this->resolution > 0.0f);
    if(!resolutionInput)
        this->resolution = detect::computeResolution(queryMesh) * this->objectScale;
    if(objectScale != 1)
        queryMesh = filter::scale(queryMesh, this->objectScale);

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
    COVIS_ASSERT( this->cutoff > 0 && this->cutoff <= 100 );

    // Preprocess
    pcl::PointCloud<PointT>::Ptr querySurf, targetSurf;
    querySurf = filter::preprocess<PointT>(queryMesh, 1, true, this->far, this->resolution, nrad, false, false, verbose);
    targetSurf = filter::preprocess<PointT>(targetMesh, 1, true, this->far, this->resolution, nrad, false, false, verbose);
    COVIS_ASSERT( !querySurf->empty() && !targetSurf->empty() );

    // Generate feature points
    this->queryCloud = filter::downsample<PointT>(querySurf, resQuery);
    this->targetCloud = filter::downsample<PointT>(targetSurf, resTarget);
    COVIS_ASSERT( !queryCloud->empty() && !targetCloud->empty() );

    // // Filter x-value TODO Remove cropping only used to make tests easier
    // pcl::PassThrough<PointT> pass;
    // pass.setInputCloud(targetCloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits (-150, 200);
    // pass.filter (*targetCloud);
    // // Filter y-value TODO Remove cropping only used to make tests easier
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits (-200, 50);
    // pass.filter (*targetCloud);

    // Compute features
    feature::MatrixT objectFeat, sceneFeat;
    objectFeat = feature::computeFeature<PointT>(this->feature, this->queryCloud, querySurf, frad);
    sceneFeat = feature::computeFeature<PointT>(this->feature, this->targetCloud, targetSurf, frad);

     // Match features
    this->corr = detect::computeRatioMatches(objectFeat, sceneFeat);

    // Sort correspondences and cutoff at <cutoff> %
    if(this->cutoff < 100) {
        covis::core::sort(*this->corr);
        this->corr->resize(this->corr->size() * this->cutoff / 100);
    }
}
