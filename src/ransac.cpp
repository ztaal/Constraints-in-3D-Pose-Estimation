#include "../headers/ransac.hpp"

ransac::ransac()
{
}

ransac::~ransac()
{
}

void ransac::setSource( pcl::PointCloud<PointT>::Ptr _source )
{
    this->source = _source;
}

void ransac::setTarget( pcl::PointCloud<PointT>::Ptr _target )
{
    this->target = _target;
}

void ransac::setCorrespondences( core::Correspondence::VecPtr _corr )
{
    this->corr = _corr;
}

void ransac::setIterations( size_t _iterations )
{
    this->iterations = _iterations;
}

void ransac::setSampleSize( size_t _sampleSize )
{
    this->sampleSize = _sampleSize;
}

void ransac::setInlierThreshold( float _inlierThreshold )
{
    this->inlierThreshold = _inlierThreshold;
}

void ransac::setInlierFraction( float _inlierFraction )
{
    this->inlierFraction = _inlierFraction;
}

void ransac::setReestimatePose( bool _reestimatePose )
{
    this->reestimatePose = _reestimatePose;
}

void ransac::setFullEvaluation( bool _fullEvaluation )
{
    this->fullEvaluation = _fullEvaluation;
}

void ransac::setPrerejection( bool _prerejection )
{
    this->prerejection = _prerejection;
}

void ransac::setPrerejectionSimilarity( float _prerejectionSimilarty )
{
    this->prerejectionSimilarty = _prerejectionSimilarty;
}

void ransac::setOcclusionReasoning( bool _occlusionReasoning )
{
    this->occlusionReasoning = _occlusionReasoning;
}

void ransac::setViewAxis( int _viewAxis )
{
    this->viewAxis = _viewAxis;
}

void ransac::setRootPath( std::string _rootPath )
{
    this->rootPath = _rootPath;
}

void ransac::setObjectDir( std::string _objDir )
{
    this->objDir = _objDir;
}

void ransac::setSceneDir( std::string _sceneDir )
{
    this->sceneDir = _sceneDir;
}

void ransac::setPoseDir( std::string _poseDir )
{
    this->poseDir = _poseDir;
}

void ransac::setObjExt( std::string _objExt )
{
    this->objExt = _objExt;
}

void ransac::setSceneExt( std::string _sceneExt )
{
    this->sceneExt = _sceneExt;
}

void ransac::setPoseExt( std::string _poseExt )
{
    this->poseExt = _poseExt;
}

void ransac::setPoseSep( std::string _poseSep )
{
    this->poseSep = _poseSep;
}

void ransac::setVerbose( bool _verbose )
{
  this->verbose = _verbose;
}

core::Detection ransac::estimate()
{
    detect::PointSearch<PointT>::Ptr _search;
    core::Detection::Vec _allDetections;

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
    std::vector<int> sources(this->sampleSize);
    std::vector<int> targets(this->sampleSize);

    detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(this->target));
    fe->setOcclusionReasoning( !this->occlusionReasoning );
    fe->setViewAxis( this->viewAxis );
    if( this->occlusionReasoning )
        fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
    else
        fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);

    // Instantiate fit evaluator
    if(!fe) // Not even created
        fe.reset(new covis::detect::FitEvaluation<PointT>);
    fe->setInlierThreshold(this->inlierThreshold);
    if(!fe->getSearch()) { // Not initialized with a search object
        if(_search && _search->getTarget() == this->target) // Search object provided to this, and consistent
            fe->setSearch(_search);
        else // Nothing provided, set target for indexing
            fe->setTarget(this->target);
    }

    // Output detection(s)
    core::Detection result;
    _allDetections.clear();
    if(this->corr->size() < this->sampleSize)
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

        // Apply prerejection
        std::vector<double> delta(3);
        int max_obj_index = 0, min_obj_index = 0;
        int max_target_index = 0, min_target_index = 0;
        double max_dist_obj = 0, min_dist_obj = std::numeric_limits<double>::max();
        double max_dist_target = 0, min_dist_target = std::numeric_limits<double>::max();
        std::vector<double> source_sides(3);
        std::vector<double> target_sides(3);
        for (unsigned int j = 0; j < this->sampleSize; j++) {
            double d_p = pcl::euclideanDistance(this->source->points[sources[j]],
                                                this->source->points[sources[(j + 1) % this->sampleSize]]);
            double d_q = pcl::euclideanDistance(this->target->points[targets[j]],
                                                this->target->points[targets[(j + 1) % this->sampleSize]]);
            delta[j] = fabs(d_p - d_q) / std::max(d_p, d_q);

            if (d_p > max_dist_obj) {
                max_dist_obj = d_p;
                max_obj_index = j;
            }
            if (d_q > max_dist_target) {
                max_dist_target = d_q;
                max_target_index = j;
            }
            if (d_p < min_dist_obj) {
                min_dist_obj = d_p;
                min_obj_index = j + 1;
            }
            if (d_q < min_dist_target) {
                min_dist_target = d_q;
                min_target_index = j + 1;
            }

            source_sides[j] = d_p;
            target_sides[j] = d_q;
        }

        // If the index of shortest and furthest distance does not match continue
        if (max_obj_index != max_target_index || min_obj_index != min_target_index)
            continue;

        // If dissimilarity is too large continue
        if (fabs(std::max({delta[0], delta[1], delta[2]})) > 1 - this->prerejectionSimilarty)
            continue;

        // If the difference between the distances is to great continue
        float max_diff = max_dist_obj * 0.05;
        if (max_dist_target > max_dist_obj + max_diff || max_dist_target < max_dist_obj - max_diff ) {
            // std::cout << "Max break" << '\n';
            continue;
        }

        float min_diff = min_dist_obj * 0.05;
        if (min_dist_target > min_dist_obj + min_diff || min_dist_target < min_dist_obj - min_diff ) {
            // std::cout << "Min break" << '\n';
            continue;
        }

        // Area dissimilarity
        double source_p = (source_sides[0] + source_sides[1] + source_sides[2]) / 2;
        double target_p = (target_sides[0] + target_sides[1] + target_sides[2]) / 2;
        double source_area = sqrt(source_p * (source_p - source_sides[0])
                               * (source_p * source_sides[1])
                               * (source_p - source_sides[2]));
        double target_area = sqrt(target_p * (target_p - target_sides[0])
                               * (target_p * target_sides[1])
                               * (target_p - target_sides[2]));

        double area_diff = source_area * 0.05;
        if (target_area > source_area + area_diff || target_area < source_area - area_diff ) {
            // std::cout << "Areak break" << '\n';
            continue;
        }

        // Sample a pose model
        Eigen::Matrix4f pose = poseSampler.transformation(sources, targets);

        // Find consensus set
        fe->update(this->source, pose); // Using full models

        // If number of inliers (consensus set) is high enough
        if(fe->inlierFraction() >= this->inlierFraction) {
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

void ransac::loadData(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                        std::vector<util::DatasetLoader::ModelPtr> *sceneMesh)
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

    COVIS_MSG(dataset);

    // Load object and scene point clouds
    *objectMesh = dataset.getObjects();
    for(size_t i = 0; i < dataset.size(); ++i) {
        util::DatasetLoader::SceneEntry scene = dataset.at(i);
        (*sceneMesh).push_back(scene.scene);
    }
    COVIS_ASSERT(!objectMesh->empty() && !sceneMesh->empty());

}

void ransac::computeCorrespondence(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
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
        this->resolution /= float(objectMesh->size());
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

    // Estimation
    const float inlierThreshold =
            (this->inlierThreshold > 0.0 ?
            this->inlierThreshold * this->resolution :
            5 * this->resolution);

    // Preprocess
    objectSurf.resize(objectMesh->size());
    for(size_t i = 0; i < objectMesh->size(); ++i) {
        objectSurf[i] = filter::preprocess<PointT>((*objectMesh)[i], 1, true, this->far, this->resolution, nrad, false,
                                                  false, false);
        COVIS_ASSERT(!objectSurf[i]->empty());
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
                core::sort(*this->correspondences[i][j]);
                this->correspondences[i][j]->resize(this->correspondences[i][j]->size() * cutoff / 100);
            }
        }
    }
}

void ransac::initBenchmark()
{
    printf("Loading data\n");
    std::vector<util::DatasetLoader::ModelPtr> objectMesh, sceneMesh;
    loadData( &objectMesh, &sceneMesh );
    computeCorrespondence( &objectMesh, &sceneMesh );
}

void ransac::benchmark()
{
    printf("Benchmark initiated\n");

    // Call init if it has not been called before
    boost::call_once([this]{initBenchmark();}, this->flagInit);

    // Benchmark
    std::vector<std::vector<core::Detection> > d(sceneCloud.size());
    {
        // Start timer
        core::ScopedTimer t( "Benchmark" );

        // Run through scenes and estimate pose of each object
        for (size_t i = 0; i < sceneCloud.size(); i++) {
            d[i].resize(objectCloud.size());
            for (size_t j = 0; j < objectCloud.size(); j++) {

                if(this->verbose)
                    t.intermediate("Ransac time elapsed");

                setSource( this->objectCloud[j] );
                setTarget( this->sceneCloud[i] );
                setCorrespondences( this->correspondences[i][j] );
                d[i][j] = estimate();
                // result.pose = pose;
                // result.rmse = fe->rmse();
                // result.inlierfrac = fe->inlierFraction();
                // result.outlierfrac = fe->outlierFraction();
                // result.penalty = fe->penalty();
            }
            std::cout << "Iteration: " << i << " of " << sceneCloud.size() << '\n';
        }
    }
}
