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

void ransac::setFitEvaluation( detect::FitEvaluation<PointT>::Ptr _fe )
{
    this->fe = _fe;
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

    // Instantiate fit evaluator
    if(!this->fe) // Not even created
        this->fe.reset(new covis::detect::FitEvaluation<PointT>);
    this->fe->setInlierThreshold(this->inlierThreshold);
    if(!this->fe->getSearch()) { // Not initialized with a search object
        if(_search && _search->getTarget() == this->target) // Search object provided to this, and consistent
            this->fe->setSearch(_search);
        else // Nothing provided, set target for indexing
            this->fe->setTarget(this->target);
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
            std::cout << "Max break" << '\n';
            continue;
        }

        float min_diff = min_dist_obj * 0.05;
        if (min_dist_target > min_dist_obj + min_diff || min_dist_target < min_dist_obj - min_diff ) {
            std::cout << "Min break" << '\n';
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
            std::cout << "Areak break" << '\n';
            continue;
        }

        // Sample a pose model
        Eigen::Matrix4f pose = poseSampler.transformation(sources, targets);

        // Find consensus set
        this->fe->update(this->source, pose); // Using full models

        // If number of inliers (consensus set) is high enough
        if(this->fe->inlierFraction() >= this->inlierFraction) {
            // Reestimate pose using consensus set
            if(this->fe->inliers() >= this->sampleSize) {
                pose = poseSampler.transformation(this->fe->getInliers());

                // Evaluate updated model
                this->fe->update(this->source, pose); // Using full models
            }

            // Add to the list of all detections
            _allDetections.push_back(core::Detection(pose,
                                                     this->fe->rmse(),
                                                     this->fe->penalty(),
                                                     this->fe->inlierFraction(),
                                                     this->fe->outlierFraction())
            );

            // Update result if updated model is the best so far
            if(this->fe->penalty() < result.penalty) {
                result.pose = pose;
                result.rmse = this->fe->rmse();
                result.inlierfrac = this->fe->inlierFraction();
                result.outlierfrac = this->fe->outlierFraction();
                result.penalty = this->fe->penalty();
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
