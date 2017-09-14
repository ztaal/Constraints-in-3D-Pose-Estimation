#include <covis/covis.h>
using namespace covis;

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/random.h>

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Loaded point clouds and computed histogram features
pcl::PointCloud<PointT>::Ptr querySurf, targetSurf;
pcl::PointCloud<PointT>::Ptr queryCloud, targetCloud;
feature::MatrixT queryFeat, targetFeat;

core::Detection pose_estimation_RANSAC(
    pcl::PointCloud<PointT>::Ptr object_cloud,
    pcl::PointCloud<PointT>::Ptr target_cloud,
    core::Correspondence::VecPtr correspondences,
    float threshold, size_t iterations);

int main( int argc, const char** argv )
{
    pcl::ScopeTime( "Main" );
    printf( " -- Initialized Program\n" );

    // Setup program options
    core::ProgramOptions po;
    po.addPositional("query", "mesh or point cloud file for query model");
    po.addPositional("target", "mesh or point cloud file for target model");

    // Surfaces and normals
    po.addOption("resolution", 'r', 0.001, "downsample point clouds to this resolution (<= 0 for disabled)");
    po.addOption("far", -1, "do not consider target points beyond this depth (<= 0 for disabled)");
    po.addOption("radius-normal", 'n', 5, "normal estimation radius in mr (<= 0 means two resolution units)");
    po.addFlag('o', "orient-query-normals", "ensure consistent normal orientation for the query model");

    // Features and matching
    po.addOption("feature", "si", "choose which feature to use from this list: " + feature::FeatureNames);
    po.addOption("resolution-query", 5, "resolution of query features in mr (<= 0 for five resolution units)");
    po.addOption("resolution-target", 5, "resolution of target features in mr (<= 0 for five resolution units)");
    po.addOption("radius-feature", 'f', 25, "feature estimation radius (<= 0 means 25 resolution units)");
    po.addOption("cutoff", 50, "use the <cutoff> % best L2 ratio correspondences for RANSAC");

    // Estimation
    po.addOption("iterations", 'i', 1000, "RANSAC iterations");
    po.addOption("inlier-threshold", 't', 5, "RANSAC inlier threshold (<= 0 for infinite)");
    po.addOption("inlier-fraction", 'a', 0.05, "RANSAC inlier fraction required for accepting a pose hypothesis");
    po.addFlag('e', "no-reestimate", "disable re-estimation of pose hypotheses using consensus set during RANSAC");
    po.addFlag('u', "full-evaluation", "enable full pose evaluation during RANSAC, otherwise only the existing feature matches are used during verification");
    po.addFlag('p', "prerejection", "enable prerejection during RANSAC");
    po.addOption("prerejection-similarity", 's', 0.9, "prerejection similarity threshold in [0,1]");
    po.addFlag('c', "no-occlusion-reasoning", "disable occlusion reasoning during pose hypothesis evaluation");
    po.addOption("view-axis", 'x', 2, "if occlusion reasoning is on (default), assume axis x, y, or z (0, 1 or 2) to point in the direction of the view");

    // Refinement
    po.addFlag("refine", "apply pose refinement of the RANSAC result using ICP");
    po.addOption("icp-iterations", 25, "number of ICP iterations");

    // Misc.
    po.addFlag('v', "visualize", "show additional results");

    // Parse
    if(!po.parse(argc, argv))
        return 1;
    po.print();

    // Load models
    pcl::PolygonMesh::Ptr queryMesh(new pcl::PolygonMesh);
    pcl::PolygonMesh::Ptr targetMesh(new pcl::PolygonMesh);
    util::load(po.getValue("query"), *queryMesh);
    util::load(po.getValue("target"), *targetMesh);

    // Surfaces and normals
    float res = po.getValue<float>("resolution");
    const bool resolutionInput = (res > 0.0f);
    if(!resolutionInput)
        res = detect::computeResolution(targetMesh);
    const float far = po.getValue<float>("far");
    const float nrad =
            po.getValue<float>("radius-normal") > 0.0 ?
                    po.getValue<float>("radius-normal") * res :
                    2 * res;

    // Features and matching
    const float resQuery =
            po.getValue<float>("resolution-query") > 0.0 ?
                    po.getValue<float>("resolution-query") * res :
                    5 * res;
    const float resTarget =
            po.getValue<float>("resolution-target") > 0.0 ?
                    po.getValue<float>("resolution-target") * res :
                   5 * res;
    const float frad =
            po.getValue<float>("radius-feature") > 0.0 ?
                    po.getValue<float>("radius-feature") * res :
                    25 * res;
    const size_t cutoff = po.getValue<size_t>("cutoff");
    COVIS_ASSERT(cutoff > 0 && cutoff <= 100);

    // Estimation
    const size_t iterations = po.getValue<size_t>("iterations");
    const float inlierThreshold =
            (po.getValue<float>("inlier-threshold") > 0.0 ?
                    po.getValue<float>("inlier-threshold") * res :
                    5 * res);
    const float inlierFraction = po.getValue<float>("inlier-fraction");
    const bool noReestimate = po.getFlag("no-reestimate");
    const bool fullEvaluation = po.getFlag("full-evaluation");
    const bool prerejection = po.getFlag("prerejection");
    const float prerejectionSimilarty = po.getValue<float>("prerejection-similarity");
    const bool noOcclusionReasoning = po.getFlag("no-occlusion-reasoning");
    const int viewAxis = po.getValue<int>("view-axis");

    // Refinement
    const bool refine = po.getFlag("refine");
    const size_t icpIterations = po.getValue<size_t>("icp-iterations");

    // Misc.
    const bool visualize = po.getFlag("visualize");

    // Preprocess
    querySurf = filter::preprocess<PointT>(queryMesh, 1, true, far, res, nrad, po.getFlag("orient-query-normals"), false, true);
    targetSurf = filter::preprocess<PointT>(targetMesh, 1, true, far, res, nrad, false, false, true);
    COVIS_ASSERT(!querySurf->empty() && !targetSurf->empty());

    // Generate feature points
    queryCloud = filter::downsample<PointT>(querySurf, resQuery);
    targetCloud = filter::downsample<PointT>(targetSurf, resTarget);
    COVIS_ASSERT(!queryCloud->empty() && !targetCloud->empty());

    /*
     * Compute features
     */
    {
        const std::string feat = po.getValue("feature");
        core::ScopedTimer t("Features (" + feat + ")");
        queryFeat = feature::computeFeature<PointT>(feat, queryCloud, querySurf, frad);
        targetFeat = feature::computeFeature<PointT>(feat, targetCloud, targetSurf, frad);
        COVIS_MSG("\tComputed " << queryFeat.cols() << "/" << targetFeat.cols() << " query/target features");
    }

    /*
     * Match features
     */
    core::Correspondence::VecPtr corr;
    {
        core::ScopedTimer t("Feature matching");
        std::cout << "FUCK1" << '\n';
        corr = detect::computeRatioMatches(queryFeat, targetFeat);
        std::cout << "FUCK2" << '\n';
    }

    /*
     * Sort correspondences and cutoff at <cutoff> %
     */
    if(cutoff < 100) {
        core::sort(*corr);
        corr->resize(corr->size() * cutoff / 100);
    }

    /*
     * Execute RANSAC
     */
    core::Detection d;
    {
        d = pose_estimation_RANSAC(queryCloud, targetCloud, corr, 0.05, iterations);
    }
    // core::Detection d;
    // {
    //     core::ScopedTimer t("RANSAC");
    //     detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(targetCloud));
    //     fe->setOcclusionReasoning(!noOcclusionReasoning);
    //     fe->setViewAxis(viewAxis);
    //     if(noOcclusionReasoning)
    //         fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
    //     else
    //         fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);
    //
    //     detect::Ransac<PointT> ransac;
    //     ransac.setSource(queryCloud);
    //     ransac.setTarget(targetCloud);
    //     ransac.setCorrespondences(corr);
    //     ransac.setFitEvaluation(fe);
    //
    //     ransac.setIterations(iterations);
    //     ransac.setInlierThreshold(inlierThreshold);
    //     ransac.setInlierFraction(inlierFraction);
    //     ransac.setReestimatePose(!noReestimate);
    //     ransac.setFullEvaluation(fullEvaluation);
    //     ransac.setPrerejection(prerejection);
    //     ransac.setPrerejectionSimilarity(prerejectionSimilarty);
    //     ransac.setVerbose(true);
    //
    //     d = ransac.estimate();
    // }

    if(d) {
        if(refine) {
            core::ScopedTimer t("ICP");
            pcl::IterativeClosestPoint<PointT,PointT> icp;
            icp.setInputSource(queryCloud);
            icp.setInputTarget(targetCloud);
            icp.setMaximumIterations(icpIterations);
            icp.setMaxCorrespondenceDistance(inlierThreshold);
            pcl::PointCloud<PointT> tmp;
            icp.align(tmp, d.pose);
            if(icp.hasConverged()) {
                d.pose = icp.getFinalTransformation();
                d.rmse = icp.getFitnessScore();
            } else {
                COVIS_MSG_WARN("ICP failed!");
            }
        }

        // Print result and visualize
        COVIS_MSG(d);
        if(visualize)
            visu::showDetection(queryMesh, targetMesh, d.pose);
    } else {
        COVIS_MSG_WARN("RANSAC failed!");
    }

    return 0;
}



core::Detection pose_estimation_RANSAC(
    pcl::PointCloud<PointT>::Ptr object_cloud,
    pcl::PointCloud<PointT>::Ptr target_cloud,
    core::Correspondence::VecPtr correspondences,
    float threshold, size_t iterations)
{
    pcl::ScopeTime t("RANSAC");

    core::Detection result;

    pcl::search::KdTree<PointT> kdtree;
  	kdtree.setInputCloud(target_cloud);

    pcl::common::UniformGenerator<int> gen(0, correspondences->size() - 1);
    pcl::PointCloud<PointT>::Ptr object_aligned(new pcl::PointCloud<PointT>);
    float previous_penalty = std::numeric_limits<float>::max();

    for(size_t i = 0; i < iterations; i++) {

        // Get 3 random samples from object and target
        std::vector<int> object_indices(3);
        std::vector<int> target_indices(3);

        for(int j = 0; j < 3; j++) {
            const int index = gen.run();
            object_indices[j] = (*correspondences)[index].query;
            target_indices[j] = (*correspondences)[index].match[0];
        }

        // Threshold dissimilarity vector between points
        std::vector<double> delta(3);
        int max_obj_index = 0, min_obj_index = 0;
        int max_target_index = 0, min_target_index = 0;
        double max_dist_obj = 0, min_dist_obj = std::numeric_limits<double>::max();
        double max_dist_target = 0, min_dist_target = std::numeric_limits<double>::max();
        for (int j = 0; j < 3; j++) {
            double d_p = pcl::euclideanDistance(object_cloud->points[object_indices[j]],
                                                object_cloud->points[object_indices[(j + 1) % 3]]);
            double d_q = pcl::euclideanDistance(target_cloud->points[target_indices[j]],
                                                target_cloud->points[target_indices[(j + 1) % 3]]);
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
        }

        if (fabs(std::max({delta[0], delta[1], delta[2]})) > 0.20)
            continue;

        // If the index of shortest and furthest distance does not match continue
        if (max_obj_index != max_target_index || min_obj_index != min_target_index)
            continue;

        // Estimate transformation
        Eigen::Matrix4f transformation;
    	pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
        svd.estimateRigidTransformation(*object_cloud, object_indices, *target_cloud,
            object_indices, transformation);

        // Apply transformation
        pcl::transformPointCloud(*object_cloud, *object_aligned, transformation);

        // Validate
        std::vector<std::vector<int> > index;
        std::vector<std::vector<float> > distance;
        kdtree.nearestKSearch(*object_aligned, std::vector<int>(), 1, index, distance);

        // Compute inliers and RMSE
        size_t inliers = 0;
        float rmse = 0;
        for(size_t j = 0; j < distance.size(); j++) {
            if(distance[j][0] <= threshold) {
                inliers++;
                rmse += distance[j][0];
            }
        }
        rmse = sqrtf(rmse / inliers);

        // Evaluate a penalty function
        const float outlier_rate = 1.0f - float(inliers) / object_cloud->size();
        const float penalty = outlier_rate;


        // Update result
        if(penalty < previous_penalty) {
            std::cout << "\t-- Got a new model with " << inliers <<
                " inliers after " << i << " iterations!" << std::endl;
            previous_penalty = penalty;
            result.pose = transformation;
            result.rmse = rmse;
            result.inlierfrac = inliers;
            result.outlierfrac = outlier_rate;
            result.penalty = penalty;
        }
    }
    return result;
}

core::detection RANSAC2()
{
    // Instantiate pose sampler
    PoseSampler<PointT> poseSampler;
    poseSampler.setSource(_source);
    poseSampler.setTarget(_target);
    std::vector<int> sources(_sampleSize);
    std::vector<int> targets(_sampleSize);

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

    // Output detection(s)
    core::Detection result;
    _allDetections.clear();
    if(_correspondences->size() < _sampleSize)
        return result;

    // Start main loop
    for(size_t i = 0; i < _iterations; ++i) {

        // Create a sample from data
        const core::Correspondence::Vec maybeInliers =
                poseSampler.sampleCorrespondences(*_correspondences, _sampleSize);
        for(size_t j = 0; j < _sampleSize; ++j) {
            sources[j] = maybeInliers[j].query;
            targets[j] = maybeInliers[j].match[0];
        }

        // Apply prerejection
        // if(_prerejection)
        //     if(!poly.thresholdPolygon(sources, targets))
        //         continue;

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
