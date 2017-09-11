#include <covis/covis.h>
using namespace covis;

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Loaded point clouds and computed histogram features
pcl::PointCloud<PointT>::Ptr querySurf, targetSurf;
pcl::PointCloud<PointT>::Ptr queryCloud, targetCloud;
feature::MatrixT queryFeat, targetFeat;

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
        corr = detect::computeRatioMatches(queryFeat, targetFeat);
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
        core::ScopedTimer t("RANSAC");
        detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(targetCloud));
        fe->setOcclusionReasoning(!noOcclusionReasoning);
        fe->setViewAxis(viewAxis);
        if(noOcclusionReasoning)
            fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
        else
            fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);

        detect::Ransac<PointT> ransac;
        ransac.setSource(queryCloud);
        ransac.setTarget(targetCloud);
        ransac.setCorrespondences(corr);
        ransac.setFitEvaluation(fe);

        ransac.setIterations(iterations);
        ransac.setInlierThreshold(inlierThreshold);
        ransac.setInlierFraction(inlierFraction);
        ransac.setReestimatePose(!noReestimate);
        ransac.setFullEvaluation(fullEvaluation);
        ransac.setPrerejection(prerejection);
        ransac.setPrerejectionSimilarity(prerejectionSimilarty);
        ransac.setVerbose(true);

        d = ransac.estimate();
    }

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
