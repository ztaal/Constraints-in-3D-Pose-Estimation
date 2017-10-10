// Covis
#include <covis/covis.h>
using namespace covis;

// Ransac
#include "../headers/ransac.hpp"

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

int main( int argc, const char** argv )
{
    printf( " -- Initialized Program\n\n" );

    // Setup program options
    core::ProgramOptions po;

    po.addPositional("root-path", "root path of your dataset");

    // Surfaces and normals
    po.addOption("resolution", 'r', 1, "downsample point clouds to this resolution (<= 0 for disabled)");
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
    po.addOption("iterations", 'i', 10000, "RANSAC iterations");
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

    // Benchmark
    po.addFlag('b', "benchmark", "benchmark ransac");
    po.addOption("object-dir", 'o', "objects", "subdirectory for the object models");
    po.addOption("scene-dir", 's', "scenes", "subdirectory for the scene models");
    po.addOption("pose-dir", 'p', "ground_truth", "subdirectory for the ground truth pose models");
    po.addOption("object-ext", ".ply", "object file extension");
    po.addOption("scene-ext", ".ply", "scene file extension");
    po.addOption("pose-ext", ".xf", "pose file extension");
    po.addOption("pose-sep", "-", "pose file separator");
    po.addOption("object-regex", "", "set this option to use a regular expression search when collecting object files");
    po.addOption("scene-regex", "", "set this option to use a regular expression search when collecting scene files");

    // Parse
    if(!po.parse(argc, argv))
        return 1;
    po.print();

    // Estimation
    float res = po.getValue<float>("resolution");
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

    /*
     * Benchmark RANSAC
     */
    core::Detection d;
    {
        // detect::FitEvaluation<PointT>::Ptr fe(new detect::FitEvaluation<PointT>(targetCloud));
        // fe->setOcclusionReasoning(!noOcclusionReasoning);
        // fe->setViewAxis(viewAxis);
        // if(noOcclusionReasoning)
        //     fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS);
        // else
        //     fe->setPenaltyType(detect::FitEvaluation<PointT>::INLIERS_OUTLIERS_RMSE);

        ransac ransac;

        // ransac variables
        ransac.setIterations( iterations );
        // ransac.setFitEvaluation( fe );
        ransac.setInlierThreshold( inlierThreshold );
        ransac.setInlierFraction( inlierFraction );
        ransac.setReestimatePose( !noReestimate );
        ransac.setFullEvaluation( fullEvaluation );
        ransac.setPrerejection( prerejection );
        ransac.setOcclusionReasoning( noOcclusionReasoning );
        ransac.setPrerejectionSimilarity( prerejectionSimilarty );

        // Benchmark variables
        ransac.setRootPath( po.getValue("root-path") );
        ransac.setObjectDir( po.getValue("object-dir") );
        ransac.setSceneDir( po.getValue("scene-dir") );
        ransac.setPoseDir( po.getValue("pose-dir") );
        ransac.setObjExt( po.getValue("object-ext") );
        ransac.setSceneExt( po.getValue("scene-ext") );
        ransac.setPoseExt( po.getValue("pose-ext") );
        ransac.setPoseSep( po.getValue("pose-sep") );


        if( po.getFlag("benchmark") ) {
            ransac.benchmark();
        }

         //     ransac.setVerbose(true);
     }

    //  if(d) {
    //      if(refine) {
    //          core::ScopedTimer t("ICP");
    //          pcl::IterativeClosestPoint<PointT,PointT> icp;
    //          icp.setInputSource(queryCloud);
    //          icp.setInputTarget(targetCloud);
    //          icp.setMaximumIterations(icpIterations);
    //          icp.setMaxCorrespondenceDistance(inlierThreshold);
    //          pcl::PointCloud<PointT> tmp;
    //          icp.align(tmp, d.pose);
    //          if(icp.hasConverged()) {
    //              d.pose = icp.getFinalTransformation();
    //              d.rmse = icp.getFitnessScore();
    //          } else {
    //              COVIS_MSG_WARN("ICP failed!");
    //          }
    //      }
     //
    //      // Print result and visualize
    //      COVIS_MSG(d);
    //      if(visualize)
    //          visu::showDetection(queryMesh, targetMesh, d.pose);
    //  } else {
    //      COVIS_MSG_WARN("RANSAC failed!");
    //  }

    return 0;
}
