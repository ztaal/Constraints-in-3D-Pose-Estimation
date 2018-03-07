
 /**  Todo list:
  * TODO Run tests with pose priors with one point
  * TODO Create a prerejection check that if the height difference between the corr and
         the plane in both the scene and object is too different discard it
  * TODO Pose priors with two points
  * TODO Look into finding poses using the height map (heatmap) and sampling the points with the same height
  * TODO Remove line 261 from dataset_loader.cpp to load datasets without poses
  */

// Errors 01
    // Off Position
        //  41,  47,  55,  88, 113, 124, 126, 127, 130, 131, 134, 135, 136, 137, 138,
        // 139, 140, 141, 142, 147, 148, 151, 153, 154, 156, 157, 158, 159, 164, 168
        // 170, 171, 172, 174, 175, 176, 178, 179, 183, 185, 186, 187, 188, 189, 190
        // 191, 195, 197

    // Failed test 2
        //  38,  41,  91, 124, 127, 130, 131, 134, 135, 136, 140, 141, 149, 151, 154,
        // 156, 157, 158, 159, 164, 170, 171, 172, 174, 175, 176, 177, 178, 179, 186,
        // 188, 189, 190, 191,
// Errors 02
    // None
// Errors 03
    // Off Position
        //  16,  84,  87,  89,  90,  91,  92,  93,  94,  95,  96, 101, 121, 123, 124, 126,
        // 128, 129,
    // wall
        // 105

// Covis
#include <covis/covis.h>
using namespace covis;

// Includes
#include "../headers/Ransac.hpp"
#include "../headers/Benchmark.hpp"
#include "../headers/Correspondence.hpp"
#include "../headers/PosePrior.hpp"
#include "../headers/yml_loader.hpp"
#include "../headers/Benchmark_Tejani.hpp"

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
    po.addOption("radius-normal", 'n', 15, "normal estimation radius in mr (<= 0 means two resolution units)");
    // po.addOption("radius-normal", 'n', 10, "normal estimation radius in mr (<= 0 means two resolution units)");
    // po.addOption("radius-normal", 'n', 5, "normal estimation radius in mr (<= 0 means two resolution units)"); // TODO Change back
    po.addFlag('o', "orient-query-normals", "ensure consistent normal orientation for the query model");

    // Features and matching
    po.addOption("feature", "ppfhistfull", "choose which feature to use from this list: " + feature::FeatureNames); // Other feature: ppfhistfull
    // po.addOption("feature", "si", "choose which feature to use from this list: " + feature::FeatureNames); // Other feature: ppfhistfull
    // po.addOption("resolution-query", 20, "resolution of query features in mr (<= 0 for five resolution units)");
    // po.addOption("resolution-target", 20, "resolution of target features in mr (<= 0 for five resolution units)");
    po.addOption("resolution-query", 5, "resolution of query features in mr (<= 0 for five resolution units)"); // TODO Change back
    po.addOption("resolution-target", 5, "resolution of target features in mr (<= 0 for five resolution units)"); // TODO Change back
    po.addOption("radius-feature", 'f', 50, "feature estimation radius (<= 0 means 25 resolution units)");
    // po.addOption("radius-feature", 'f', 25, "feature estimation radius (<= 0 means 25 resolution units)");
    po.addOption("cutoff", 50, "use the <cutoff> % best L2 ratio correspondences for RANSAC");
    po.addOption("object-scale", 1, "scale of object (1 is default)");
    po.addOption("sample-size", 3, "sample size used for RANSAC");

    // Estimation
    po.addOption("iterations", 'i', 10000, "RANSAC iterations");
    // po.addOption("inlier-threshold", 't', 0, "RANSAC inlier threshold (<= 0 for infinite)");
    // po.addOption("inlier-threshold", 't', 5, "RANSAC inlier threshold (<= 0 for infinite)"); // TODO Change back
    po.addOption("inlier-threshold", 't', 10, "RANSAC inlier threshold (<= 0 for infinite)"); // TODO Change back (best)
    // po.addOption("inlier-fraction", 'a', 0.0, "RANSAC inlier fraction required for accepting a pose hypothesis");
    po.addOption("inlier-fraction", 'a', 0.05, "RANSAC inlier fraction required for accepting a pose hypothesis"); // TODO Change back
    po.addFlag('u', "full-evaluation", "enable full pose evaluation during RANSAC, otherwise only the existing feature matches are used during verification");
    po.addFlag('d', "prerejectionD", "enable dissimilarity prerejection during RANSAC");
    po.addFlag('g', "prerejectionG", "enable geometric prerejection during RANSAC");
    po.addOption("prerejection-similarity", 's', 0.9, "prerejection similarity threshold in [0,1]");
    po.addFlag('c', "no-occlusion-reasoning", "disable occlusion reasoning during pose hypothesis evaluation");
    po.addOption("view-axis", 'x', 2, "if occlusion reasoning is on (default), assume axis x, y, or z (0, 1 or 2) to point in the direction of the view");

    // Misc.
    po.addFlag('v', "verbose", "show additional information");
    po.addFlag('z', "visualize", "show additional results");

    // Ransac
    po.addFlag('r', "ransac", "ransac");

    // Pose prior
    po.addFlag('p', "pose_prior", "pose_prior");
    po.addOption("query", 'q', "models", "mesh or point cloud file for query model");
    po.addOption("target", 't', "scenes", "mesh or point cloud file for target model");
    po.addOption("yml-file", 'y', "ground_truth", "path to yml file contaning ground truth poses");
    po.addFlag('t', "benchmark-tejani", "benchmark tejani");

    // Benchmark
    po.addFlag('b', "benchmark", "benchmark ransac");
    po.addOption("object-dir", 'o', "objects", "subdirectory for the object models");
    po.addOption("scene-dir", 's', "scenes", "subdirectory for the scene models");
    po.addOption("pose-dir", 'p', "ground_truth", "subdirectory for the ground truth pose models");
    po.addOption("object-ext", ".ply", "object file extension");
    po.addOption("scene-ext", ".ply", "scene file extension");
    po.addOption("pose-ext", ".xf", "pose file extension");
    po.addOption("pose-sep", "-", "pose file separator");

    // Parse
    if(!po.parse(argc, argv))
        return 1;

    // Misc.
    const bool verbose = po.getFlag("verbose");

    if (verbose)
        po.print();

    // Estimation
    const int sampleSize = po.getValue<int>("sample-size");
    float res = po.getValue<float>("resolution");
    const size_t iterations = po.getValue<size_t>("iterations");
    const float inlierThreshold = (po.getValue<float>("inlier-threshold") > 0.0 ?
                    po.getValue<float>("inlier-threshold") * res : 5 * res);
    const float inlierFraction = po.getValue<float>("inlier-fraction");
    const bool fullEvaluation = po.getFlag("full-evaluation");
    const bool prerejectionD = po.getFlag("prerejectionD");
    const bool prerejectionG = po.getFlag("prerejectionG");
    const float prerejectionSimilarty = po.getValue<float>("prerejection-similarity");
    const bool noOcclusionReasoning = po.getFlag("no-occlusion-reasoning");
    const int viewAxis = po.getValue<int>("view-axis");

    /*
     * Benchmark RANSAC
     */
    core::Detection d;
    {
        class covis::detect::correspondence correspondence;
        class covis::detect::posePrior posePrior;
        class covis::detect::ransac ransac;
        class covis::detect::Benchmark benchmark;
        class covis::detect::Benchmark_Tejani bt;

        // Correspondence variables
        correspondence.setResolution( po.getValue<float>("resolution") );
        correspondence.setObjectScale( po.getValue<float>("object-scale") );
        correspondence.setFar( po.getValue<float>("far") );
        correspondence.setRadiusNormal( po.getValue<float>("radius-normal") );
        correspondence.setResolutionQuery( po.getValue<float>("resolution-query") );
        correspondence.setResolutionTarget( po.getValue<float>("resolution-target") );
        correspondence.setRadiusFeature( po.getValue<float>("radius-feature") );
        correspondence.setCutoff( po.getValue<size_t>("cutoff") );
        correspondence.setFeature( po.getValue("feature") );
        correspondence.setVerbose( verbose );

        // correspondence.compute( po.getValue("query"), po.getValue("target") );
        // CloudT::Ptr queryCloud = correspondence.getQuery();
        // CloudT::Ptr targetCloud = correspondence.getTarget();
        // covis::core::Correspondence::VecPtr corr = correspondence.getCorrespondence();

        if( po.getFlag("pose_prior") ) {
            posePrior.setInlierThreshold( inlierThreshold );
            posePrior.setInlierFraction( 0.008 );
            // posePrior.setInlierFraction( 0.008 ); // Best so far
            // posePrior.setInlierFraction( 0.02 );
            // posePrior.setInlierFraction( inlierFraction );
            posePrior.setViewAxis( viewAxis );
            posePrior.setVerbose( verbose );

            // Pose Prior
            // posePrior.setSource( queryCloud );
            // posePrior.setTarget( targetCloud );
            // posePrior.setCorrespondences( corr );
            // posePrior.setInlierFraction( 0.01 );

            // {
            //     core::ScopedTimer t("Pose priors");
            //     d = posePrior.estimate();
            // }
            // if(d) {
            //     // Visualize
            //     if (verbose)
            //         COVIS_MSG(d);
            //     if( po.getFlag("visualize") ) {
            //         visu::showDetection<PointT>( queryCloud, targetCloud, d.pose );
            //     }
            // } else {
            //     COVIS_MSG_WARN("Pose priors failed!");
            // }
        }

        if( po.getFlag("ransac") ) {

            // Ransac variables
            ransac.setIterations( iterations );
            ransac.setSampleSize( sampleSize );
            ransac.setInlierThreshold( inlierThreshold );
            ransac.setInlierFraction( inlierFraction );
            ransac.setFullEvaluation( fullEvaluation );
            ransac.setViewAxis( viewAxis );
            ransac.setPrerejectionD( prerejectionD );
            ransac.setPrerejectionG( prerejectionG );
            ransac.setOcclusionReasoning( noOcclusionReasoning );
            ransac.setPrerejectionSimilarity( prerejectionSimilarty );
            ransac.setVerbose( verbose );

            // Ransac
            // ransac.setSource( queryCloud );
            // ransac.setTarget( targetCloud );
            // ransac.setCorrespondences( corr );
            // ransac.setInlierFraction( 0 );

            // {
            //     core::ScopedTimer t("Ransac");
            //     d = ransac.estimate();
            // }
            //
            // if(d) {
            //     // Visualize
            //     if (verbose)
            //         COVIS_MSG(d);
            //     if( po.getFlag("visualize") ) {
            //         visu::showDetection<PointT>( queryCloud, targetCloud, d.pose );
            //     }
            // } else {
            //     COVIS_MSG_WARN("RANSAC failed!");
            // }
        }

        if( po.getFlag("benchmark") ) {
            // Benchmark variables
            benchmark.setRootPath( po.getValue("root-path") );
            benchmark.setObjectDir( po.getValue("object-dir") );
            benchmark.setSceneDir( po.getValue("scene-dir") );
            benchmark.setPoseDir( po.getValue("pose-dir") );
            benchmark.setObjExt( po.getValue("object-ext") );
            benchmark.setSceneExt( po.getValue("scene-ext") );
            benchmark.setPoseExt( po.getValue("pose-ext") );
            benchmark.setPoseSep( po.getValue("pose-sep") );

            benchmark.setResolution( po.getValue<float>("resolution") );
            benchmark.setObjectScale( po.getValue<float>("object-scale") );
            benchmark.setFar( po.getValue<float>("far") );
            benchmark.setRadiusNormal( po.getValue<float>("radius-normal") );
            benchmark.setResolutionQuery( po.getValue<float>("resolution-query") );
            benchmark.setResolutionTarget( po.getValue<float>("resolution-target") );
            benchmark.setRadiusFeature( po.getValue<float>("radius-feature") );
            benchmark.setCutoff( po.getValue<size_t>("cutoff") );
            benchmark.setFeature( po.getValue("feature") );
            benchmark.setVerbose( verbose );
            // benchmark.setBenchmarkPrerejection( true );

            // for ( size_t i = 0; i < 1; i++ ) {
                ransac.setPrerejectionD( true );
                ransac.setPrerejectionG( true );
                benchmark.run( &ransac, "Base case" );

                // ransac.setCorrection( true );
                // benchmark.run( &ransac, "Correction" );

                benchmark.printResults();
                // benchmark.saveResults("../test_data/");
                // benchmark.printPrerejectionResults();
                benchmark.clearResults();
                benchmark.generateNewSeed();
            // }
        }

        if( po.getFlag("benchmark-tejani") ) {
            // Benchmark Tejani variables
            bt.setRootPath( po.getValue("root-path") );
            bt.setObjectDir( po.getValue("object-dir") );
            bt.setSceneDir( po.getValue("scene-dir") );
            bt.setPosePath( po.getValue("yml-file") );
            bt.setObjExt( po.getValue("object-ext") );
            bt.setSceneExt( po.getValue("scene-ext") );

            bt.setResolution( po.getValue<float>("resolution") );
            bt.setObjectScale( po.getValue<float>("object-scale") );
            bt.setFar( po.getValue<float>("far") );
            bt.setRadiusNormal( po.getValue<float>("radius-normal") );
            bt.setResolutionQuery( po.getValue<float>("resolution-query") );
            bt.setResolutionTarget( po.getValue<float>("resolution-target") );
            bt.setRadiusFeature( po.getValue<float>("radius-feature") );
            bt.setCutoff( po.getValue<size_t>("cutoff") );
            bt.setFeature( po.getValue("feature") );
            bt.setVerbose( verbose );

            // for ( size_t i = 0; i < 1; i++ ) {
                bt.run( &posePrior, "Pose Prior" );
                bt.printResults();
                bt.clearResults();
            // }
        }
     }

    return 0;
}
