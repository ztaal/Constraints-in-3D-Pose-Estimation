
 /**  Todo list:
  * TODO Create a start up period where no corr are removed only votes are cast
  * TODO Create threshold for when to stop ransac based on the angle of the corr (when it is low we stop)
  * TODO Start working on pose priors
  */

// Covis
#include <covis/covis.h>
using namespace covis;

// Ransac
#include "../headers/Ransac.hpp"
#include "../headers/Benchmark.hpp"

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
    po.addOption("sample-size", 3, "sample size used for RANSAC");

    // Estimation
    po.addOption("iterations", 'i', 10000, "RANSAC iterations");
    po.addOption("inlier-threshold", 't', 5, "RANSAC inlier threshold (<= 0 for infinite)");
    po.addOption("inlier-fraction", 'a', 0.0, "RANSAC inlier fraction required for accepting a pose hypothesis");
    // po.addOption("inlier-fraction", 'a', 0.05, "RANSAC inlier fraction required for accepting a pose hypothesis"); // TODO Change back
    po.addFlag('e', "no-reestimate", "disable re-estimation of pose hypotheses using consensus set during RANSAC");
    po.addFlag('u', "full-evaluation", "enable full pose evaluation during RANSAC, otherwise only the existing feature matches are used during verification");
    po.addFlag('d', "prerejectionD", "enable prerejection during RANSAC");
    po.addFlag('g', "prerejectionG", "enable prerejection during RANSAC");
    po.addOption("prerejection-similarity", 's', 0.9, "prerejection similarity threshold in [0,1]");
    po.addFlag('c', "no-occlusion-reasoning", "disable occlusion reasoning during pose hypothesis evaluation");
    po.addOption("view-axis", 'x', 2, "if occlusion reasoning is on (default), assume axis x, y, or z (0, 1 or 2) to point in the direction of the view");

    // Refinement
    po.addFlag("refine", "apply pose refinement of the RANSAC result using ICP");
    po.addOption("icp-iterations", 25, "number of ICP iterations");

    // Misc.
    po.addFlag('v', "verbose", "show additional information");
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

    // Parse
    if(!po.parse(argc, argv))
        return 1;
    po.print();

    // Misc.
    const bool verbose = po.getFlag("verbose");

    // Estimation
    const int sampleSize = po.getValue<int>("sample-size");
    float res = po.getValue<float>("resolution");
    const size_t iterations = po.getValue<size_t>("iterations");
    const float inlierThreshold =
            (po.getValue<float>("inlier-threshold") > 0.0 ?
                    po.getValue<float>("inlier-threshold") * res :
                    5 * res);
    const float inlierFraction = po.getValue<float>("inlier-fraction");
    const bool noReestimate = po.getFlag("no-reestimate");
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
        class covis::detect::ransac ransac;

        // ransac variables
        ransac.setIterations( iterations );
        ransac.setSampleSize( sampleSize );
        ransac.setInlierThreshold( inlierThreshold );
        ransac.setInlierFraction( inlierFraction );
        ransac.setReestimatePose( !noReestimate );
        ransac.setFullEvaluation( fullEvaluation );
        ransac.setViewAxis( viewAxis );
        ransac.setPrerejectionD( prerejectionD );
        ransac.setPrerejectionG( prerejectionG );
        ransac.setOcclusionReasoning( noOcclusionReasoning );
        ransac.setPrerejectionSimilarity( prerejectionSimilarty );
        ransac.setVerbose( verbose );

        // Benchmark variables
        covis::detect::Benchmark benchmark;
        benchmark.setRootPath( po.getValue("root-path") );
        benchmark.setObjectDir( po.getValue("object-dir") );
        benchmark.setSceneDir( po.getValue("scene-dir") );
        benchmark.setPoseDir( po.getValue("pose-dir") );
        benchmark.setObjExt( po.getValue("object-ext") );
        benchmark.setSceneExt( po.getValue("scene-ext") );
        benchmark.setPoseExt( po.getValue("pose-ext") );
        benchmark.setPoseSep( po.getValue("pose-sep") );
        benchmark.setVerbose( verbose );
        // benchmark.setBenchmarkPrerejection( true );

        if( po.getFlag("benchmark") ) {
            for ( size_t i = 0; i < 1; i++ ) {
                ransac.setPrerejectionD( false );
                ransac.setPrerejectionG( false );
                benchmark.run( &ransac, "Base case " + std::to_string(i) );

                ransac.setPrerejectionG( true );
                benchmark.run( &ransac, "Geometric " + std::to_string(i) );
                ransac.setPrerejectionG( false );
                ransac.setPrerejectionG2( true );
                benchmark.run( &ransac, "Geometric2 " + std::to_string(i) );
                ransac.setPrerejectionG2( false );

                ransac.setPrerejectionD( true );
                // ransac.setPrerejectionG( false );
                // ransac.setPrerejectionSimilarity( 0.95 );
                // benchmark.run( &ransac, "0.95 Dissimilarity " + std::to_string(i) );
                // ransac.setPrerejectionG( true );
                // benchmark.run( &ransac, "0.95 Both " + std::to_string(i) );

                ransac.setPrerejectionG( false );
                ransac.setPrerejectionSimilarity( 0.9 );
                benchmark.run( &ransac, "0.9 Dissimilarity " + std::to_string(i) );
                // ransac.setPrerejectionG( true );
                // benchmark.run( &ransac, "0.9 Both " + std::to_string(i) );

                // ransac.setPrerejectionG( false );
                // ransac.setPrerejectionSimilarity( 0.8 );
                // benchmark.run( &ransac, "0.8 Dissimilarity " + std::to_string(i) );
                // ransac.setPrerejectionG( true );
                // benchmark.run( &ransac, "0.8 Both " + std::to_string(i) );
                //
                // ransac.setPrerejectionG( false );
                // ransac.setPrerejectionSimilarity( 0.7 );
                // benchmark.run( &ransac, "0.7 Dissimilarity " + std::to_string(i) );
                // ransac.setPrerejectionG( true );
                // benchmark.run( &ransac, "0.7 Both " + std::to_string(i) );

                // ransac.setCorrection( true );
                // benchmark.run( &ransac, "Correction" );

                benchmark.printResults();
                benchmark.saveResults("../test_data/");
                // benchmark.printPrerejectionResults();
                benchmark.clearResults();
                benchmark.generateNewSeed();
            }
        }
     }

    return 0;
}
