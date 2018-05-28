// Covis
#include <covis/covis.h>
using namespace covis;

// Includes
#include "../headers/Ransac.hpp"
#include "../headers/Benchmark.hpp"
#include "../headers/Correspondence.hpp"
#include "../headers/PosePrior.hpp"
#include "../headers/yml_loader.hpp"
#include "../headers/Benchmark_Sixd.hpp"

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

int main( int argc, const char** argv )
{
    core::ScopedTimer t("Benchmark");
    printf( " -- Initialized Program\n\n" );

    // Set pcl log level
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // Setup program options
    core::ProgramOptions po;

    po.addPositional("root-path", "root path of your dataset");

    // Surfaces and normals
    po.addOption("resolution", 'r', 1, "downsample point clouds to this resolution (<= 0 for disabled)");
    po.addOption("far", -1, "do not consider target points beyond this depth (<= 0 for disabled)");
    po.addOption("radius-normal", 'n', 6, "normal estimation radius in mr (<= 0 means two resolution units)");
    po.addFlag('o', "orient-query-normals", "ensure consistent normal orientation for the query model");

    // Features and matching
    po.addOption("feature", "ppfhistfull", "choose which feature to use from this list: " + feature::FeatureNames);
    po.addOption("resolution-query", 5, "resolution of query features in mr (<= 0 for five resolution units)");
    po.addOption("resolution-target", 5, "resolution of target features in mr (<= 0 for five resolution units)");
    po.addOption("radius-feature", 'f', 0.3, "feature estimation radius (<= 0 means 25 resolution units)");
    po.addOption("cutoff", 100, "use the <cutoff> % best L2 ratio correspondences for RANSAC");
    po.addOption("object-scale", 1, "scale of object (1 is default)");
    po.addOption("sample-size", 3, "sample size used for RANSAC");

    // Estimation
    po.addOption("iterations", 'i', 100000, "RANSAC iterations");
    po.addOption("inlier-threshold", 't', 8, "RANSAC inlier threshold (<= 0 for infinite)");
    po.addOption("inlier-fraction", 'a', 0.0, "RANSAC inlier fraction required for accepting a pose hypothesis");
    po.addFlag('u', "full-evaluation", "enable full pose evaluation during RANSAC, otherwise only the existing feature matches are used during verification");
    po.addFlag('d', "prerejectionD", "enable dissimilarity prerejection during RANSAC");
    po.addFlag('g', "prerejectionG", "enable geometric prerejection during RANSAC");
    po.addOption("prerejection-similarity", 's', 0.9, "prerejection similarity threshold in [0,1]");
    po.addFlag('c', "no-occlusion-reasoning", "disable occlusion reasoning during pose hypothesis evaluation");
    po.addOption("view-axis", 'x', 2, "if occlusion reasoning is on (default), assume axis x, y, or z (0, 1 or 2) to point in the direction of the view");

    // Misc.
    po.addFlag('v', "verbose", "show additional information");
    po.addFlag('z', "visualize", "vizualize transformation");
    po.addFlag('s', "save", "save poses");
    po.addOption("save-dir", "save directory");

    // Ransac
    po.addFlag('r', "ransac", "ransac");

    // Pose prior
    po.addFlag('p', "pose_prior", "pose_prior");
    po.addOption("query", 'q', "models", "mesh or point cloud file for query model");
    po.addOption("target", 't', "scenes", "mesh or point cloud file for target model");
    po.addOption("yml-file", 'y', "ground_truth", "path to yml file contaning ground truth poses");
    po.addOption("benchmark-file", 'b', "benchmark", "path to yml file contaning benchmark indices");
    po.addFlag('t', "benchmark-sixd", "benchmark sixd");

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
        class covis::detect::Benchmark_Sixd bs;

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

        if( po.getFlag("pose_prior") ) {
            posePrior.setInlierThreshold( inlierThreshold );
            posePrior.setViewAxis( viewAxis );
            posePrior.setVerbose( verbose );
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

            ransac.setCorrection( true );
            ransac.setPrerejectionD( true );
            ransac.setPrerejectionG( true );
        }

        if( po.getFlag("benchmark-sixd") ) {
            // Benchmark Tejani variables
            bs.setRootPath( po.getValue("root-path") );
            bs.setObjectDir( po.getValue("object-dir") );
            bs.setSceneDir( po.getValue("scene-dir") );
            bs.setPoseFile( po.getValue("yml-file") );
            bs.setBenchmarkFile( po.getValue("benchmark-file") );
            bs.setObjExt( po.getValue("object-ext") );
            bs.setSceneExt( po.getValue("scene-ext") );

            bs.setResolution( po.getValue<float>("resolution") );
            bs.setObjectScale( po.getValue<float>("object-scale") );
            bs.setFar( po.getValue<float>("far") );
            bs.setRadiusNormal( po.getValue<float>("radius-normal") );
            bs.setResolutionQuery( po.getValue<float>("resolution-query") );
            bs.setResolutionTarget( po.getValue<float>("resolution-target") );
            bs.setRadiusFeature( po.getValue<float>("radius-feature") );
            bs.setCutoff( po.getValue<size_t>("cutoff") );
            bs.setFeature( po.getValue("feature") );

            // bs.run( &ransac, "Ransac" );
            bs.run( &posePrior, "Pose Prior" );
            bs.printResults();
            if (po.getFlag("save"))
                bs.savePoses(po.getValue("save-dir"));
            bs.clearResults();
        }
     }

    return 0;
}
