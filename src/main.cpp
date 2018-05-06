 /**  Todo list:
  * TODO Try different ICP on hinterstoisser
  * TODO Test on T-less
  * TODO Add centroid distance threshold to RANSAC
  * TODO Try to make a correspondence removal check by making a plane based on normal and looking at the number of surface points
  * TODO Look into finding poses using the height map (heatmap) and sampling the points with the same height
  * TODO Remove line 261 from dataset_loader.cpp to load datasets without poses
  */

// Guide on how run data on server
    // Mount: sudo sshfs -o allow_other msteenberg@sdur-2.sandbox.tek.sdu.dk:/ /media/ztaal/sdur-2
    // Connect: vglconnect -s msteenberg@sdur-2.sandbox.tek.sdu.dk
    // Navigate: cd /workspace/sixd_toolkit/tools
    // Run: vglrun python eval_calc_errors.py
    // Run: vglrun python eval_loc.py
    // Run: vglrun python vis_sixd_poses.py
    // Inspect: cd /home/msteenberg/data
    // Unmount: fusermount /media/ztaal/sdur-2 -u
// Notes
    // root folder needs to have lower case dataset name (e.g. data_tejani)
    // eval_calc_errors.py && vis_sixd_poses.py
        // Change result_base og result_path
    // eval_loc.py
        // Change error_bpath
        // Set require_all_errors = false

// Get errors:
// cat errors_05.yml | grep -E "0: 0.[3-9][0-9]*|0: 1." | sed -r 's/.{10}//' | sed 's/,.*//'

// Hintertoisser Error 05
//    9,   13,   36,   69,  161,  179,  219,  304,  318,  335,  450,  458,  471,  504,  552,
//  628,  664,  665,  679,  827,  840,  951,  998, 1016, 1050, 1051, 1180, 1182

// Errors: 38

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
    // po.addOption("radius-normal", 'n', 5, "normal estimation radius in mr (<= 0 means two resolution units)"); // UWA
    po.addOption("radius-normal", 'n', 15, "normal estimation radius in mr (<= 0 means two resolution units)"); // Tejani
    // po.addOption("radius-normal", 'n', 6, "normal estimation radius in mr (<= 0 means two resolution units)"); // Hintertoisser // 6 BEST
    po.addFlag('o', "orient-query-normals", "ensure consistent normal orientation for the query model");

    // Features and matching
    // po.addOption("feature", "si", "choose which feature to use from this list: " + feature::FeatureNames); // si
    po.addOption("feature", "ppfhistfull", "choose which feature to use from this list: " + feature::FeatureNames); // ppf
    po.addOption("resolution-query", 5, "resolution of query features in mr (<= 0 for five resolution units)"); // Hintertoisser // 5 BEST
    po.addOption("resolution-target", 5, "resolution of target features in mr (<= 0 for five resolution units)"); // Hintertoisser // 5 BEST
    // po.addOption("resolution-query", 5, "resolution of query features in mr (<= 0 for five resolution units)"); // Tejani
    // po.addOption("resolution-target", 5, "resolution of target features in mr (<= 0 for five resolution units)"); // Tejani
    // po.addOption("radius-feature", 'f', 25, "feature estimation radius (<= 0 means 25 resolution units)"); // UWA
    po.addOption("radius-feature", 'f', 50, "feature estimation radius (<= 0 means 25 resolution units)"); // Tejani
    // po.addOption("radius-feature", 'f', 0.3, "feature estimation radius (<= 0 means 25 resolution units)"); // Hintertoisser // 0.3 BEST
    // po.addOption("radius-feature", 'f', 0.3, "feature estimation radius (<= 0 means 25 resolution units)"); // T-Less // 0.3 BEST
    // po.addOption("cutoff", 50, "use the <cutoff> % best L2 ratio correspondences for RANSAC"); // UWA
    po.addOption("cutoff", 100, "use the <cutoff> % best L2 ratio correspondences for RANSAC"); // Tejani
    po.addOption("object-scale", 1, "scale of object (1 is default)");
    po.addOption("sample-size", 3, "sample size used for RANSAC");

    // Estimation
    po.addOption("iterations", 'i', 10000, "RANSAC iterations");
    // po.addOption("inlier-threshold", 't', 5, "RANSAC inlier threshold (<= 0 for infinite)"); // UWA
    // po.addOption("inlier-threshold", 't', 8, "RANSAC inlier threshold (<= 0 for infinite)"); // Tejani
    po.addOption("inlier-threshold", 't', 8, "Inlier threshold (<= 0 for infinite)"); // Hintertoisser // 8 BEST
    // po.addOption("inlier-fraction", 'a', 0.0, "RANSAC inlier fraction required for accepting a pose hypothesis"); // UWA
    po.addOption("inlier-fraction", 'a', 0.05, "RANSAC inlier fraction required for accepting a pose hypothesis"); // Tejani
    // po.addOption("inlier-fraction", 'a', 0.0, "RANSAC inlier fraction required for accepting a pose hypothesis"); // Hintertoisser
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

    // if (verbose)
    //     po.print();

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
        class covis::detect::Benchmark_Sixd bt;

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

        if( po.getFlag("benchmark-sixd") ) {
            // Benchmark Tejani variables
            bt.setRootPath( po.getValue("root-path") );
            bt.setObjectDir( po.getValue("object-dir") );
            bt.setSceneDir( po.getValue("scene-dir") );
            bt.setPoseFile( po.getValue("yml-file") );
            bt.setBenchmarkFile( po.getValue("benchmark-file") );
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

            // ransac.setPrerejectionD( true );
            // ransac.setPrerejectionG( true );
            // ransac.setPrerejectionD( false );
            // ransac.setPrerejectionG( false );
            // bt.run( &ransac, "Ransac" );
            bt.run( &posePrior, "Pose Prior" );
            bt.printResults();
            if (po.getFlag("save"))
                bt.savePoses(po.getValue("save-dir"));
                // bt.savePoses("../data_tejani/");
                // bt.savePoses("../data_hinterstoisser/");
                // bt.savePoses("../ransac_tejani/");
            bt.clearResults();
        }
     }

    return 0;
}
