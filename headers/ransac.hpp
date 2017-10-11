#pragma once

// Covis
#include <covis/covis.h>
using namespace covis;

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Point and feature types
typedef pcl::PointCloud<PointT> CloudT;

struct benchmarkResult
{
    std::string name;
    std::vector<std::vector<core::Detection> > d;
    std::vector<std::vector<double> > time;
    double totalTime;
};

class ransac
{
    public:
        ransac();
        ~ransac();

        // Ransac
        void setSource( pcl::PointCloud<PointT>::Ptr _source );
        void setTarget( pcl::PointCloud<PointT>::Ptr _target );
        void setCorrespondences( core::Correspondence::VecPtr corr );
        void setIterations( size_t _iterations );
        void setSampleSize( size_t _sampleSize );
        void setInlierThreshold( float _inlierThreshold );
        void setInlierFraction( float _inlierFraction );
        void setReestimatePose( bool _reestimatePose );
        void setFullEvaluation( bool _fullEvaluation );
        void setPrerejection( bool _prerejection );
        void setPrerejectionSimilarity( float _prerejectionSimilarty );
        void setOcclusionReasoning( bool _occlusionReasoning );
        void setViewAxis( int _viewAxis );
        void setVerbose( bool _verbose );

        core::Detection estimate();

        // benchmark
        void setRootPath( std::string _rootPath );
        void setObjectDir( std::string _objDir );
        void setSceneDir( std::string _sceneDir );
        void setPoseDir( std::string _poseDir );
        void setObjExt( std::string _objExt );
        void setSceneExt( std::string _sceneExt );
        void setPoseExt( std::string _poseExt );
        void setPoseSep( std::string _poseSep );

        void benchmark( std::string _funcName );
        void printResults();

    private:
        // Ransac variables
        pcl::PointCloud<PointT>::Ptr source;
        pcl::PointCloud<PointT>::Ptr target;
        core::Correspondence::VecPtr corr;
        size_t iterations = 10000;
        size_t sampleSize = 3;
        bool prerejection = true;
        float prerejectionSimilarty = 0.9;
        float inlierThreshold = 0.01;
        float inlierFraction = 0.1;
        bool reestimatePose = true;
        bool fullEvaluation = false;
        bool occlusionReasoning = false;
        bool occlusionRemoval = false;
        int viewAxis = 0;
        bool verbose = false;

        // Feature matching
        float resolution = 1;
        float objectScale = 1;
        float far = -1;
        float radiusNormal = 5;
        float resolutionQuery = 5;
        float resolutionTarget = 5;
        float radiusFeature = 25;
        float cutoff = 50;
        std::string feature = "si";

        // Benchmark variables
        size_t seed;
        std::string rootPath;
        std::string objDir;
        std::string sceneDir;
        std::string poseDir;
        std::string objExt;
        std::string sceneExt;
        std::string poseExt;
        std::string poseSep;
        boost::once_flag flagInit = BOOST_ONCE_INIT;
        std::vector<CloudT::Ptr> objectCloud;
        std::vector<CloudT::Ptr> sceneCloud;
        std::vector< std::vector<core::Correspondence::VecPtr> > correspondences;
        std::vector<benchmarkResult> results;
        std::vector<std::string> objectLabels;

        // Benchmark functions
        void initBenchmark();
        void loadData(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                        std::vector<util::DatasetLoader::ModelPtr> *sceneMesh);
        void computeCorrespondence(std::vector<util::DatasetLoader::ModelPtr> *objectMesh,
                                    std::vector<util::DatasetLoader::ModelPtr> *sceneMesh);
};
