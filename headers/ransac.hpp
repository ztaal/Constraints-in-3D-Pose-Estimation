#pragma once

// Covis
#include <covis/covis.h>
using namespace covis;

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// template<typename PointT>
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
        void setFitEvaluation( detect::FitEvaluation<PointT>::Ptr _fe );
        void setInlierThreshold( float _inlierThreshold );
        void setInlierFraction( float _inlierFraction );
        void setReestimatePose( bool _reestimatePose );
        void setFullEvaluation( bool _fullEvaluation );
        void setPrerejection( bool _prerejection );
        void setPrerejectionSimilarity( float _prerejectionSimilarty );
        //     setVerbose(true);

        core::Detection estimate();

        // benchmark
        void setRootPath( std::string _rootPath );
        void setObjectDir( std::string _objDir );
        void setSceneDir( std::string _sceneDir );
        void setPoseDir( std::string _poseDir );
        void setObjExt( std::string _objExt );
        void setSceneExt( std::string _sceneExt );
        void setPoseExt( std::string _poseExt );
        void setPoseSep( std::string _PoseSep );

        void benchmark();

    private:
        // Ransac variables
        pcl::PointCloud<PointT>::Ptr source;
        pcl::PointCloud<PointT>::Ptr target;
        core::Correspondence::VecPtr corr;
        size_t iterations = 10000;
        size_t sampleSize = 3;
        detect::FitEvaluation<PointT>::Ptr fe;
        bool prerejection = true;
        float prerejectionSimilarty = 0.9;
        float inlierThreshold = 0.01;
        float inlierFraction = 0.1;
        bool reestimatePose = true;
        bool fullEvaluation = false;
        bool occlusionRemoval = false;
        bool verbose = false;

        // Benchmark variables
        std::string rootPath;
        std::string objDir;
        std::string sceneDir;
        std::string poseDir;
        std::string objExt;
        std::string sceneExt;
        std::string poseExt;
        std::string PoseSep;
};
