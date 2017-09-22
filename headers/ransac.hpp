#pragma once

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

    private:
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
};
