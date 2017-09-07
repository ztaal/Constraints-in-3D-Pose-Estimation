// #include "../headers/includes.h"

// COVIS
#include <covis/covis.h>
using namespace covis;

// Point and feature types
typedef pcl::PointXYZRGBNormal PointT;

// Loaded point clouds and computed histogram features
pcl::PointCloud<PointT>::Ptr querySurf, targetSurf;
pcl::PointCloud<PointT>::Ptr queryCloud, targetCloud;
feature::MatrixT queryFeat, targetFeat;



int main()
{
    pcl::ScopeTime t( "Main" );
    printf(" -- Initialized Program\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PLYReader Reader;
    Reader.read( "/home/ztaal/Downloads/cheff.ply", *cloud );

    pcl::visualization::CloudViewer viewer( "Simple Cloud Viewer" );
    viewer.showCloud( cloud );

    while ( !viewer.wasStopped() ) {
    }

    return 0;
}
