#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/features/spin_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/random.h>
#include <pcl/common/common.h>
#include <pcl/search/search.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/pfh.h>
#include <pcl/common/eigen.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <limits>