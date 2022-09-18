#define PCL_NO_PRECOMPILE

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/warp_point_rigid.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/gicp.h>

#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/feature.h>
#include <pcl/features/shot_lrf.h>

// #include "feature_matching/nn_feature.hpp"


POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<32>,
  (float[32], histogram, nn_feat)
)

typedef pcl::Histogram<32> NNFeature;


// struct NNFeature{
//     float histogram[32];
//     static int descriptorSize() { return 32; }
// };

// POINT_CLOUD_REGISTER_POINT_STRUCT (NNFeature,
//   (float[32], histogram, nn_feat)
// )


// struct NNFeature
// {
// //   PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//   float descriptor[32];
//   PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
// } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

// struct NNFeature
// {
//   float descriptor[32];
// };

// POINT_CLOUD_REGISTER_POINT_STRUCT (NNFeature,           
//                                    (float[32], descriptor, descriptor)
// )


// int main (int argc, char** argv)
// {
//   pcl::PointCloud<MyPointType> cloud;
//   cloud.points.resize (2);
//   cloud.width = 2;
//   cloud.height = 1;

//   cloud[0].test = 1;
//   cloud[1].test = 2;
//   cloud[0].x = cloud[0].y = cloud[0].z = 0;
//   cloud[1].x = cloud[1].y = cloud[1].z = 3;

//   pcl::io::savePCDFile ("test.pcd", cloud);
// }