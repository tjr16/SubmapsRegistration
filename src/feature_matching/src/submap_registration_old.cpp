#define PCL_NO_PRECOMPILE
#include <iostream>
#include <fstream>
#include <vector>
#include <feature_matching/corresp_matching.hpp>
#include <feature_matching/utils_visualization.hpp>
#include <pcl/io/pcd_io.h>


POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<32>,
  (float[32], histogram, nn_feat)
)

typedef pcl::Histogram<32> NNFeature;
typedef pcl::PointXYZ PointT;

void extractKeypointsCorrespondences(const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2,
                                     CorrespondencesPtr good_correspondences,YAML::Node config)
{
  // Basic correspondence estimation between keypoints
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
  CorrespondencesPtr all_correspondences(new Correspondences);
  est.setInputSource(keypoints_1);
  est.setInputTarget(keypoints_2);
  double kps_cor_thres = config["kps_cor_thres"].as<double>();

  est.determineReciprocalCorrespondences(*all_correspondences, kps_cor_thres);
  rejectBadCorrespondences(all_correspondences, keypoints_1, keypoints_2, *good_correspondences);

  std::cout << "Number of correspondances " << all_correspondences->size() << std::endl;
  std::cout << "Number of good correspondances " << good_correspondences->size() << std::endl;
}

void extractFeaturesCorrespondences(const PointCloud<SHOT352>::Ptr &shot_src,
                                    const PointCloud<SHOT352>::Ptr &shot_trg,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2,
                                    CorrespondencesPtr all_correspondences, YAML::Node config)
{
  // Basic correspondence estimation between keypoints
  pcl::registration::CorrespondenceEstimation<SHOT352, SHOT352> est;
  // CorrespondencesPtr all_correspondences(new Correspondences);
  est.setInputSource(shot_src);
  est.setInputTarget(shot_trg);
  double feats_cor_thres = config["feats_cor_thres"].as<double>();
  
  est.determineReciprocalCorrespondences(*all_correspondences, feats_cor_thres);
  // rejectBadCorrespondences(all_correspondences, keypoints_1, keypoints_2, *good_correspondences);

  std::cout << "Number of correspondances " << all_correspondences->size() << std::endl;
  // std::cout << "Number of good correspondances " << good_correspondences->size() << std::endl;
}

void extractFeaturesCorrespondencesNN(const PointCloud<NNFeature>::Ptr &nn_feat_src,
                                    const PointCloud<NNFeature>::Ptr &nn_feat_trg,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2,
                                    CorrespondencesPtr all_correspondences, YAML::Node config)
{
  // Basic correspondence estimation between keypoints
  pcl::registration::CorrespondenceEstimation<NNFeature, NNFeature> est;
  // CorrespondencesPtr all_correspondences(new Correspondences);
  est.setInputSource(nn_feat_src);
  est.setInputTarget(nn_feat_trg);
  double feats_cor_thres = config["nn_feats_cor_thres"].as<double>();
  
  est.determineReciprocalCorrespondences(*all_correspondences, feats_cor_thres);
  // rejectBadCorrespondences(all_correspondences, keypoints_1, keypoints_2, *good_correspondences);
  std::cout << "Number of correspondances " << all_correspondences->size() << std::endl;
  // std::cout << "Number of good correspondances " << good_correspondences->size() << std::endl;
}

// void extractFeaturesCorrespondences(const PointCloud<SHOT352>::Ptr &shot_src,
//                                     const PointCloud<SHOT352>::Ptr &shot_trg,
//                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1,
//                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2,
//                                     CorrespondencesPtr all_correspondences, YAML::Node config)
// {
//   // Basic correspondence estimation between keypoints
//   pcl::registration::CorrespondenceEstimation<SHOT352, SHOT352> est;
//   CorrespondencesPtr good_correspondences(new Correspondences);
//   est.setInputSource(shot_src);
//   est.setInputTarget(shot_trg);
//   double feats_cor_thres = config["feats_cor_thres"].as<double>();
  
//   est.determineReciprocalCorrespondences(*all_correspondences, feats_cor_thres);
//   rejectBadCorrespondences(all_correspondences, shot_src, shot_trg, *good_correspondences);

//   std::cout << "Number of correspondances " << all_correspondences->size() << std::endl;
//   all_correspondences = good_correspondences;
//   std::cout << "Number of good correspondances " << all_correspondences->size() << std::endl;
// }

int main(int argc, char **argv)
{
    bool use_nn;
    switch (argc){
    case 6:
      use_nn = false;
      break;
    case 10:
      use_nn = true;
      break;   
    default:
      std::cerr << "Number of input arguments is invalid" << std::endl;
      return (-1);
    }
    PCL_INFO ("argc = %i, use NN = %i\n", argc, use_nn);
    // std::cout<<"argc=" << argc << ", use NN: "<<use_nn<<std::endl;

    // Parse the command line arguments for .pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<PointT>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the cloud files
    PCL_INFO("Load point clouds...\n");
    if (pcl::io::loadPCDFile (argv[1], *cloud_1) < 0){
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }

    if (pcl::io::loadPCDFile (argv[2], *cloud_2) < 0){
        PCL_ERROR ("Error loading cloud %s.\n", argv[2]);
        return (-1);
    }

    // Load the yaml file
    PCL_INFO("Load configuration...\n");
    YAML::Node config = YAML::LoadFile(argv[3]);

    // Load the keypoint cloud files
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2(new pcl::PointCloud<pcl::PointXYZ>);
    // use_nn: assign keypoint clouds
    if (use_nn) {
      PCL_INFO("Load keypoints for NN...\n");
      if (pcl::io::loadPCDFile (argv[4], *keypoints_1) < 0){
        PCL_ERROR ("Error loading cloud %s.\n", argv[4]);
        return (-1);
      }
      if (pcl::io::loadPCDFile (argv[5], *keypoints_2) < 0){
        PCL_ERROR ("Error loading cloud %s.\n", argv[5]);
        return (-1);
      }
    }
 
    const int maxIter = config["gicp_iter"].as<int>();

    // Visualize initial point clouds
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    rgbVis(viewer, cloud_1, 0);
    rgbVis(viewer, cloud_2, 1);

    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
    viewer->resetStoppedFlag();

    // Extract keypoints
    // pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2(new pcl::PointCloud<pcl::PointXYZ>);

    if (!use_nn) {
      // Extract harris keypoints
      PCL_INFO("Extract Harris keypoints...\n");
      harrisKeypoints(cloud_1, *keypoints_1, config);
      harrisKeypoints(cloud_2, *keypoints_2, config);
    } else {
      pcl::console::print_highlight("#Keypoints from cloud1 %zd \n", keypoints_1->size());
      pcl::console::print_highlight("#Keypoints from cloud2 %zd \n", keypoints_2->size());
    }

    // Visualize keypoints
    PCL_INFO("Visualize...\n");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints1_color_handler(keypoints_1, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints2_color_handler(keypoints_2, 0, 255, 0);
    viewer->addPointCloud(keypoints_1, keypoints1_color_handler, "keypoints_src", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_src");
    viewer->addPointCloud(keypoints_2, keypoints2_color_handler, "keypoints_trg", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_trg");

    // Extract correspondences between keypoints/features
    CorrespondencesPtr good_correspondences(new Correspondences);

    if (use_nn) {
      // Load NN descriptors
      // PointCloud<NNFeature>::Ptr nn_feat_1(new PointCloud<NNFeature>);
      // PointCloud<NNFeature>::Ptr nn_feat_2(new PointCloud<NNFeature>);
      // if (pcl::io::loadPCDFile (argv[6], *nn_feat_1) < 0){
      //     PCL_ERROR ("Error loading cloud %s.\n", argv[6]);
      //     return (-1);
      // }
      // if (pcl::io::loadPCDFile (argv[7], *nn_feat_2) < 0){
      //     PCL_ERROR ("Error loading cloud %s.\n", argv[7]);
      //     return (-1);
      // }
      // PCL_INFO("load success");
      // extractFeaturesCorrespondencesNN(nn_feat_1, nn_feat_2, keypoints_1, keypoints_2, good_correspondences, config);

      PCL_INFO("Read correspondences...\n");
      std::vector<int> query_indices;
      std::vector<int> match_indices;
      std::ifstream file1(argv[6]);
      int n1;
      // PCL_INFO("Read correspondences 1... %s\n", argv[6]);
      while (file1 >> n1) {
        query_indices.push_back(n1);
      }
      file1.close();
      std::ifstream file2(argv[7]);
      int n2;
      while (file2 >> n2) {
        match_indices.push_back(n2);
      }
      file2.close();

      // PCL_INFO("Read correspondences 3...\n");
      // for (const auto &n: query_indices) {
      //   std::cout<< n << ";";     
      // }
      // for (const auto &n: match_indices) {
      //   std::cout<< n << ";";     
      // }
      PCL_INFO("Set correspondences ...\n");
      good_correspondences->resize(query_indices.size());
      assert(query_indices.size() == match_indices.size());


      for(int i = 0; i < query_indices.size(); i++){
          good_correspondences->at(i).index_query = query_indices[i];
          good_correspondences->at(i).index_match = match_indices[i];
      }
      PCL_INFO("Print correspondences ...\n");
      for(int i = 0; i < good_correspondences->size(); i++){
        std::cout<< good_correspondences->at(i).index_query << ";";
        std::cout<< good_correspondences->at(i).index_match << ";";
      }
    } else {
      PCL_INFO("Computing SHOT descriptors...\n");
      // Compute SHOT descriptors
      PointCloud<SHOT352>::Ptr shot_1(new PointCloud<SHOT352>);
      PointCloud<SHOT352>::Ptr shot_2(new PointCloud<SHOT352>);
      // TODO: remove shot
      estimateSHOT(cloud_1, keypoints_1, shot_1, config);
      estimateSHOT(cloud_2, keypoints_2, shot_2, config);
      PCL_INFO("Extract SHOT correspondences...\n");
      extractFeaturesCorrespondences(shot_1, shot_2, keypoints_1, keypoints_2, good_correspondences, config);
      // TODO: remove
      for(int i = 0; i < good_correspondences->size(); i++){
          std::cout<< good_correspondences->at(i).index_query << ", " <<  good_correspondences->at(i).index_match <<
          std::endl;
      }
    }

    // Extract correspondences between keypoints
    PCL_INFO("Visualize correspondences...\n");

    plotCorrespondences(*viewer, *good_correspondences, keypoints_1, keypoints_2);
    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
    viewer->resetStoppedFlag();

    // Best transformation between the two sets of keypoints given the remaining correspondences
    Eigen::Matrix4f transform;
    TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;

    //Jiarui: another overriding function: do not use correspondence
    trans_est.estimateRigidTransformation(*keypoints_1, *keypoints_2, *good_correspondences, transform);
    pcl::transformPointCloud(*cloud_2, *cloud_2, transform.inverse());
    // pcl::transformPointCloud(*cloud_1, *cloud_trans, transform);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    rgbVis(viewer, cloud_1, 0);
    rgbVis(viewer, cloud_2, 1);
    // rgbVis(viewer, cloud_trans, 2);

    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
    viewer->resetStoppedFlag();

    // Run GICP
    PCL_INFO ("GICP...\n");
    runGicp(cloud_2, cloud_1, maxIter);

    PCL_INFO ("Save pcd files after registration...\n");
    if (use_nn) {
      pcl::io::savePCDFileASCII(argv[8], *cloud_1);
      pcl::io::savePCDFileASCII(argv[9], *cloud_2);
    } else {
      pcl::io::savePCDFileASCII(argv[4], *cloud_1);
      pcl::io::savePCDFileASCII(argv[5], *cloud_2);
    }

    viewer->removeAllPointClouds();
    rgbVis(viewer, cloud_1, 0);
    rgbVis(viewer, cloud_2, 1);
    // rgbVis(viewer, cloud_trans, 2);
    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
    viewer->resetStoppedFlag();

    return 0;
}

