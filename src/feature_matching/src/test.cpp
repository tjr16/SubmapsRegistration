// # include<feature_matching/nn_feature.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// typedef pcl::Histogram NNFeat;

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<32>,
  (float[32], histogram, nn_feat)
)

typedef pcl::Histogram<32> NNFeat;

int main (int argc, char** argv)
{
    pcl::PointCloud<NNFeat> cloud;

    cloud.points.resize (5);
    cloud.width = 5;
    cloud.height = 1;
    for (int j=0; j<5; j++){
        // cloud[j].descriptor = new float[32];
        for (int i=0; i<32; i++){
            cloud[j].histogram[i] = static_cast<float> (rand());
        }
    }
    


// float* new_arr = new float[dimension]();

//    int randArray[sz];
//    for(int i=0;i<sz;i++)
//       randArray[i]=rand()%100;  //Generate number between 0 to 99


//   cloud[0].test = 1;
//   cloud[1].test = 2;
//   cloud[0].x = cloud[0].y = cloud[0].z = 0;
//   cloud[1].x = cloud[1].y = cloud[1].z = 3;

  pcl::io::savePCDFile ("testtest.pcd", cloud);
}