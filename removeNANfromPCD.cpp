#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;

int main( int argc, char** argv ){
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud, output_cloud;
  pcl::io::loadPCDFile (argv[1], input_cloud);

  // remove nan
  for( size_t i=0; i<input_cloud.points.size(); i++ )
    if( !isnan(input_cloud.points[ i ].x) && !isnan(input_cloud.points[ i ].y) && !isnan(input_cloud.points[ i ].z) )
      output_cloud.points.push_back( input_cloud.points[ i ] );
  //output_cloud.width = input_cloud.width;
  //output_cloud.height = input_cloud.height;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  pcl::io::savePCDFileBinary( argv[2], output_cloud );

  return 1;
}
