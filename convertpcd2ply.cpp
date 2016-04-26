#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// notice! fake faces!

using namespace pcl;

int main( int argc, char** argv ){

  if( argc != 3 ){
    std::cerr << "usage: " << argv[0] << " [Input PCD file] [Output PLY file]" << std::endl;
    return 0;
  }

  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::io::loadPCDFile (argv[1], input_cloud);

  //* count number of points
  int vnum = 0;
  for( size_t i = 0; i < input_cloud.points.size(); ++i )
    if( !isnan(input_cloud.points[ i ].x) && !isnan(input_cloud.points[ i ].y) && !isnan(input_cloud.points[ i ].z) )
      vnum++;

  //* fake faces
  int fnum = floor(vnum/3);
  if( vnum % 3 != 0 )
    fnum++;

  //* write PLY file
  FILE *fp = fopen( argv[2], "w" );
  fprintf(fp,"ply\n");
  fprintf(fp,"format ascii 1.0\n");
  fprintf(fp,"element vertex %d\n", vnum);
  fprintf(fp,"property float x\n");
  fprintf(fp,"property float y\n");
  fprintf(fp,"property float z\n");
  fprintf(fp,"property uchar red\n");
  fprintf(fp,"property uchar green\n");
  fprintf(fp,"property uchar blue\n");
  fprintf(fp,"element face %d\n", fnum);
  fprintf(fp,"property list uchar int vertex_indices\n");
  fprintf(fp,"end_header\n");
  for( size_t i = 0; i < input_cloud.points.size(); ++i ){
    if( isnan(input_cloud.points[ i ].x) || isnan(input_cloud.points[ i ].y) || isnan(input_cloud.points[ i ].z) )
      continue;
    int color = *reinterpret_cast<const int*>(&(input_cloud.points[i].rgb));
    int r = (0xff0000 & color) >> 16;
    int g = (0x00ff00 & color) >> 8;
    int b =  0x0000ff & color;
    fprintf(fp,"%f %f %f %d %d %d\n", input_cloud.points[ i ].x, input_cloud.points[ i ].y, input_cloud.points[ i ].z, r, g, b);
  }
  for( int i = 0; i < fnum-1; ++i )
    fprintf(fp,"3 %d %d %d\n", i*3, i*3+1, i*3+2);
  fprintf(fp,"3 %d %d %d\n", vnum-3, vnum-2, vnum-1);
  fclose( fp );

  return 1;
}
