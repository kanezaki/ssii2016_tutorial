#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>

class SavePCD {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub1, _sub2;
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  int save_count;
public:
  SavePCD() : save_count(0) {
    //* subscribe ROS topics
    _sub1 = _nh.subscribe ("/camera/rgb/image_color", 1,  &SavePCD::image_cb, this);
    ROS_INFO ("Listening for incoming data on topic /camera/rgb/image_color ..." );
    _sub2 = _nh.subscribe ("/camera/depth_registered/points", 1,  &SavePCD::points_cb, this);
    ROS_INFO ("Listening for incoming data on topic /camera/depth_registered/points ..." );
  }
  ~SavePCD() {}

  //* get points
  void points_cb( const sensor_msgs::PointCloud2ConstPtr& cloud ){
    if ((cloud->width * cloud->height) == 0)
      return;
    pcl::fromROSMsg (*cloud, input_cloud);
  }

  //* show color img and save color img + point cloud
  void image_cb( const sensor_msgs::ImageConstPtr& msg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);

    //* show color img
    cv::Mat color_img = cv_ptr->image;
    cv::imshow( "color image", color_img );
    cv::waitKey(10);

    if ((input_cloud.width * input_cloud.height) == 0)
      return;

    //* save
    std::stringstream filename1;
    filename1 << save_count << ".png";
    cv::imwrite( filename1.str(), color_img );
    std::cout << filename1.str() << " saved." << std::endl;
    std::stringstream filename2;
    filename2 << save_count << ".pcd";
    pcl::io::savePCDFileBinary( filename2.str(), input_cloud );
    std::cout << filename2.str() << " saved." << std::endl;
    save_count++;
    usleep( 300000 );
  }
};

int main( int argc, char** argv ){
  ros::init(argc,argv,"save_pcd");
  SavePCD spcd;
  ros::spin();

  return 1;
}
