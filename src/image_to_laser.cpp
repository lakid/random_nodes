#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#define RESOLUTION 0.05

using namespace sensor_msgs;

std::ofstream csvfile_range, csvfile_bearing;
ros::Publisher pub;

void callback(const ImageConstPtr &msg)
{
  cv::Mat image;

  image = cv_bridge::toCvCopy(msg, "mono8")->image; // or use "bgr8"

   cv::Mat gdst, rsrc, dst, gray_image;
  // //cv::cvtColor(image, gray_image, CV_BGR2GRAY);
  // //cv::resize(gray_image, rsrc, Size(round(rz_factor * cv_ptr->image.cols), round(rz_factor * cv_ptr->image.rows)));

  cv::GaussianBlur(image, gdst, cv::Size(3, 3), 0.95);

  cv::Canny(gdst, dst, 15, 50, 3);

  sensor_msgs::LaserScan out_msg;
  out_msg.header.stamp = msg->header.stamp;

  csvfile_range << msg->header.stamp.toNSec();
  csvfile_bearing << msg->header.stamp.toNSec();

  for (int row = 0; row < dst.rows; row++ )
  {
    for (int col = 0; col < dst.cols; col++)
    {
      if (dst.at<int>(cv::Point(col,row)))
      {
        double range = sqrt(pow(row-dst.rows/2, 2) + pow(col-dst.cols/2, 2)) * RESOLUTION ;
        out_msg.ranges.push_back(range);
        csvfile_range << ", "<< range ;

        double bearing = atan2 (row-dst.rows/2, col-dst.cols/2);
        out_msg.intensities.push_back(bearing);
        csvfile_bearing << ", " << bearing;

        //ROS_INFO("Range: %f, bearing: %f", range, bearing);
      }
    }
  }
  
  csvfile_range << std::endl;
  csvfile_bearing << std::endl;

  pub.publish(out_msg);

  cv::imshow("view", dst);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  csvfile_range.open("/tmp/csvfile_range.csv", std::ofstream::out);
  csvfile_bearing.open("/tmp/csvfile_bearing.csv", std::ofstream::out);

  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/downward_cam/camera/image", 10, callback);

  pub = nh.advertise<sensor_msgs::LaserScan>("edge_laser",1000);

  ros::spin();

  csvfile_range.close();
  csvfile_bearing.close();
  return 0;
}