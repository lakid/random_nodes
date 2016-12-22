#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace sensor_msgs;
using namespace message_filters;

std::ofstream csvfile;

void callback(const ImageConstPtr &image, const nav_msgs::OdometryConstPtr &odom)
{
  cv_bridge::CvImagePtr cv_ptr;

  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
  // Solve all of perception here...
  
  std::stringstream filename;
  filename << "image_" << image->header.seq <<".png";
  cv::imwrite(filename.str(), cv_ptr->image);

  csvfile << image->header.seq <<"," << 
    image->header.stamp.toNSec() << "," <<
    filename.str() << "," <<
    odom->pose.pose.position.x << "," <<
    odom->pose.pose.position.y << "," <<
    odom->pose.pose.position.z << "," <<
    odom->pose.pose.orientation.x << "," <<
    odom->pose.pose.orientation.y << "," <<
    odom->pose.pose.orientation.z << "," <<
    odom->pose.pose.orientation.w << "," <<
    odom->twist.twist.linear.x << "," <<
    odom->twist.twist.linear.y << "," <<
    odom->twist.twist.linear.z << "," <<
    odom->twist.twist.angular.x << "," <<
    odom->twist.twist.angular.y << "," <<
    odom->twist.twist.angular.z << "," <<
    std::endl;

  ROS_INFO("Saved image: %s", filename.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  csvfile.open("image_gt.csv", std::ofstream::out | std::ofstream::app);

  csvfile << "Seq, Timestamp, image_file, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, " <<
    "lvelocity_x, lvelocity_y, lvelocity_z, avelocity_x, avelocity_y, avelocity_z" <<std::endl;

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "/ardrone/bottom/image_raw", 10);
  message_filters::Subscriber<nav_msgs::Odometry> gt_sub(nh, "/ground_truth/state", 10);

  typedef sync_policies::ApproximateTime<Image, nav_msgs::Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, gt_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  csvfile.close();
  return 0;
}