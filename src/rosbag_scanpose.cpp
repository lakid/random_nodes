#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <clog_msgs/imgForward.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <clog_msgs/ScanPose.h>

#define OPTIMISE 0b10000000
#define MERGE 0b01000000

#define MAP_RESOLUTION 0.05

#define WRITE_BAG 1

// A struct to hold the synchronized camera data
// Struct to store stereo data

// Command list
// 0 - don't optimise merge
// 1 - optimise don't merge
// 2 - optimise and merge
// 3 - display only, sont optimse
#if WRITE_BAG
 rosbag::Bag outBag;
#endif

class ScanposeData
{
public:
  clog_msgs::ScanPose msg;

  ScanposeData(sensor_msgs::LaserScan &scan,
               const nav_msgs::Odometry::ConstPtr &pose,
               const nav_msgs::Odometry::ConstPtr &gps,
               const sensor_msgs::CameraInfo::ConstPtr &info, int seq)
  {
    msg.header.stamp = pose->header.stamp;
    msg.scan = scan;
    msg.pose = *pose;
    msg.gps = *gps;
    msg.cinfo = *info;
    msg.imgType = 1;
    msg.cinfo.header.seq = seq;
  }
};

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};

std::vector<ScanposeData> scanpose_dataset_;
ros::Publisher scanpose_pub_;
// Callback for synchronized messages
void callback(const sensor_msgs::Image::ConstPtr &img,
              const nav_msgs::Odometry::ConstPtr &pose,
              const nav_msgs::Odometry::ConstPtr &gps,
              const sensor_msgs::CameraInfo::ConstPtr &info)
{
  sensor_msgs::LaserScan scan;
  cv::Mat gdst, rectifiedDst;

  cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(img, img->encoding);

  cv::GaussianBlur(cvPtr->image, gdst, cv::Size(9, 9), 5.0);
  cv::Canny(gdst, rectifiedDst, 15, 50, 3);

  //Step 3: Convert the extracted edge pixels to a laser scan.
  for (int row = 0; row < rectifiedDst.rows; row++)
  {
    for (int col = 0; col < rectifiedDst.cols; col++)
    {
      //printf("Pixel value: %d\n", rectifiedDst.at<uchar>(cv::Point(col, row)));
      if (rectifiedDst.at<uchar>(cv::Point(col, row)) > 240)
      {
        double range = sqrt(pow(row - rectifiedDst.rows / 2, 2) + pow(col - rectifiedDst.cols / 2, 2)) * MAP_RESOLUTION;
        scan.ranges.push_back(range);

        double bearing = atan2(row - rectifiedDst.rows / 2, col - rectifiedDst.cols / 2);
        scan.intensities.push_back(bearing);
      }
    }
  }

  // Stereo dataset is class variable to store data

  if (!scanpose_dataset_.empty() && 
      (pose->header.stamp.toSec() - scanpose_dataset_.back().msg.pose.header.stamp.toSec()) <.1) // if the previous message was less than 100ms ago
  {
    scanpose_dataset_.pop_back();
  }

  ScanposeData sd(scan, pose, gps, info, scanpose_dataset_.size());
  scanpose_dataset_.push_back(sd);

#if WRITE_BAG
  outBag.write("scanpose", sd.msg.header.stamp, sd.msg);
  outBag.write("pose", pose->header.stamp, pose);
  outBag.write("image_raw", img->header.stamp, img);
  outBag.write("gps", gps->header.stamp, gps);
#endif
}

// Load bag
void loadBag(const std::string &filename)
{
  rosbag::Bag bag;

  ROS_INFO_STREAM("Reading bag: " << filename);

  try
  {
    bag.open(filename, rosbag::bagmode::Read);
  }
  catch (std::string e)
  {
    ROS_ERROR_STREAM("Cannot open bag: "<< filename << " Exception: "<< e);
    ros::shutdown();
    return;
  }

  std::string cam_topic = "/hexacopter/perspective_camera/image_raw";
  std::string gps_topic = "/hexacopter/gps/rtkfix";
  std::string pose_topic = "/hexacopter/mavros/local_position/odom";
  std::string cinfo_topic = "/hexacopter/perspective_camera/camera_info";

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(cam_topic);
  topics.push_back(gps_topic);
  topics.push_back(pose_topic);
  topics.push_back(cinfo_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> img_sub;
  BagSubscriber<nav_msgs::Odometry> gps_sub, pose_sub;
  BagSubscriber<sensor_msgs::CameraInfo> info_sub;

  //   // Use time synchronizer to make sure we get properly synchronized images
  // message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry, nav_msgs::Odometry, sensor_msgs::CameraInfo>
  //   sync(img_sub, gps_sub, pose_sub, info_sub, 25);

  // Use time synchronizer to make sure we get properly synchronized images
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry,
                                                          nav_msgs::Odometry, sensor_msgs::CameraInfo>
      MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(200), img_sub, pose_sub, gps_sub, info_sub);

  // message_filters::TimeSynchronizer<sensor_msgs::Image, nav_msgs::Odometry, nav_msgs::Odometry, sensor_msgs::CameraInfo>
  //   sync(img_sub, gps_sub, pose_sub, info_sub, 25);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  // Load all messages into our stereo dataset
  BOOST_FOREACH (rosbag::MessageInstance const m, view)
  {
    if (!ros::ok())
      exit(0);

    if (m.getTopic() == cam_topic || ("/" + m.getTopic() == cam_topic))
    {
      sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
      if (img != NULL)
        img_sub.newMessage(img);
    }

    if (m.getTopic() == gps_topic || ("/" + m.getTopic() == gps_topic))
    {
      nav_msgs::Odometry::ConstPtr gps = m.instantiate<nav_msgs::Odometry>();
      if (gps != NULL)
        gps_sub.newMessage(gps);
    }

    if (m.getTopic() == pose_topic || ("/" + m.getTopic() == pose_topic))
    {
      nav_msgs::Odometry::ConstPtr pose = m.instantiate<nav_msgs::Odometry>();
      if (pose != NULL)
        pose_sub.newMessage(pose);
    }

    if (m.getTopic() == cinfo_topic || ("/" + m.getTopic() == cinfo_topic))
    {
      sensor_msgs::CameraInfo::ConstPtr info = m.instantiate<sensor_msgs::CameraInfo>();
      if (info != NULL)
        info_sub.newMessage(info);
    }
    std::cout << ".";
  }
  bag.close();
}

bool cmdCallback(clog_msgs::imgForward::Request &req, clog_msgs::imgForward::Response &resp)
{
  ROS_INFO_STREAM("Rcvd cmd:" << req);

  if (req.type >= 0 && req.type < scanpose_dataset_.size())
  {
    clog_msgs::ScanPose out_msg = scanpose_dataset_[req.type].msg;
    if (req.cmd == 1)
    {
      out_msg.imgType |= OPTIMISE;
    }
    else if (req.cmd == 2)
    {
      out_msg.imgType |= OPTIMISE;
      out_msg.imgType |= MERGE;
    }
    else if (req.cmd == 0)
    {
      out_msg.imgType |= MERGE;
    }

    ROS_INFO("Publishing Message...");
    scanpose_pub_.publish(out_msg);
  }
  else
  {
    ROS_ERROR("Invalid seqno");
  }

  resp.ret = 0;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh("~");

  if (argc < 2)
  {
    ROS_ERROR("Please provide a bagfile name.");
    ros::shutdown();
    return 1;
  }
  

  scanpose_pub_ = nh.advertise<clog_msgs::ScanPose>("/scanpose", 1000);

  ros::ServiceServer cmd_srv = nh.advertiseService("/cmd", cmdCallback);

#if WRITE_BAG
  outBag.open("scanpose_bag.bag", rosbag::bagmode::Write);
#endif

  loadBag(argv[1]);

  int dataset_length = scanpose_dataset_.size();
  std::cout << std::endl
            << "extracted length: " << dataset_length << std::endl;

  ros::spin();

#if WRITE_BAG
  outBag.close();
#endif

  return 0;
}
