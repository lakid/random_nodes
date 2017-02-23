#include <ros/ros.h>
#include <iostream>
#include <signal.h>

#include <clog_msgs/imgForward.h>

#define OPTIMISE 0b10000000
#define MERGE 0b01000000

void sigintHandler(int sig)
{
  std::cout << "SIGINT recieved" <<std::endl;
  std::cin.clear();
  ros::shutdown();
}

// A struct to hold the synchronized camera data
// Struct to store stereo data

// Command list
// 0 - don't optimise merge
// 1 - optimise don't merge
// 2 - optimise and merge
// 3 - display only, sont optimse

int main(int argc, char **argv)
{
  int count = 50;
  clog_msgs::imgForward srv;

  ros::init(argc, argv, "map_maker", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  signal(SIGINT, sigintHandler);

  ros::ServiceClient srv_client = nh.serviceClient<clog_msgs::imgForward>("/cmd");

  srv.request.cmd = 0;
  srv.request.type = count;

  srv_client.call(srv);
  count++;

  while (ros::ok())
  {
    srv.request.cmd = 1;
    srv.request.type = count;
    srv_client.call(srv);

    std::string a;
    std::cout << "Press enter to merge. Input n to skip: ";
    std::cin.clear();
    // std::cin.ignore(INT_MAX);
    //std::cin >> a;
    std::getline(std::cin, a);
    if (a.compare("n") && ros::ok())
    {
      std::cout << "Merging" << std::endl;
      srv.request.cmd = 2;
      srv.request.type = count;
      srv_client.call(srv);
    }
    else
    {
      std::cout << "Skipped merging" << std::endl;
    }

    count +=2;
    ros::spinOnce();
  }
  return 0;
}
