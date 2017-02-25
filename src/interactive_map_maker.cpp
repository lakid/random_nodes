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
// 3 - display only, dont optimse/merge

int main(int argc, char **argv)
{
  int count = 50;
  std::string st_count;
  
  // bool keep_list[] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1};


  clog_msgs::imgForward srv;

  ros::init(argc, argv, "map_maker", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  signal(SIGINT, sigintHandler);

  ros::ServiceClient srv_client = nh.serviceClient<clog_msgs::imgForward>("/cmd");

  std::cout << "Enter the start index: " ;
  std::getline(std::cin, st_count);

  count  = std::atoi(st_count.c_str());
  
  // if (!keep_list[count])
  // {
  //   ROS_INFO("Input inside ignore list, skipping");
  //   while(!keep_list[count])
  //    count++;
  // }

  srv.request.cmd = 0;
  srv.request.type = count;
  srv_client.call(srv);

  std::cout << std::endl << "Initialized at: "<<count  <<std::endl;
  // count++;

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

    count++;
    // while(!keep_list[count])
    //  count++;
    ros::spinOnce();
  }
  return 0;
}
