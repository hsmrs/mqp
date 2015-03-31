#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot");

  ros::NodeHandle n;

  ros::Publisher registration_pub = n.advertise<std_msgs::String>("hsmrs/robot_registration", 100);
  ros::Publisher log_pub = n.advertise<std_msgs::String>("test_robot/log_messages", 100);
  ros::Publisher status_pub = n.advertise<std_msgs::String>("test_robot/status", 100);
  ros::Publisher help_pub = n.advertise<std_msgs::String>("test_robot/help", 100);

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("test_robot/pose", 100);

  ros::spinOnce();
  ros::Rate loop_rate(1);
  loop_rate.sleep();

  std_msgs::String msg;

  std::stringstream ss;

  //name;logTopic;imageTopic;poseTopic;statusTopic;helpTopic
  std::string name = "Test Robot";
  std::string logTopic = "test_robot/log_messages";
  std::string imageTopic = "test_robot/camera";
  std::string poseTopic = "test_robot/pose";
  std::string statusTopic = "test_robot/status";
  std::string helpTopic = "test_robot/help";

  ss << name << ";" << logTopic << ";" << imageTopic << ";" << poseTopic << ";" << statusTopic << ";" << helpTopic;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  int i = 0;

  while (ros::ok())
  {
    if (i == 0){
      registration_pub.publish(msg);
      i++;
    }
    else if (i == 1){
      msg.data = "This is my first log!";
      log_pub.publish(msg);
      i++;
    }
    else if (i == 2){
      msg.data = "Happy";
      status_pub.publish(msg);
      i++;
    }
    else if (i == 3){
      msg.data = "true";
      help_pub.publish(msg);
      i++;
    }
    /*
    else if (i == 4){
      registration_pub.publish(msg);
      i++;
    }
    else if (i == 5){
      registration_pub.publish(msg);
      i++;
    }*/
      loop_rate.sleep();
  }

  return 0;
}