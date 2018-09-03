#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include <sstream>

float terabee_data[8];

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  int topic_index = msg->data[0];
  terabee_data[topic_index] = msg->data[1];
  // ROS_INFO("I heard: [%f] from [%d]", terabee_data[topic_index], topic_index);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terabee_manager");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/niv1/lidar_terabee", 100);

  ros::Subscriber sub[8];

  // int lidar_number;
  // if(!n.getParam("lidar_number", lidar_number)){
  //   ROS_ERROR("lidar_number is not specified!");
  //   return -1;
  // }
  
  for(int i = 0; i<8; i++){
<<<<<<< HEAD
    char topic_name[20];
    sprintf(topic_name, "/niv1/lidar_terabee_%d", i);

    sub[i] = n.subscribe(topic_name, 1000, chatterCallback); 
    ROS_INFO("subscribe %s", topic_name);
=======
    sub[i] = n.subscribe("/niv1/terabee_lidar_"+std::to_string(i), 1000, chatterCallback); 
    ROS_INFO("subscribe /niv1/terabee_lidar_%d", i);
>>>>>>> 063d48428301b5034317c79248ba2db3c2babbef
  }

  ros::Rate loop_rate(120);

  while (ros::ok())
  {
    std_msgs::Float32MultiArray msg;

    msg.data.clear();
    std_msgs::MultiArrayDimension dim;
    dim.label = "ranges";
    dim.size = 8;
    dim.stride = 8;
    msg.layout.dim.push_back(dim);

    for(int i = 0; i<8; i++){
      msg.data.push_back(terabee_data[i]);
    }

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
