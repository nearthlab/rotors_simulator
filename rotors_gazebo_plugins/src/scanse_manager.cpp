#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "mc_msg/SweepInfo.h"
#include "mc_msg/SweepInfoArray.h"

#include <sstream>

ros::Publisher pub; 
mc_msg::SweepInfoArray msgArray;

float prevAng = 0.0;
float currAng = 0.0;

bool SortbyAngle_asc(const mc_msg::SweepInfo &a, const mc_msg::SweepInfo &b)
{
  return a.angle < b.angle;
}

void ScanseRayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  currAng = msg->data[1];

  mc_msg::SweepInfo sweepInfo;    
  sweepInfo.range = msg->data[0];
  sweepInfo.angle = msg->data[1];
  sweepInfo.xrel = msg->data[2];
  sweepInfo.yrel = msg->data[3];

  if((fabs(currAng - prevAng) > 180.0))
  {
    msgArray.VecSweepInfo.push_back(sweepInfo); 
    sort(msgArray.VecSweepInfo.begin(), msgArray.VecSweepInfo.end(), &(SortbyAngle_asc));
    pub.publish(msgArray);
    msgArray.VecSweepInfo.clear();	
  }
  else
  {
    msgArray.VecSweepInfo.push_back(sweepInfo); 
  }

  prevAng = currAng;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanse_manager");

  ros::NodeHandle n;
  pub = n.advertise<mc_msg::SweepInfoArray>("/niv1/lidar_scanse_buffer", 10);
  ros::Subscriber sub = n.subscribe("/niv1/lidar_scanse", 5, ScanseRayCallback); 

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
