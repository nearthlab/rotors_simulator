#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "mc_msg/SweepInfo.h"
#include "mc_msg/SweepInfoArray.h"
#define PIf  3.141592
#define D2R PIf/180.0
#define R2D 180.0/PIf
#include <sstream>

ros::Publisher pub; 
mc_msg::SweepInfoArray msgArray;

float prevAng = 0.0;
float currAng = 0.0;

bool SortbyAngle_asc(const mc_msg::SweepInfo &a, const mc_msg::SweepInfo &b)
{
  return a.angle < b.angle;
}

float sat(float _val)
{
  float res = 0.0f;

  if(_val > 40.0f)
  {
    res = 40.0f;
  }
  else if (_val < 1.0f)
  {
    res = 1.0f;
  }
  else
  {
    res = _val;
  }

  return res;
}

// wrap-up function, angle between -PI and PI
float wrap(float _angle)
{
    _angle = fmod(_angle, 2.0*PIf);

    if(_angle < -PIf)
    {
        _angle += 2.0*PIf;
    }
    else if(_angle > PIf)
    {
        _angle -= 2.0*PIf;
    }
    else
    {
        _angle = _angle;
    }

    return _angle;
}

void ScanseRayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  currAng = msg->data[1];

  mc_msg::SweepInfo sweepInfo;

  sweepInfo.range = sat(msg->data[0]);
  sweepInfo.angle = (msg->data[1]);
  sweepInfo.xrel = msg->data[2];
  sweepInfo.yrel = msg->data[3];

  if((fabs(currAng - prevAng) > 180.0f))
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
  pub = n.advertise<mc_msg::SweepInfoArray>("/niv1/lidar_scanse_info", 10);
  ros::Subscriber sub = n.subscribe("/niv1/lidar_scanse", 5, ScanseRayCallback); 

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
