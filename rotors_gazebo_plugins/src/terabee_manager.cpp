#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include "mc_msg/TeraRangerInfo.h"
#include "mc_msg/TeraRangerInfoArray.h"

#include <sstream>

#define PIf 3.141592f
#define D2R PIf/180.0f
#define R2D 180.0f/PIf

float terabee_data[8];
float fAngle[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};

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

float sat(float _val)
{
  float res = 0.0f;

  if(_val > 60.0f)
  {
    res = 60.0f;
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
  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/niv1/lidar_terabee", 10);
  ros::Publisher pub_info = n.advertise<mc_msg::TeraRangerInfoArray>("/niv1/lidar_terabee_info", 10);

  ros::Subscriber sub[8];
  for(int i = 0; i<8; i++){
    sub[i] = n.subscribe("/niv1/terabee_lidar_"+std::to_string(i), 5, chatterCallback); 
    ROS_INFO("subscribe /niv1/terabee_lidar_%d", i);
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

    mc_msg::TeraRangerInfoArray vecTeraRanger;

    for(int i = 0; i < 8; i++){
      msg.data.push_back(terabee_data[i]);

      mc_msg::TeraRangerInfo terarangerInfoTemp;
      terarangerInfoTemp.range = sat(terabee_data[i]);
      terarangerInfoTemp.angle = wrap(fAngle[i]*D2R);
      terarangerInfoTemp.xrel = terarangerInfoTemp.range * cos(terarangerInfoTemp.angle);
      terarangerInfoTemp.yrel = terarangerInfoTemp.range * sin(terarangerInfoTemp.angle);
   
      vecTeraRanger.VecTeraRangerInfo.push_back(terarangerInfoTemp);
    }

    pub.publish(msg);
    pub_info.publish(vecTeraRanger);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
