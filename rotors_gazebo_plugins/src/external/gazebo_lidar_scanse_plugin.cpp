/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "rotors_gazebo_plugins/external/gazebo_lidar_scanse_plugin.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo/sensors/sensors.hh>
#include <boost/algorithm/string.hpp>

#include "rotors_gazebo_plugins/common.h"

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// USER
#include <sstream>
#include <cmath>
#include "ConnectGazeboToRosTopic.pb.h"

#define PI  3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

double constrainAngle(double x){
    x = fmod(x,360.0);
    if (x < 0.0)
        x += 360.0;
    return x;
}

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboLidarScansePlugin)

GazeboLidarScansePlugin::GazeboLidarScansePlugin()
    : pubs_and_subs_created_(false) {}

GazeboLidarScansePlugin::~GazeboLidarScansePlugin() {
  this->ray_sensor->LaserShape()->DisconnectNewLaserScans(this->newLaserScansConnection);
  this->newLaserScansConnection.reset();

  this->ray_sensor.reset();
  this->world.reset();
}

void GazeboLidarScansePlugin::FindSensorAndJoint(const physics::ModelPtr &_model) {
  this->joint = _model->GetJoint("niv1/lidar_optical_joint_"+lidar_number_);
  if(!this->joint)
    GZ_ASSERT(_model, "joint not found");
  const auto link = joint->GetChild();
  std::string ray_sensor_name = link->GetSensorName(0);
  
  const auto _ray_sensor = sensors::get_sensor(ray_sensor_name);
  if(!_ray_sensor)
    GZ_ASSERT(_ray_sensor, "Sensor not found");

  this->ray_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_ray_sensor);
}

void GazeboLidarScansePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  float angular_velocity;

  GZ_ASSERT(_model, "_model pointer is NULL");
  GZ_ASSERT(_sdf, "_sdf pointer is NULL");

  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  getSdfParam<std::string>(_sdf, "lidarNumber", lidar_number_, "lidar_default");
  getSdfParam<float>(_sdf, "angularVelocity", angular_velocity, 10);
  angular_velocity = angular_velocity*360.0*M_PI/180.0;

  gzwarn << "lidar_number_: " << lidar_number_ << "\n";

  FindSensorAndJoint(_model);

  if (!this->ray_sensor)
    gzthrow("RayPlugin requires a Ray Sensor");
  if (!this->joint)
    gzthrow("RayPlugin requires a joint");

  this->world = physics::get_world(this->ray_sensor->WorldName());

  this->joint->SetVelocity(0, angular_velocity);

  this->newLaserScansConnection =
    this->ray_sensor->LaserShape()->ConnectNewLaserScans(
          boost::bind(&GazeboLidarScansePlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  boost::replace_all(namespace_, "/", "");

  getSdfParam<std::string>(_sdf, "lidarTopic", lidar_topic_, "lidar_default");
  
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  node_handle_->Init();
}

void GazeboLidarScansePlugin::OnNewLaserScans() {
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  std::vector<double> ranges;

  std_msgs::Float32MultiArray array_msg;
  array_msg.data.clear();

  std_msgs::MultiArrayDimension dim;
  dim.label = "ranges";
  dim.size = 4;
  dim.stride = 4;
  array_msg.layout.dim.push_back(dim);

  ray_sensor->Ranges(ranges);
  int resolution = ranges.size();  // number of rays
  float min_range;

  double angle_raw;
  math::Angle angle = this->joint->GetAngle(0);
  angle_raw = (angle.Degree());
  angle_raw = constrainAngle(angle_raw);
  angle.Normalize();

  // ss << parentSensor->GetRange(i) <<" ";
  
  min_range = ray_sensor->RangeMax();
  for (int i = 0; i < resolution; i++)
    if (ranges[i] < min_range)
      min_range = ranges[i];

  float xrel = min_range*cos(angle_raw*D2R);
  float yrel = min_range*sin(angle_raw*D2R);
  
  //ROS_INFO("range:%.4f, angle:%.2f", min_range, angle_raw);
  //ROS_INFO(" ");
  array_msg.data.push_back(min_range);  
  array_msg.data.push_back(angle_raw);
  array_msg.data.push_back(xrel);
  array_msg.data.push_back(yrel);

  lidar_pub_.publish(array_msg);
}

void GazeboLidarScansePlugin::CreatePubsAndSubs() {
  gzdbg << "Lidar pub , ros init = " << ros::isInitialized() << std::endl;
  ros::NodeHandle nh;
  // rosnode_ = new ros::NodeHandle("niv1");
  // lidar_pub_ = nh.advertise<lidar_msgs::msgs::lidar>("/niv1/lidar", 10);
  lidar_pub_ = nh.advertise<std_msgs::Float32MultiArray>(
      "/" + namespace_ + "/" + lidar_topic_, 10);
}
