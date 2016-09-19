/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 */

// #include "gazebo_ros_template.h"
 
// #include <gazebo/common/Plugin.hh>
// #include <gazebo/gazebo.hh>
// #include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/Illuminance.h>


namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLight)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosLight::GazeboRosLight():
  _nh("light_sensor_plugin"),
  _fov(6),
  _range(10)
  {
    _sensorPublisher = _nh.advertise<sensor_msgs::Illuminance>("lightSensor", 1);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosLight::~GazeboRosLight()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosLight::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosLight::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    static int seq=0;

    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
      {
        common::Time cur_time = this->world_->GetSimTime();
        if (cur_time - this->last_update_time_ >= this->update_period_)
        {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;

          sensor_msgs::Illuminance msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "";
          msg.header.seq = seq;

          int startingPix = _width * ( (int)(_height/2) - (int)( _fov/2)) - (int)(_fov/2);

          double illum = 0;
          for (int i=0; i<_fov ; ++i)
          {
            int index = startingPix + i*_width;
            for (int j=0; j<_fov ; ++j)
              illum += _image[index+j];
          }

          msg.illuminance = illum/(_fov*_fov);
          msg.variance = 0.0;

          _sensorPublisher.publish(msg);

          seq++;
        }
      }
    }
  }
}

// namespace gazebo
// {
// // Register this plugin with the simulator
// GZ_REGISTER_MODEL_PLUGIN(ga);

// ////////////////////////////////////////////////////////////////////////////////
// // Constructor
// GazeboRosTemplate::GazeboRosTemplate()
// {
// }

// ////////////////////////////////////////////////////////////////////////////////
// // Destructor
// GazeboRosTemplate::~GazeboRosTemplate()
// {
// }

// void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
// {
//   // int i = i + 1;
//   // Make sure the ROS node for Gazebo has already been initalized
//   if (!ros::isInitialized())
//   {
//     ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
//       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
//     return;
//   }
// ros::Rate rate(30);
// //   ros::NodeHandle _nh;
// //   ros::Publisher _wheel1Publisher;
// //    // this->model->SetLinearVel(math::Vector3(.03, 0, 0));
// //   std_msgs::String msg;
// //   std::stringstream ss;
// //   ss << "From Topic" ;
// //   msg.data = ss.str();

// //   _wheel1Publisher = _nh.advertise<std_msgs::String>("/From_plugin", 1);
// //    while(ros::ok){
// //     _wheel1Publisher.publish(msg);

// //     ros::spinOnce();
// //     rate.sleep();
//    // }

// }

// ////////////////////////////////////////////////////////////////////////////////
// // Update the controller
// void GazeboRosTemplate::UpdateChild()
// {
// }

// }