//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_CONTACT_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_CONTACT_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>


namespace gazebo
{

class GazeboRosContact: public SensorPlugin
{
public:
  GazeboRosContact();
  virtual ~GazeboRosContact();

protected:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  sensors::ContactSensorPtr parentSensor_;

  ros::NodeHandle* node_handle_;
  ros::Publisher publisher_;

  std_msgs::Int32 contacts_;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;

  SensorModel sensor_model_;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_CONTACT_H
