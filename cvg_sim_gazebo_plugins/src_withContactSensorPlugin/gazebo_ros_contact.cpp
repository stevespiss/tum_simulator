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

#include <hector_gazebo_plugins/gazebo_ros_contact.h>
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/ContactSensor.hh"


#include <limits>

namespace gazebo {

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosContact)


GazeboRosContact::GazeboRosContact(): SensorPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosContact::~GazeboRosContact()
{
  parentSensor_->SetActive(false);
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosContact::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
	parentSensor_ = boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
  if (!parentSensor_)
  {
    gzthrow("GazeboRosContact requires a Contact Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = parentSensor_->GetWorldName();
  world = physics::get_world(worldName);

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameId"))
    frame_id_ = "";
  else
    frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();

  if (!_sdf->HasElement("topicName"))
    topic_ = "contact";
  else
    topic_ = _sdf->GetElement("topicName")->Get<std::string>();

  sensor_model_.Load(_sdf);

  contacts_.data = 0;

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<std_msgs::Int32>(topic_, 1);

  Reset();
  updateConnection = parentSensor_->ConnectUpdated(boost::bind(&GazeboRosContact::Update, this));

  // activate RaySensor
  parentSensor_->SetActive(true);
}

void GazeboRosContact::Reset()
{
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosContact::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
//  if (last_time + updatePeriod > sim_time) return;

  // activate RaySensor if it is not yet active
  if (!parentSensor_->IsActive()) parentSensor_->SetActive(true);


//  std::cout << "test" << std::endl;

  contacts_.data = parentSensor_->GetContacts().contact_size();

  publisher_.publish(contacts_);
}

} // namespace gazebo
