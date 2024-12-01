/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

// SYSTEM
#include <stdio.h>

// 3RD PARTY
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h> // This comes from the mav_comm repo

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"
#include "Float32.pb.h"

// TMN
#include "utility.h"

namespace gazebo
{
    class GazeboServoModel : public ModelPlugin
    {

    public:
        GazeboServoModel()
            : ModelPlugin(),
              rotating_period_(2.0),
              node_handle_(nullptr){}

        virtual ~GazeboServoModel();

    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:

        std::string joint_name_;
        std::string link_name_;
        std::string namespace_;

        double rotating_period_;

        gazebo::transport::NodePtr node_handle_;
        ros::NodeHandle* ros_node_handle_;
        ros::Publisher servo_odom_pub_;

        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::JointPtr joint_;
        physics::LinkPtr  link_;

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;

    };

} // namespace gazebo {

#endif // ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
