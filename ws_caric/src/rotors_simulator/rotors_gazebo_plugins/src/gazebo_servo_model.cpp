/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.M
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_gazebo_plugins/gazebo_servo_model.h"

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo
{

    GazeboServoModel::~GazeboServoModel() {}

    void GazeboServoModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (kPrintOnPluginLoad)
        {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        model_ = _model;
        world_ = _model->GetWorld();

        namespace_.clear();

        // Get the namespace
        if (_sdf->HasElement("namespace"))
            namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
        else
            gzerr << "[gazebo_servo_model] Please specify a namespace.\n";

        // Get the joint name
        if (_sdf->HasElement("jointName"))
            joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
        else
            gzerr << "[gazebo_servo_model] Please specify a jointName, where the rotor is attached.\n";

        // Get the pointer to the joint.
        joint_ = model_->GetJoint(joint_name_);
        if (joint_ == NULL)
            gzthrow("[gazebo_servo_model] Couldn't find specified joint \"" << joint_name_ << "\".");

        // Get the link name
        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_servo_model] Please specify a linkName of the rotor.\n";

        // Get pointer to link
        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_servo_model] Couldn't find specified link \"" << link_name_ << "\".");

        // Create a gazebo node, initialised with default namespace (typically /gazebo/default/)
        node_handle_ = gazebo::transport::NodePtr(new transport::Node());
        node_handle_->Init();

        // Create a ros node
        ros_node_handle_ = new ros::NodeHandle("/" + namespace_ + "_rosnode");

        // Create a pose publisher
        servo_odom_pub_ = ros_node_handle_->advertise<nav_msgs::Odometry>("/" + namespace_ + "/servo/odometry", 1);

        // Get the rotating period
        getSdfParam<double>(_sdf, "rotatingPeriod", rotating_period_, 2.0);
        printf(KMAG "rotatingPeriod: %f\n" RESET, rotating_period_);

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboServoModel::OnUpdate, this, _1));
    }

    // This gets called by the world update start event.
    void GazeboServoModel::OnUpdate(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
        {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        // Calculate the velocity reference
        double ref_velocity_ = (2*M_PI/rotating_period_)*(sin(2*M_PI/rotating_period_*world_->SimTime().Double()) > 0 ? 1.0 : -1.0);

        // TODO: To be correct, we should calculate the torque and apply it on the link, ignoring it for now

        // Set the velocity
        joint_->SetVelocity(0, ref_velocity_);

        // Get the servo pose
        ignition::math::Pose3d linkPose = link_->WorldPose();
        ignition::math::Vector3d linkLinearVel = link_->WorldLinearVel();
        ignition::math::Vector3d linkAngularVel = link_->WorldAngularVel();
        // ignition::math::Vector3d linkLinearAccel = link_->WorldLinearAccel();

        // Publish the pose
        static nav_msgs::Odometry servo_odom;
        servo_odom.header.stamp = Util::GazeboTimeToRosTime(world_->SimTime());
        servo_odom.header.frame_id = "world";

        servo_odom.pose.pose.position.x = linkPose.X();
        servo_odom.pose.pose.position.y = linkPose.Y();
        servo_odom.pose.pose.position.z = linkPose.Z();
        servo_odom.pose.pose.orientation.x = linkPose.Rot().X();
        servo_odom.pose.pose.orientation.y = linkPose.Rot().Y();
        servo_odom.pose.pose.orientation.z = linkPose.Rot().Z();
        servo_odom.pose.pose.orientation.w = linkPose.Rot().W();

        servo_odom.twist.twist.linear.x = linkLinearVel.X();
        servo_odom.twist.twist.linear.y = linkLinearVel.Y();
        servo_odom.twist.twist.linear.z = linkLinearVel.Z();
        servo_odom.twist.twist.angular.x = linkLinearVel.X();
        servo_odom.twist.twist.angular.y = linkLinearVel.Y();
        servo_odom.twist.twist.angular.z = linkLinearVel.Z();

        servo_odom_pub_.publish(servo_odom);
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboServoModel);
}
