/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <algorithm>
#include <assert.h>

#include <motor_plugin.h>

#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <math.h>


namespace gazebo{

enum{
  SERVOLEFT,
  SERVORIGHT,
  FRONTRIGHT,
  FRONTLEFT,
  REARRIGHT,
  REARLEFT,
};

enum{
  DRIVE_MODE_AUTO,
  DRIVE_MODE_PIVOT,
  DRIVE_MODE_SPIN,
  DRIVE_MODE_SIDEWAYS,
};

// Constructor
MotorDrivePlugin::MotorDrivePlugin(){
}

// Destructor
MotorDrivePlugin::~MotorDrivePlugin(){
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void MotorDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
  this->parent = _parent;
  this->world = _parent->GetWorld();

  gzdbg << "plugin parent sensor name: " << parent->GetName() << "\n";

  if (!this->parent) { gzthrow("Differential_Position2d controller requires a Model as its parent"); }

  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace")){
    this->robotNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
  }

  if (!_sdf->HasElement("topicName")){
    ROS_WARN("Motor Drive plugin missing <topicName>, defaults to cmd_vel");
    this->topicName = "cmd_vel";
  }else{
    this->topicName = _sdf->Get<std::string>("topicName");
  }


  motorCmd[FRONTRIGHT] = 0;
  motorCmd[FRONTLEFT] = 0;
  motorCmd[REARRIGHT] = 0;
  motorCmd[REARLEFT] = 0;
  motorCmd[SERVOLEFT] = 0;
  motorCmd[SERVORIGHT] = 0;

  speed_ = 0;
  heading_ = 90;
  alive_ = true;
  mode_ = DRIVE_MODE_AUTO;


  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "motor_drive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

  ROS_INFO("starting motor drive plugin in ns: %s", this->robotNamespace.c_str());

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<roberto_msgs::MotorState>(topicName, 1,
                                                          boost::bind(&MotorDrivePlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);


  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  servo_pub = rosnode_->advertise<roberto_msgs::JointCommand>("servo_joint_position_controller/command", 1);

  // Initialize the controller
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;


  // start custom queue for diff drive
  this->callback_queue_thread_ = boost::thread(boost::bind(&MotorDrivePlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorDrivePlugin::UpdateChild, this));
}

// Update the controller
void MotorDrivePlugin::UpdateChild(){
  double d1, d2, d3, d4;
  double dr, da;
  double dtime = this->world->GetSimTime().Double();
  double stepTime = dtime - lastTime;
  lastTime = dtime;

  GetPositionCmd();

  // Distance travelled by front wheels
  /*d1 = stepTime * wheelDiameter / 2 * joints[FRONTLEFT]->GetVelocity(0);
  d2 = stepTime * wheelDiameter / 2 * joints[FRONTRIGHT]->GetVelocity(0);
  d3 = stepTime * wheelDiameter / 2 * joints[REARLEFT]->GetVelocity(0);
  d4 = stepTime * wheelDiameter / 2 * joints[REARRIGHT]->GetVelocity(0);*/


  dr = (d1 + d2 + d3 + d4) / 4;

  da = (d1 - d2) / wheelSeparation;   // FIX

  // Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  // Compute odometric instantaneous velocity
  odomVel[0] = dr / stepTime;
  odomVel[1] = 0.0;
  odomVel[2] = da / stepTime;

  servo_JC.names.resize(2);
  servo_JC.command.resize(2);
  
  servo_JC.names[0] = "servo_front_right";
  servo_JC.command[0] = 0.0;

  servo_JC.names[1] = "servo_front_left";
  servo_JC.command[1] = 0.0;


  //servo_pub.publish(servo_JC);

  //PublishMotorCommands();
  //write_position_data();
  //publish_odometry();
}

// Finalize the controller
void MotorDrivePlugin::FiniChild(){
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

void MotorDrivePlugin::PublishMotorCommands(){

}

void MotorDrivePlugin::GetPositionCmd(){
  lock.lock();

  double vr, va;

  vr = speed_;
  va = heading_; 

  if(mode_ == DRIVE_MODE_AUTO){
    motorCmd[SERVOLEFT] = va;
    motorCmd[SERVORIGHT] = va;

    motorCmd[FRONTLEFT] = vr;
    motorCmd[FRONTRIGHT] = vr;
    motorCmd[REARLEFT] = vr;
    motorCmd[REARRIGHT] = vr;
  }else if(mode_ == DRIVE_MODE_PIVOT){

    motorCmd[FRONTLEFT] = vr;
    motorCmd[FRONTRIGHT] = vr;
    motorCmd[REARLEFT] = vr;
    motorCmd[REARRIGHT] = vr;
  }else if(mode_ == DRIVE_MODE_SPIN){
    motorCmd[SERVOLEFT] = 45;
    motorCmd[SERVORIGHT] = 45;

    motorCmd[FRONTLEFT] = -vr;
    motorCmd[FRONTRIGHT] = vr;
    motorCmd[REARLEFT] = -vr;
    motorCmd[REARRIGHT] = vr;
  }else if(mode_ == DRIVE_MODE_SIDEWAYS){
    motorCmd[SERVOLEFT] = 90;
    motorCmd[SERVORIGHT] = 90;

    motorCmd[FRONTLEFT] = -vr;
    motorCmd[FRONTRIGHT] = vr;
    motorCmd[REARLEFT] = -vr;
    motorCmd[REARRIGHT] = vr;
  }


  lock.unlock();
}

void MotorDrivePlugin::cmdVelCallback(const roberto_msgs::MotorStateConstPtr & cmd_msg){
  lock.lock();

  heading_ = cmd_msg->heading_angle;
  speed_ = cmd_msg->speed;
  acceleration_ = cmd_msg->acceleration;
  mode_ = cmd_msg->mode;

  lock.unlock();
}

void MotorDrivePlugin::QueueThread(){
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok()){
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void MotorDrivePlugin::publish_odometry(){
/*  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, "odom");
  std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

  // getting data for base_footprint to odom transform
  gazebo::physics::ModelState state(this->parent);
  math::Pose pose = state.GetPose();

  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                             current_time,
                                                             odom_frame,
                                                             base_footprint_frame));

  // publish odom topic
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;

  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;

  math::Vector3 linear = this->parent->GetWorldLinearVel();
  odom_.twist.twist.linear.x = linear.x;
  odom_.twist.twist.linear.y = linear.y;
  odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  pub_.publish(odom_);*/
}

// Update the data in the interface
void MotorDrivePlugin::write_position_data(){
  // // TODO: Data timestamp
  // pos_iface_->data->head.time = Simulator::Instance()->GetSimTime().Double();

  // pose.pos.x = odomPose[0];
  // pose.pos.y = odomPose[1];
  // pose.rot.GetYaw() = NORMALIZE(odomPose[2]);

  // pos_iface_->data->velocity.pos.x = odomVel[0];
  // pos_iface_->data->velocity.yaw = odomVel[2];

  /*math::Pose orig_pose = this->parent->GetWorldPose();

  math::Pose new_pose = orig_pose;
  new_pose.pos.x = odomPose[0];
  new_pose.pos.y = odomPose[1];
  new_pose.rot.SetFromEuler(math::Vector3(0,0,odomPose[2]));

  this->parent->SetWorldPose( new_pose );*/
}

GZ_REGISTER_MODEL_PLUGIN(MotorDrivePlugin)
}