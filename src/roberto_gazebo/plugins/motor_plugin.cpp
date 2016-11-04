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


#define MAX_ANGLE_PIVOT 13


float d, L;

float spinAngle = 0.0;
uint8_t currentMode = roberto_msgs::MotorState::DRIVE_MODE_PIVOT;
//uint32_t lastMsg;
bool initialized = false;


bool waitForServos = false;

bool spinningAutonomously = false;


namespace gazebo{

enum{
  SERVOLEFT,
  SERVORIGHT,
  FRONTRIGHT,
  FRONTLEFT,
  REARRIGHT,
  REARLEFT,
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

  ROS_INFO("loading motor_plugin");

  motorCmd[FRONTRIGHT] = 0;
  motorCmd[FRONTLEFT] = 0;
  motorCmd[REARRIGHT] = 0;
  motorCmd[REARLEFT] = 0;
  motorCmd[SERVOLEFT] = 0;
  motorCmd[SERVORIGHT] = 0;

  speed_ = 0;
  heading_ = 90;
  alive_ = true;
  mode_ = roberto_msgs::MotorState::DRIVE_MODE_PIVOT;

  L = 0.17;
  d = 0.045;



  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "motor_drive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

  ROS_INFO("starting motor drive plugin in ns: %s", this->robotNamespace.c_str());

  //tf_prefix_ = tf::getPrefixParam(*rosnode_);
  //transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<roberto_msgs::MotorState>(topicName, 1,
                                                          boost::bind(&MotorDrivePlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);


  sub_ = rosnode_->subscribe(so);
  //pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  servo_pub = rosnode_->advertise<roberto_msgs::JointCommand>("servo_joint_position_controller/command", 1);
  //motor_pub = rosnode_->advertise<roberto_msgs::JointCommand>("motor_joint_velocity_controller/command", 1);

  // Initialize the controller
  // Reset odometric pose
  /*odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;*/


  // start custom queue for diff drive
  this->callback_queue_thread_ = boost::thread(boost::bind(&MotorDrivePlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MotorDrivePlugin::UpdateChild, this));

  initialized = true;
}

// Update the controller
void MotorDrivePlugin::UpdateChild(){
  //ROS_INFO("UpdateChild");
 /* double d1, d2, d3, d4;
  double dr, da;
  double dtime = this->world->GetSimTime().Double();
  double stepTime = dtime - lastTime;
  lastTime = dtime;*/

  GetPositionCmd();


  // Distance travelled by front wheels
  /*d1 = stepTime * wheelDiameter / 2 * joints[FRONTLEFT]->GetVelocity(0);
  d2 = stepTime * wheelDiameter / 2 * joints[FRONTRIGHT]->GetVelocity(0);
  d3 = stepTime * wheelDiameter / 2 * joints[REARLEFT]->GetVelocity(0);
  d4 = stepTime * wheelDiameter / 2 * joints[REARRIGHT]->GetVelocity(0);*/


/*  dr = (d1 + d2 + d3 + d4) / 4;

  da = (d1 - d2) / wheelSeparation;   // FIX

  // Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  // Compute odometric instantaneous velocity
  odomVel[0] = dr / stepTime;
  odomVel[1] = 0.0;
  odomVel[2] = da / stepTime;*/

  servo_JC.names.resize(2);
  servo_JC.command.resize(2);
  
  servo_JC.names[0] = "servo_front_right";
  servo_JC.command[0] = motorCmd[SERVORIGHT] * (1.57 - (-0.2618)) / 90 + 1.1777;// WONDER WHERE C COMES FROM?

  servo_JC.names[1] = "servo_front_left";
  servo_JC.command[1] = motorCmd[SERVOLEFT] * ((-1.57) - 0.2618) / 90 + 2.4859;

  servo_pub.publish(servo_JC);

  //PublishMotorCommands();
 /*motor_JC.names.resize(4);
  motor_JC.command.resize(4);
  

  motor_JC.names[0] = "front_right_wheel_hinge";
  motor_JC.command[0] = 0.0;

  motor_JC.names[1] = "front_left_wheel_hinge";
  motor_JC.command[1] = 1.0;

  motor_JC.names[2] = "rear_right_wheel_hinge";
  motor_JC.command[2] = 0.0;

  motor_JC.names[3] = "rear_left_wheel_hinge";
  motor_JC.command[3] = 0.0;

  motor_pub.publish(motor_JC);*/

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

  if(!initialized)
    return;
  //lastMsg = millis();
  uint8_t intMode = mode_;
  if(intMode == roberto_msgs::MotorState::DRIVE_MODE_AUTO){

  }

  /*if(currentMode != intMode){
    waitForServos = true;
  }*/

  if(intMode == roberto_msgs::MotorState::DRIVE_MODE_PIVOT){
    float speedMult[2] = {1,1};
    float angle[2] = {0, 0};
    if (heading_ != 0){
      float R = 0.05/sin(heading_*M_PI/360);
      speedMult[0] = (2*R)/(2*R - (L/2 + d));
      speedMult[1] = (2*R)/(2*R + (L/2 + d));
      //angle[0] = atan(L/(B+R));   // B=L as robot is quadratic
      angle[0] = heading_ + atan(L/(L+R));
      angle[1] = heading_ + atan(L/(L-R));
    }else{
      angle[0] = 0;
      angle[1] = 0;
    }

    angle[0] = angle[0] > MAX_ANGLE_PIVOT? MAX_ANGLE_PIVOT : ( angle[0] < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : angle[0]);
    angle[1] = angle[1] > MAX_ANGLE_PIVOT? MAX_ANGLE_PIVOT : ( angle[1] < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : angle[1]);

    if(heading_ > 0){
      motorCmd[SERVOLEFT] = angle[0]+135;
      motorCmd[SERVORIGHT] = -angle[1]+135;
    }else{
      motorCmd[SERVOLEFT] = angle[1]+135;
      motorCmd[SERVORIGHT] = -angle[0]+135;
    }
    motorCmd[FRONTRIGHT] = speed_*speedMult[0];
    motorCmd[FRONTLEFT] = speed_*speedMult[1];
    motorCmd[REARLEFT] = speed_*speedMult[1];
    motorCmd[REARRIGHT] = speed_*speedMult[0];

  }else if(intMode == roberto_msgs::MotorState::DRIVE_MODE_SPIN){
    if(spinAngle == 0){
      if(heading_ != 0){
        spinningAutonomously = true;
        spinAngle = heading_;
      }else{
        spinningAutonomously = false;
      }
      motorCmd[SERVOLEFT] = 90;
      motorCmd[SERVORIGHT] = 90;

      motorCmd[FRONTRIGHT] = speed_;
      motorCmd[FRONTLEFT] = -speed_;
      motorCmd[REARLEFT] = -speed_;
      motorCmd[REARRIGHT] = speed_;
    }

  }else if(intMode == roberto_msgs::MotorState::DRIVE_MODE_SIDEWAYS){
    motorCmd[SERVOLEFT] = 45;
    motorCmd[SERVORIGHT] = 45;

    motorCmd[FRONTRIGHT] = speed_;  //FR
    motorCmd[FRONTLEFT] = -speed_; //FL
    motorCmd[REARLEFT] = speed_; //RL
    motorCmd[REARRIGHT] = -speed_;  //RR
  }
  currentMode = intMode;


  lock.unlock();
}

void MotorDrivePlugin::cmdVelCallback(const roberto_msgs::MotorStateConstPtr & cmd_msg){
  lock.lock();
  //ROS_INFO("Callback");
  heading_ = cmd_msg->heading_angle;
  speed_ = cmd_msg->speed;
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