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

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <algorithm>
#include <assert.h>

#include <motor_plugin.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>


#define MAX_ANGLE_PIVOT 13

#define KP_POS 15
#define KI_POS 10
#define KD_POS 0.5

#define KP_VEL 0.005
#define KI_VEL 0
#define KD_VEL 0


namespace gazebo
{

enum{
  FRONTRIGHT,
  FRONTLEFT,
  REARLEFT,
  REARRIGHT,
  SERVORIGHT,
  SERVOLEFT,
  SERVORIGHTMIMIC,
  SERVOLEFTMIMIC
};


MotorDrivePlugin::MotorDrivePlugin() {}

// Destructor
MotorDrivePlugin::~MotorDrivePlugin() {
    //event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
    
    
}

// Load the controller
void MotorDrivePlugin::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "MotorDrivePlugin" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );

    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( L, "widthBetweenMotorPivotPoints", 0.17 );
    gazebo_ros_->getParameter<double> ( d, "motorPivotPointToWheel", 0.045 );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );


    joints_.resize ( 8 );
    joints_[SERVOLEFT] = gazebo_ros_->getJoint ( parent, "servoLeftJoint", "servo_front_left" );
    joints_[SERVORIGHT] = gazebo_ros_->getJoint ( parent, "servoRightJoint", "servo_front_right" );
    joints_[SERVOLEFTMIMIC] = gazebo_ros_->getJoint ( parent, "servoLeftMimicJoint", "servo_rear_left" );
    joints_[SERVORIGHTMIMIC] = gazebo_ros_->getJoint ( parent, "servoRightMimicJoint", "servo_rear_right" );
    
    joints_[FRONTLEFT] = gazebo_ros_->getJoint ( parent, "frontLeftJoint", "front_left_wheel_hinge" );
    joints_[FRONTRIGHT] = gazebo_ros_->getJoint ( parent, "frontRightJoint", "front_right_wheel_hinge" );
    joints_[REARLEFT] = gazebo_ros_->getJoint ( parent, "rearLeftJoint", "rear_left_wheel_hinge" );
    joints_[REARRIGHT] = gazebo_ros_->getJoint ( parent, "rearRightJoint", "rear_right_wheel_hinge" );

    pid.resize(8);
    pid[FRONTLEFT] = common::PID(KP_VEL, KI_VEL, KD_VEL);
    pid[FRONTRIGHT] = common::PID(KP_VEL, KI_VEL, KD_VEL);
    pid[REARLEFT] = common::PID(KP_VEL, KI_VEL, KD_VEL);
    pid[REARRIGHT] = common::PID(KP_VEL, KI_VEL, KD_VEL);

    pid[SERVOLEFT] = common::PID(KP_POS, KI_POS, KD_POS);
    pid[SERVORIGHT] = common::PID(KP_POS, KI_POS, KD_POS);
    pid[SERVOLEFTMIMIC] = common::PID(KP_POS, KI_POS, KD_POS);
    pid[SERVORIGHTMIMIC] = common::PID(KP_POS, KI_POS, KD_POS);


    for ( int i = 0; i < joints_.size(); i++ ) {
#if GAZEBO_MAJOR_VERSION > 2
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
#else
        joints_[i]->SetMaxForce ( 0, wheel_torque );
#endif   
    }

    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN("MotorDrivePlugin Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) 
        this->update_period_ = 1.0 / this->update_rate_;
    else 
        this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->GetSimTime();

    // Initialize velocity stuff
    motorCmd[SERVOLEFT] = 0;
    motorCmd[SERVORIGHT] = 0;
    motorCmd[FRONTLEFT] = 0;
    motorCmd[FRONTRIGHT] = 0;
    motorCmd[REARLEFT] = 0;
    motorCmd[REARRIGHT] = 0;

    // Initialize variables
    speed_ = 0;
    heading_ = 0;
    alive_ = true;
    mode_ = roberto_msgs::MotorState::DRIVE_MODE_PIVOT;
    currentMode = roberto_msgs::MotorState::DRIVE_MODE_PIVOT;
    spinAngle = 0;
    spinningAutonomously = false;
    waitForServos = false;

    math::Pose pose = parent->GetWorldPose();
    pose_encoder_.x = pose.pos.x;
    pose_encoder_.y = pose.pos.y;
    pose_encoder_.theta = pose.rot.GetYaw();

    l = sqrt(pow((L/2),2)*2);

    alpha[FRONTRIGHT] = -M_PI/4;
    alpha[FRONTLEFT ] = M_PI/4;
    alpha[REARLEFT  ] = 3*M_PI/4;
    alpha[REARRIGHT ] = -3*M_PI/4;



    if (this->publishWheelJointState_){
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO("%s: Advertise joint_states!", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO("%s: Try to subscribe to \"%s\"!", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<roberto_msgs::MotorState>(command_topic_, 1,
                boost::bind(&MotorDrivePlugin::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO("%s: Subscribe to \"%s\"!", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      odometry_publisher__ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>("odom_ground_truth", 1);
      ROS_INFO("%s: Advertise odom to \"%s\" !", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    // start custom queue for motor drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &MotorDrivePlugin::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &MotorDrivePlugin::UpdateChild, this ) );
}

void MotorDrivePlugin::Init() {
    gazebo::ModelPlugin::Init();
}


void MotorDrivePlugin::Reset() {
    gazebo::ModelPlugin::Reset();
    last_update_time_ = parent->GetWorld()->GetSimTime();
    math::Pose pose = parent->GetWorldPose();
    pose_encoder_.x = pose.pos.x;
    pose_encoder_.y = pose.pos.y;
    pose_encoder_.theta = pose.rot.GetYaw();
    speed_ = 0;
    heading_ = 0;
    alive_ = true;
    mode_ = roberto_msgs::MotorState::DRIVE_MODE_PIVOT;

    for ( int i = 0; i < joints_.size(); i++ ) {
#if GAZEBO_MAJOR_VERSION > 2
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
#else
        joints_[i]->SetMaxForce ( 0, wheel_torque );
#endif
    }
}

void MotorDrivePlugin::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );
    joint_state_.velocity.resize ( joints_.size() );
    joint_state_.effort.resize ( joints_.size() );

    for ( int i = 0; i < joints_.size(); i++ ) {
        joint_state_.name[i] = joints_[i]->GetName();
        joint_state_.position[i] = joints_[i]->GetAngle ( 0 ).Radian () ;
        joint_state_.velocity[i] = joints_[i]->GetVelocity ( 0 );
        joint_state_.effort[i] = joints_[i]->GetForce ( 0 );
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void MotorDrivePlugin::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < joints_.size(); i++ ) {
        
        std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());
        
        math::Pose poseWheel = joints_[i]->GetChild()->GetRelativePose() - joints_[i]->GetParent()->GetRelativePose();

        tf::Quaternion qt ( poseWheel.rot.x, poseWheel.rot.y, poseWheel.rot.z, poseWheel.rot.w );
        tf::Vector3 vt ( poseWheel.pos.x, poseWheel.pos.y, poseWheel.pos.z );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}

// Update the controller
void MotorDrivePlugin::UpdateChild()
{
  
    /* force reset SetMaxForce since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than MotorDrivePlugin::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    
    for ( int i = 0; i < 8; i++ ) {
#if GAZEBO_MAJOR_VERSION > 2
      if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
#else
      if ( fabs(wheel_torque -joints_[i]->GetMaxForce ( 0 )) > 1e-6 ) {
        joints_[i]->SetMaxForce ( 0, wheel_torque );
#endif
      }
    }
*/

    if ( odom_source_ == ENCODER ) 
        UpdateOdometryEncoder();

    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        GetPositionCmd();

        double current_speed[8];

        current_speed[FRONTLEFT ] = joints_[FRONTLEFT ]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[FRONTRIGHT] = joints_[FRONTRIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[REARLEFT  ] = joints_[REARLEFT  ]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[REARRIGHT ] = joints_[REARRIGHT ]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[SERVOLEFT ] = joints_[SERVOLEFT ]->GetAngle ( 0 ).Radian();
        current_speed[SERVORIGHT ] = joints_[SERVORIGHT ]->GetAngle ( 0 ).Radian();
        current_speed[SERVOLEFTMIMIC ] = joints_[SERVOLEFTMIMIC ]->GetAngle ( 0 ).Radian();
        current_speed[SERVORIGHTMIMIC ] = joints_[SERVORIGHTMIMIC ]->GetAngle ( 0 ).Radian();

        if ( wheel_accel == 0 ||
                ( ( fabs ( motorCmd[FRONTLEFT ] - current_speed[FRONTLEFT ] ) < 0.01 ) &&
                  ( fabs ( motorCmd[FRONTRIGHT] - current_speed[FRONTRIGHT] ) < 0.01 ) &&
                  ( fabs ( motorCmd[REARLEFT  ] - current_speed[REARLEFT  ] ) < 0.01 ) &&
                  ( fabs ( motorCmd[REARRIGHT ] - current_speed[REARRIGHT ] ) < 0.01 ) ) ) {
            //if max_accel == 0, or target speed is reached
    	    wheel_speed_instr_[FRONTLEFT ] = motorCmd[FRONTLEFT ];
            wheel_speed_instr_[FRONTRIGHT] = motorCmd[FRONTRIGHT];
            wheel_speed_instr_[REARLEFT  ] = motorCmd[REARLEFT  ];
    	    wheel_speed_instr_[REARRIGHT ] = motorCmd[REARRIGHT ];
        } else {
            if ( motorCmd[FRONTLEFT ] >= current_speed[FRONTLEFT ] )
                wheel_speed_instr_[FRONTLEFT ] += fmin ( motorCmd[FRONTLEFT ]-current_speed[FRONTLEFT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[FRONTLEFT ] += fmax ( motorCmd[FRONTLEFT ]-current_speed[FRONTLEFT], -wheel_accel * seconds_since_last_update );


            if ( motorCmd[FRONTRIGHT] >  current_speed[FRONTRIGHT] )
                wheel_speed_instr_[FRONTRIGHT] += fmin ( motorCmd[FRONTRIGHT]-current_speed[FRONTRIGHT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[FRONTRIGHT] += fmax ( motorCmd[FRONTRIGHT]-current_speed[FRONTRIGHT], -wheel_accel * seconds_since_last_update );

            if ( motorCmd[REARLEFT] >  current_speed[REARLEFT] )
                wheel_speed_instr_[REARLEFT] += fmin ( motorCmd[REARLEFT]-current_speed[REARLEFT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[REARLEFT] += fmax ( motorCmd[REARLEFT]-current_speed[REARLEFT], -wheel_accel * seconds_since_last_update );

            if ( motorCmd[REARRIGHT] >  current_speed[REARRIGHT] )
                wheel_speed_instr_[REARRIGHT] += fmin ( motorCmd[REARRIGHT]-current_speed[REARRIGHT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[REARRIGHT] += fmax ( motorCmd[REARRIGHT]-current_speed[REARRIGHT], -wheel_accel * seconds_since_last_update );

             //ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[FRONTLEFT ], motorCmd[FRONTLEFT ]);
             //ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[FRONTRIGHT], motorCmd[FRONTRIGHT]);
        }
        wheel_applied_vel[FRONTLEFT ] = wheel_speed_instr_[FRONTLEFT ] / ( wheel_diameter_ / 2.0 );
        wheel_applied_vel[FRONTRIGHT] = wheel_speed_instr_[FRONTRIGHT] / ( wheel_diameter_ / 2.0 );
        wheel_applied_vel[REARLEFT  ] = wheel_speed_instr_[REARLEFT  ] / ( wheel_diameter_ / 2.0 );
	    wheel_applied_vel[REARRIGHT ] = wheel_speed_instr_[REARRIGHT ] / ( wheel_diameter_ / 2.0 );

        last_update_time_ = current_time;

        // TODO: Calculate these parameters from servo joint limits!
        joints_[SERVOLEFT      ]->SetParam ( "vel",  0, pid[SERVOLEFT].Update( current_speed[SERVOLEFT] - (((180-motorCmd[SERVOLEFT]) * -0.0176134615) + 0.792605913 ) , seconds_since_last_update));
        joints_[SERVORIGHT     ]->SetParam ( "vel",  0, pid[SERVORIGHT].Update( current_speed[SERVORIGHT] - ((motorCmd[SERVORIGHT] * -0.0176134615) + 2.37781677) , seconds_since_last_update));
        joints_[SERVOLEFTMIMIC ]->SetParam ( "vel",  0, pid[SERVOLEFTMIMIC].Update( current_speed[SERVOLEFTMIMIC] + (((180-motorCmd[SERVOLEFT]) * -0.0176134615) + 0.792605913 ) , seconds_since_last_update));
        joints_[SERVORIGHTMIMIC]->SetParam ( "vel",  0, pid[SERVORIGHTMIMIC].Update( current_speed[SERVORIGHTMIMIC] + ((motorCmd[SERVORIGHT] * -0.0176134615) + 2.37781677) , seconds_since_last_update));

        // (sr, sl) output of low pass filtered servo references, compare these to the setpoint, in order to see whether filter has reached setpoint
        /*if(waitForServos && ((int)sr == (int)servo_right->getReference() && (int)sl == (int)servo_left->getReference())){

            waitForServos = false;*/
#if GAZEBO_MAJOR_VERSION > 2
            joints_[FRONTLEFT ]->SetParam ( "vel", 0, wheel_applied_vel[FRONTLEFT ] );
            joints_[FRONTRIGHT]->SetParam ( "vel", 0, wheel_applied_vel[FRONTRIGHT] );
            joints_[REARLEFT  ]->SetParam ( "vel", 0, wheel_applied_vel[REARLEFT  ] );
            joints_[REARRIGHT ]->SetParam ( "vel", 0, wheel_applied_vel[REARRIGHT ] );

            /*joints_[FRONTLEFT ]->SetParam ( "vel",  0, pid[FRONTLEFT  ].Update( wheel_applied_vel[FRONTLEFT ], seconds_since_last_update));
            joints_[FRONTRIGHT]->SetParam ( "vel",  0, pid[FRONTRIGHT ].Update( wheel_applied_vel[FRONTRIGHT], seconds_since_last_update));
            joints_[REARLEFT  ]->SetParam ( "vel",  0, pid[REARLEFT   ].Update( wheel_applied_vel[REARLEFT  ], seconds_since_last_update));
            joints_[REARRIGHT ]->SetParam ( "vel",  0, pid[REARRIGHT  ].Update( wheel_applied_vel[REARRIGHT ], seconds_since_last_update));
*/
#else
            joints_[FRONTLEFT ]->SetVelocity ( 0, wheel_applied_vel[FRONTLEFT ] );
            joints_[FRONTRIGHT]->SetVelocity ( 0, wheel_applied_vel[FRONTRIGHT] );
            joints_[REARLEFT  ]->SetVelocity ( 0, wheel_applied_vel[REARLEFT  ] );
            joints_[REARRIGHT ]->SetVelocity ( 0, wheel_applied_vel[REARRIGHT ] );
#endif
        /*}else{

#if GAZEBO_MAJOR_VERSION > 2
            joints_[FRONTLEFT ]->SetParam ( "vel", 0, 0.0 );
            joints_[FRONTRIGHT]->SetParam ( "vel", 0, 0.0 );
            joints_[REARLEFT  ]->SetParam ( "vel", 0, 0.0 );
            joints_[REARRIGHT ]->SetParam ( "vel", 0, 0.0 );
#else
            joints_[FRONTLEFT ]->SetVelocity ( 0, 0.0 );
            joints_[FRONTRIGHT]->SetVelocity ( 0, 0.0 );
            joints_[REARLEFT  ]->SetVelocity ( 0, 0.0 );
            joints_[REARRIGHT ]->SetVelocity ( 0, 0.0 );
#endif
        }*/



        // Check autonomous spinning.
        // TODO: Convert this to be an actual controller based approach
        /*if(currentMode == roberto_msgs::MotorState::DRIVE_MODE_SPIN && spinningAutonomously && !waitForServos){
            if(spinAngle > 0){
                spinAngle -= (abs(theta_dot)*180/M_PI) * seconds_since_last_update;
            }else{
                spinAngle = 0;
                motorCmd[0] = 0.0;
                motorCmd[1] = 0.0;
                motorCmd[2] = 0.0;
                motorCmd[3] = 0.0;
            }
        }*/

        

    }
}


void MotorDrivePlugin::GetPositionCmd(){
  boost::mutex::scoped_lock scoped_lock ( lock );

  uint8_t intMode = mode_;
  if(intMode == roberto_msgs::MotorState::DRIVE_MODE_AUTO){

  }

  if(currentMode != intMode){
    waitForServos = true;
  }

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

}

void MotorDrivePlugin::FiniChild(){
    odometry_publisher_.shutdown();
    joint_state_publisher_.shutdown();
    cmd_vel_subscriber_.shutdown();
    
    queue_.clear();
    queue_.disable();
    alive_ = false;
    callback_queue_thread_.join();
}

void MotorDrivePlugin::cmdVelCallback ( const roberto_msgs::MotorStateConstPtr & cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    heading_ = cmd_msg->heading_angle;
    speed_ = -cmd_msg->speed;
    mode_ = cmd_msg->mode;
}

void MotorDrivePlugin::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void MotorDrivePlugin::UpdateOdometryEncoder()
{
    double actual_speeds[4];
    actual_speeds[FRONTRIGHT] = -joints_[FRONTRIGHT]->GetVelocity ( 0 );
    actual_speeds[FRONTLEFT ] = -joints_[FRONTLEFT ]->GetVelocity ( 0 );
    actual_speeds[REARLEFT  ] = -joints_[REARLEFT  ]->GetVelocity ( 0 );
    actual_speeds[REARRIGHT ] = -joints_[REARRIGHT ]->GetVelocity ( 0 );

    double actual_angles[4];
    actual_angles[FRONTRIGHT] =  joints_[SERVORIGHT]->GetAngle ( 0 ).Radian();// - M_PI/2;
    actual_angles[FRONTLEFT ] = joints_[SERVOLEFT ]->GetAngle ( 0 ).Radian();// + M_PI/2;
    actual_angles[REARLEFT  ] =  joints_[SERVOLEFTMIMIC ]->GetAngle ( 0 ).Radian();// + M_PI/2;
    actual_angles[REARRIGHT ] = joints_[SERVORIGHTMIMIC]->GetAngle ( 0 ).Radian();// - M_PI/2;

    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double wheel_displacement[4];
    wheel_displacement[FRONTRIGHT] = actual_speeds[FRONTRIGHT] * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    wheel_displacement[FRONTLEFT ] = actual_speeds[FRONTLEFT ] * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    wheel_displacement[REARLEFT  ] = actual_speeds[REARLEFT  ] * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    wheel_displacement[REARRIGHT ] = actual_speeds[REARRIGHT ] * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;

    

    double b = L+d;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    /*double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double theta = ( sl - sr ) /b;


    double dx = ( sl + sr ) /2.0 * cos ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dy = ( sl + sr ) /2.0 * sin ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dtheta = ( sl - sr ) /b;

    */

    float dx = 0;
    float dy = 0;
    float dtheta = 0;

    // [FRONTRIGHT, FRONTLEFT, REARLEFT, REARRIGHT]
    for(int i = 0; i < 4; i++){


        dx += cos(pose_encoder_.theta)*wheel_displacement[i];
        dy += sin(pose_encoder_.theta)*wheel_displacement[i];

        //printf("%f\t",actual_angles[i]);

        
        float motorAngle = actual_angles[i];// - M_PI/2;
        if(i==1 || i==2){
            motorAngle += M_PI;
        }
        

        float deltaX = l*cos(alpha[i]) + d*cos(motorAngle);
        float deltaY = l*sin(alpha[i]) + d*sin(motorAngle);

        float deltaNorm = sqrt(deltaX*deltaX + deltaY*deltaY);
        float deltaXNorm = deltaX/deltaNorm;
        float deltaYNorm = deltaY/deltaNorm;

        /*
        [ deltaXNorm
          deltaYNorm ]      <-- Unit vector from robot center to center of wheel 'i'

        */


        float wAngle = actual_angles[i] - M_PI/2;
        /*if(i==1 || i==2){
            wAngle += M_PI;
        }*/
        float wX = -sin(wAngle);
        float wY = cos(wAngle);

        /*
        [ wX
          wY ]      <-- Unit vector - somthing somthing!

        */

        // dtheta += (dot(deltaNorm, w) * wX)/norm(deltaNorm) * wheel_displacement[i]

        dtheta += (( (deltaXNorm*wX + deltaYNorm*wY) * wX)/deltaNorm)*wheel_displacement[i];
        //printf("%f \t\t",wheel_displacement[i]);
    }
    //printf("%f\t\n",seconds_since_last_update);
    dx /= 4;
    dy /= 4;
    dtheta /= 4;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    math::Pose pose = parent->GetWorldPose();
    pose_encoder_.theta = pose.rot.GetYaw();


    double w = dtheta/seconds_since_last_update;
    //double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom__.pose.pose.position.x = vt.x();
    odom__.pose.pose.position.y = vt.y();
    odom__.pose.pose.position.z = vt.z();

    odom__.pose.pose.orientation.x = qt.x();
    odom__.pose.pose.orientation.y = qt.y();
    odom__.pose.pose.orientation.z = qt.z();
    odom__.pose.pose.orientation.w = qt.w();

    odom__.twist.twist.angular.z = w;
    odom__.twist.twist.linear.x = dx/seconds_since_last_update;
    odom__.twist.twist.linear.y = dy/seconds_since_last_update;
}

void MotorDrivePlugin::publishOdometry ( double step_time )
{
   
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    math::Pose pose = parent->GetWorldPose();
    //if ( odom_source_ == WORLD) {
    // getting data form gazebo world
    qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
    vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );
    //}

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom__.pose.pose.orientation.x, odom__.pose.pose.orientation.y, odom__.pose.pose.orientation.z, odom__.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom__.pose.pose.position.x, odom__.pose.pose.position.y, odom__.pose.pose.position.z );

    }

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
    odom_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;

    if (this->publish_tf_){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }

    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    odom__.pose.covariance[0] = 0.00001;
    odom__.pose.covariance[7] = 0.00001;
    odom__.pose.covariance[14] = 1000000000000.0;
    odom__.pose.covariance[21] = 1000000000000.0;
    odom__.pose.covariance[28] = 1000000000000.0;
    odom__.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odom__.header.stamp = current_time;
    odom__.header.frame_id = odom_frame;
    odom__.child_frame_id = base_footprint_frame;

    if ( odom_source_ == ENCODER ) {
        odometry_publisher_.publish ( odom__ );
        odometry_publisher__.publish ( odom_ );
    }else{
        odometry_publisher_.publish ( odom_ );
    }

}

GZ_REGISTER_MODEL_PLUGIN ( MotorDrivePlugin )
}
