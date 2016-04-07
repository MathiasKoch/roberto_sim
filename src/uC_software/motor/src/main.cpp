#include <stm32f10x.h>

#include <stdio.h>
#include <math.h>

#include "motor.h"
#include "motorSettings.h"
#include "led.h"
#include "encoder.h"
#include "stm32_time.h"


#include <ros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <roberto_msgs/MotorState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>

#define CMDMSGTIMEOUT 500 // millis

/* Private function prototypes -----------------------------------------------*/



__IO uint8_t Rx_Idx = 0x00;
uint8_t slaveAddress;
uint8_t RxBuffer [RXBUFFERSIZE] = {0};
__IO uint8_t NumberOfByteToReceive = 0x00;
__IO uint8_t GenerateStartStatus = 0x00;

motor *servo_left;
motor *servo_right;
motor *front_right;
motor *front_left;
motor *rear_right;
motor *rear_left;

float d, L;

float motorCmd[4];
uint8_t currentMode;
uint32_t lastMsg;

ros::NodeHandle nh;

__IO bool shuttingDown = false;
__IO bool waitForServos = false;

void led_cb( const std_msgs::UInt8& cmd_msg){
  led_set(cmd_msg.data);
}



void motor_cb( const roberto_msgs::MotorState& cmd_msg){
  lastMsg = millis();
  uint8_t intMode = cmd_msg.mode;
  if(intMode == cmd_msg.DRIVE_MODE_AUTO){

  }

  if(currentMode != intMode){
    waitForServos = true;
  }

  if(intMode == cmd_msg.DRIVE_MODE_PIVOT){
    float angle[2] = {0, 0};
    float speedMult = 1;
    if (cmd_msg.heading_angle != 0){
      float R = (1+cmd_msg.heading_angle*cmd_msg.heading_angle)/(2*cmd_msg.heading_angle);
      //angle[0] = atan(L/(B+R));   // B=L as robot is quadratic
      angle[0] = atan(L/(L+R));
      angle[1] = atan(L/(L-R));
    }
    motorCmd[0] = cmd_msg.speed*speedMult;
    motorCmd[1] = cmd_msg.speed*speedMult;
    motorCmd[2] = cmd_msg.speed*speedMult;
    motorCmd[3] = cmd_msg.speed*speedMult;

    angle[0] = angle[0] > MAX_ANGLE_PIVOT? MAX_ANGLE_PIVOT : ( angle[0] < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : angle[0]);
    angle[1] = angle[1] > MAX_ANGLE_PIVOT? MAX_ANGLE_PIVOT : ( angle[1] < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : angle[1]);

    if(cmd_msg.heading_angle > 0){
      servo_left->setReference(angle[0]);
      servo_right->setReference(angle[1]);

    }else{
      servo_left->setReference(angle[1]);
      servo_right->setReference(angle[0]);
    }

  }else if(intMode == cmd_msg.DRIVE_MODE_SPIN){
    float h = 45;
    servo_left->setReference(h);
    servo_right->setReference(h);

    motorCmd[0] = cmd_msg.speed;
    motorCmd[1] = -cmd_msg.speed;
    motorCmd[2] = -cmd_msg.speed;
    motorCmd[3] = cmd_msg.speed;

  }else if(intMode == cmd_msg.DRIVE_MODE_SIDEWAYS){
    float h = 90;
    servo_left->setReference(h);
    servo_right->setReference(h);

    motorCmd[0] = cmd_msg.speed;
    motorCmd[1] = -cmd_msg.speed;
    motorCmd[2] = cmd_msg.speed;
    motorCmd[3] = -cmd_msg.speed;
  }
  currentMode = intMode;
}


ros::Subscriber<roberto_msgs::MotorState> motor_sub("throttled_joy_vel", &motor_cb);
//ros::Subscriber<sensor_msgs::Joy> motor_sub("joy", &motor_cb);
ros::Subscriber<std_msgs::UInt8> led_sub("led", &led_cb);

geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster odom_broadcaster;


nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

int main(){
  /* System Clocks Configuration */
  RCC_Configuration();

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  I2C1_Init();
  SysTick_Init();
  DEBUG_Init();
  SHUTDOWN_Init();
  LED_Init();

  /*str_msg.data = (float *)malloc(sizeof(float)*4);
  str_msg.data_length = 4;*/

  nh.initNode();

  nh.subscribe(motor_sub);
  nh.subscribe(led_sub);
  nh.advertise(odom_pub);
  odom_broadcaster.init(nh);

  while(!nh.connected()){
    nh.spinOnce();
  }

  shuttingDown = false;

  

  float KP;
  if(!nh.getParam("serial_node/KP", &KP, 1)){
    KP = 5000;
  }
  float KI;
  if(!nh.getParam("serial_node/KI", &KI, 1)){
    KI = 5000;
  }
  float KD;
  if(!nh.getParam("serial_node/KD", &KD, 1)){
    KD = 4;
  }
  float integralSaturation;
  if(!nh.getParam("serial_node/integralSaturation", &integralSaturation, 1)){
    integralSaturation = 10000;
  }
  float wheelRadius;
  if(!nh.getParam("serial_node/wheelRadius", &wheelRadius, 1)){
    wheelRadius = 0.04;
  }

  if(!nh.getParam("serial_node/widthBetweenMotorPivotPoints", &L, 1)){
    L = 0.17;
  }

  if(!nh.getParam("serial_node/motorPivotPointToWheel", &d, 1)){
    d = 0.06;
  }

  delay(1000);

  motorSettings SL(MOTOR_TYPE_SERVO, "servo_left", TIM4, 4);
  SL.m_ServoPin = GPIO_Pin_9;
  SL.m_ServoPort = GPIOB;

  motorSettings SR(MOTOR_TYPE_SERVO, "servo_right", TIM4, 3);
  SR.m_ServoPin = GPIO_Pin_8;
  SR.m_ServoPort = GPIOB;

  motorSettings FR(MOTOR_TYPE_DC_MOTOR, "front_right", TIM1, 1);
  FR.setDCPins(GPIO_Pin_13, GPIOC, GPIO_Pin_14, GPIOC,
              GPIO_Pin_15, GPIOC, GPIO_Pin_0, GPIOA,
              GPIO_Pin_8, GPIOA);
  FR.encoderAddr = (0x10 | 0x08);
  FR.setRegulator(KP,KI,KD,integralSaturation);
  FR.wheelRadius = wheelRadius;

  motorSettings FL(MOTOR_TYPE_DC_MOTOR, "front_left", TIM3, 4);
  FL.setDCPins(GPIO_Pin_2,GPIOB, GPIO_Pin_5,GPIOA,
              GPIO_Pin_4, GPIOA,GPIO_Pin_10,GPIOB,
              GPIO_Pin_1, GPIOB);
  FL.encoderAddr = (0x10);
  FL.setRegulator(KP,KI,KD,integralSaturation);
  FL.wheelRadius = wheelRadius;

  motorSettings RL(MOTOR_TYPE_DC_MOTOR, "rear_left", TIM2, 4);
  RL.setDCPins(GPIO_Pin_11, GPIOB, GPIO_Pin_12, GPIOB,
    GPIO_Pin_13, GPIOB, GPIO_Pin_14, GPIOB,
    GPIO_Pin_3, GPIOA);
  RL.encoderAddr = (0x10 | 0x04);
  RL.setRegulator(KP,KI,KD,integralSaturation);
  RL.wheelRadius = wheelRadius;

  motorSettings RR(MOTOR_TYPE_DC_MOTOR, "rear_right", TIM1, 4);
  RR.setDCPins(GPIO_Pin_15, GPIOB, GPIO_Pin_12,GPIOA,
              GPIO_Pin_15, GPIOA, GPIO_Pin_3, GPIOB,
              GPIO_Pin_11, GPIOA);
  RR.encoderAddr = (0x10 | 0x04 | 0x08);
  RR.setRegulator(KP,KI,KD,integralSaturation);
  RR.wheelRadius = wheelRadius;

  char error[150];
  servo_left = motor::createMotor(&SL);
  if(!servo_left->motorInit()){
    sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", servo_left->motorName());
    nh.logerror(error);
  }

  servo_right = motor::createMotor(&SR);
  if(!servo_right->motorInit()){
    sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", servo_right->motorName());
    nh.logerror(error);
  }

  front_left = motor::createMotor(&FL);
  if(!front_left->motorInit()){
    sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", front_left->motorName());
    nh.logerror(error);
  }

  front_right = motor::createMotor(&FR);
  if(!front_right->motorInit()){
    sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", front_right->motorName());
    nh.logerror(error);
  }

  rear_left = motor::createMotor(&RL);
  if(!rear_left->motorInit()){
    sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", rear_left->motorName());
    nh.logerror(error);
  }

  rear_right = motor::createMotor(&RR);
  if(!rear_right->motorInit()){
    sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", rear_right->motorName());
    nh.logerror(error);
  }
  
  
  led_set(0);

  servo_left->setReference(0);
  servo_right->setReference(0);
  front_right->setReference(0);
  front_left->setReference(0);
  rear_right->setReference(0);
  rear_left->setReference(0);

  bool connected = false;

  int cnt = 0;
  float start_time;
  float dt = 0.02;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  float l = sqrt(pow((L/2),2)*2);

  float alpha[4] = {0,0,0,0};


  while (1){
    nh.spinOnce();
    start_time = ((int)millis()) / 1000.0;

    if(shuttingDown){
      connected = false;
      // TODO: Advertise shutdown??
    }else{
      connected = nh.connected();
      if(cnt++%10==0)
        debug_toggle();
    }

    auto sl = servo_left->update(0.1, connected);
    auto sr = servo_right->update(0.1, connected);

    if(waitForServos | lastMsg + CMDMSGTIMEOUT > start_time){
      if(std::get<2>(sr) == (int)servo_right->getReference() && std::get<2>(sl) == (int)servo_left->getReference()){
        waitForServos = false;
        front_right->setReference(motorCmd[0]);
        front_left->setReference(motorCmd[1]);
        rear_left->setReference(motorCmd[2]);
        rear_right->setReference(motorCmd[3]);
      }else{
        front_right->setReference(0);
        front_left->setReference(0);
        rear_left->setReference(0);
        rear_right->setReference(0);
      }
    }
    
    auto fr = front_right->update(0.1, connected);
    auto fl = front_left->update(0.1, connected);
    auto rl = rear_left->update(0.1, connected);
    auto rr = rear_right->update(0.1, connected);

    if(std::get<3>(fr) == 65535){
      sprintf(error, "Encoder timeout: %s - [FAIL]", front_right->motorName());
      nh.logerror(error);
    }
    if(std::get<3>(fl) == 65535){
      sprintf(error, "Encoder timeout: %s - [FAIL]", front_left->motorName());
      nh.logerror(error);
    }
    if(std::get<3>(rl) == 65535){
      sprintf(error, "Encoder timeout: %s - [FAIL]", rear_left->motorName());
      nh.logerror(error);
    }
    if(std::get<3>(rr) == 65535){
      sprintf(error, "Encoder timeout: %s - [FAIL]", rear_right->motorName());
      nh.logerror(error);
    }

    float speeds[4] = {std::get<0>(fr), std::get<0>(fl), std::get<0>(rl), std::get<0>(rr)};

    // TODO: Correct these to fit!
    float angles[4] = {std::get<0>(sl), std::get<0>(sr), std::get<0>(sl), std::get<0>(sr)};

    int i;
    float x_dot = 0;
    float y_dot = 0;
    float theta_dot = 0;
    for(i = 0; i < 4; i++){
      // TODO: Calculate beta correctly
      float beta = angles[i];//*PWM2DEG;
      x_dot += sin(alpha[i]+beta)*speeds[i];
      y_dot += cos(alpha[i]+beta)*speeds[i];
      theta_dot += cos(beta - atan((l*cos(alpha[i]+d*cos(alpha[i] + beta)))/(l*sin(alpha[i] + d*sin(alpha[i] + beta)))))*speeds[i];
    }
    x_dot /= 4;
    y_dot /= 4;
    theta_dot /= 4;



    double delta_x = (x_dot * cos(th) - y_dot * sin(th)) * dt;
    double delta_y = (x_dot * sin(th) + y_dot * cos(th)) * dt;
    double delta_th = theta_dot * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;


    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

    //first, we'll publish the transform over tf
    odom_trans.header.stamp = ros::Time(start_time,0);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = ros::Time(start_time,0);
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = x_dot;
    odom.twist.twist.linear.y = y_dot;
    odom.twist.twist.angular.z = theta_dot;

    //publish the message
    odom_pub.publish(&odom);

    //delay(20);
    if((((int)millis()) / 1000.0) - start_time >= dt){
      nh.logerror("Main loop running slower than expected!");
    }else{
      while( (((int)millis()) / 1000.0) - start_time <= dt){}
    }
  }
  return 0;
}



extern "C" void EXTI2_IRQHandler(void){
  if(EXTI_GetITStatus(EXTI_Line2) != RESET){
    shuttingDown = true;
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}



void assert_failed(uint8_t* file, uint32_t line){
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  char str[150];

  sprintf(str, "Wrong parameters value: file %s on line %u\r\n", file, (unsigned int)line);
  nh.logerror(str);
  while (1){
    debug_toggle();
    delay(100);
    nh.spinOnce();
  }
}
 