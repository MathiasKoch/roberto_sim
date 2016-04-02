#include <stm32f10x.h>

#include <stdio.h>

#include "motor.h"
#include "motorSettings.h"
#include "led.h"
#include "encoder.h"
#include "stm32_time.h"


#include <ros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <roberto_msgs/MotorState.h>



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

float wheelSeperation;

ros::NodeHandle nh;

void led_cb( const std_msgs::UInt8& cmd_msg){
  led_set(cmd_msg.data);
}



void motor_cb( const roberto_msgs::MotorState& cmd_msg){
  if(cmd_msg.mode == cmd_msg.DRIVE_MODE_AUTO){

  }else if(cmd_msg.mode == cmd_msg.DRIVE_MODE_PIVOT){
    front_right->setReference(cmd_msg.speed);
    front_left->setReference(cmd_msg.speed);
    rear_right->setReference(cmd_msg.speed);
    rear_left->setReference(cmd_msg.speed);
    float h = (cmd_msg.heading_angle+1.0)*90;             //[0-2]*90 = [0-180]
    servo_left->setReference(h);
    servo_right->setReference(h);

  }else if(cmd_msg.mode == cmd_msg.DRIVE_MODE_SPIN){

  }else if(cmd_msg.mode == cmd_msg.DRIVE_MODE_SIDEWAYS){

  }
}


ros::Subscriber<roberto_msgs::MotorState> motor_sub("throttled_joy_vel", &motor_cb);
//ros::Subscriber<sensor_msgs::Joy> motor_sub("joy", &motor_cb);
ros::Subscriber<std_msgs::UInt8> led_sub("led", &led_cb);

std_msgs::Float32MultiArray str_msg;
ros::Publisher chatter("encoder", &str_msg);

int main(){
  /* System Clocks Configuration */
  RCC_Configuration();

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  I2C1_Init();
  SysTick_Init();
  DEBUG_Init();
  LED_Init();

  str_msg.data = (float *)malloc(sizeof(float)*4);
  str_msg.data_length = 4;

  nh.initNode();

  /*while(!nh.connected()){
    nh.spinOnce();
  }*/

  nh.subscribe(motor_sub);
  nh.subscribe(led_sub);
  nh.advertise(chatter);
  

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

  if(!nh.getParam("serial_node/wheelSeperation", &wheelSeperation, 1)){
    wheelSeperation = 0.3;
  }

  int enc_timeout;
  if(!nh.getParam("serial_node/encoderTimeout", &enc_timeout, 1)){
    enc_timeout = 10;
  }

  delay(1000);

  motorSettings SL(MOTOR_TYPE_SERVO, "servo_left", TIM4, 4);
  SL.m_ServoPin = GPIO_Pin_9;
  SL.m_ServoPort = GPIOB;
  SL.encoder_timeout = enc_timeout;

  motorSettings SR(MOTOR_TYPE_SERVO, "servo_right", TIM4, 3);
  SR.m_ServoPin = GPIO_Pin_8;
  SR.m_ServoPort = GPIOB;
  SR.encoder_timeout = enc_timeout;

  motorSettings FR(MOTOR_TYPE_DC_MOTOR, "front_right", TIM1, 1);
  FR.setDCPins(GPIO_Pin_13, GPIOC, GPIO_Pin_14, GPIOC,
              GPIO_Pin_15, GPIOC, GPIO_Pin_0, GPIOA,
              GPIO_Pin_8, GPIOA);
  FR.encoderAddr = (0x10 | 0x08);
  FR.setRegulator(KP,KI,KD,integralSaturation);
  FR.wheelRadius = wheelRadius;
  FR.encoder_timeout = enc_timeout;

  motorSettings FL(MOTOR_TYPE_DC_MOTOR, "front_left", TIM3, 4);
  FL.setDCPins(GPIO_Pin_2,GPIOB, GPIO_Pin_5,GPIOA,
              GPIO_Pin_4, GPIOA,GPIO_Pin_10,GPIOB,
              GPIO_Pin_1, GPIOB);
  FL.encoderAddr = (0x10);
  FL.setRegulator(KP,KI,KD,integralSaturation);
  FL.wheelRadius = wheelRadius;
  FL.encoder_timeout = enc_timeout;

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

  while (1){
    start_time = ((int)millis()) / 1000.0;
    connected = nh.connected();
    if(cnt++%5==0)
      debug_toggle();

        
    servo_left->update(0.1, connected);
    servo_right->update(0.1, connected);
    str_msg.data[0] = front_right->update(0.1, connected);
    str_msg.data[1] = front_left->update(0.1, connected);
    str_msg.data[2] = rear_left->update(0.1, connected);
    str_msg.data[3] = rear_right->update(0.1, connected);

    //servo_left->setReference(180);
    //servo_right->setReference(180);

    chatter.publish( &str_msg );

    nh.spinOnce();

    //delay(20);
    if((((int)millis()) / 1000.0) - start_time > dt){
      nh.logerror("Main loop running slower than expected!");
    }else{
      while( (((int)millis()) / 1000.0) - start_time < dt){}
    }
  }
  return 0;
}

void assert_failed(uint8_t* file, uint32_t line){
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  char str[150];

  sprintf(str, "Wrong parameters value: file %s on line %d\r\n", file, line);
  nh.logerror(str);
  while (1){
    debug_toggle();
    delay(100);
    nh.spinOnce();
  }
}
 