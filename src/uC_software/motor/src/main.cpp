#include <stm32f10x.h>

#include <stdio.h>

#include "motor.h"
#include "motorSettings.h"
#include "led.h"
#include "pid.h"
#include "encoder.h"
#include "stm32_time.h"


#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <roberto_msgs/MotorState.h>



/* Private function prototypes -----------------------------------------------*/


__IO uint8_t Rx_Idx = 0x00;
uint8_t slaveAddress;
uint8_t RxBuffer [RXBUFFERSIZE] = {0};
__IO uint8_t NumberOfByteToReceive = 0x00;
__IO uint8_t GenerateStartStatus = 0x00;



ros::NodeHandle nh;

void led_cb( const std_msgs::UInt8& cmd_msg){
  led_set(cmd_msg.data);
}



void motor_cb( const roberto_msgs::MotorState& cmd_msg){

}


ros::Subscriber<roberto_msgs::MotorState> motor_sub("motor", &motor_cb);
ros::Subscriber<std_msgs::UInt8> led_sub("led", &led_cb);

std_msgs::Float32 str_msg;
ros::Publisher chatter("encoder", &str_msg);

char hello[25];
char str[150];


int main(){
  /* System Clocks Configuration */
  RCC_Configuration();

  nh.initNode();
  nh.subscribe(motor_sub);
  nh.subscribe(led_sub);
  nh.advertise(chatter);

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  I2C1_Init();
  SysTick_Init();
  DEBUG_Init();
  LED_Init();

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
  FR.setRegulator(5000,5000,2,100000);
  FR.wheelRadius = 0.04;

  motorSettings FL(MOTOR_TYPE_DC_MOTOR, "front_left", TIM3, 4);
  FL.setDCPins(GPIO_Pin_2,GPIOB, GPIO_Pin_5,GPIOA,
              GPIO_Pin_4, GPIOA,GPIO_Pin_10,GPIOB,
              GPIO_Pin_1, GPIOB);
  FL.encoderAddr = (0x10);

  motorSettings RL(MOTOR_TYPE_DC_MOTOR, "rear_left", TIM2, 4);
  RL.setDCPins(GPIO_Pin_11, GPIOB, GPIO_Pin_12, GPIOB,
    GPIO_Pin_13, GPIOB, GPIO_Pin_14, GPIOB,
    GPIO_Pin_3, GPIOA);

  motorSettings RR(MOTOR_TYPE_DC_MOTOR, "rear_right", TIM1, 4);
  RR.setDCPins(GPIO_Pin_15, GPIOB, GPIO_Pin_12,GPIOA,
              GPIO_Pin_15, GPIOA, GPIO_Pin_3, GPIOB,
              GPIO_Pin_11, GPIOA);

  motor *servo_left = motor::createMotor(&SL);
  if(!servo_left->motorInit()){
    sprintf(str, "Unable to initialize motor: %s - [FAIL]\r\n", servo_left->motorName());
    nh.logerror(str);
  }

  motor *servo_right = motor::createMotor(&SR);
  if(!servo_right->motorInit()){
    sprintf(str, "Unable to initialize motor: %s - [FAIL]\r\n", servo_right->motorName());
    nh.logerror(str);
  }

  motor *front_left = motor::createMotor(&FL);
  if(!front_left->motorInit()){
    sprintf(str, "Unable to initialize motor: %s - [FAIL]\r\n", front_left->motorName());
    nh.logerror(str);
  }

  motor *front_right = motor::createMotor(&FR);
  if(!front_right->motorInit()){
    sprintf(str, "Unable to initialize motor: %s - [FAIL]\r\n", front_right->motorName());
    nh.logerror(str);
  }

  motor *rear_left = motor::createMotor(&RL);
  if(!rear_left->motorInit()){
    sprintf(str, "Unable to initialize motor: %s - [FAIL]\r\n", rear_left->motorName());
    nh.logerror(str);
  }

  motor *rear_right = motor::createMotor(&RR);
  if(!rear_right->motorInit()){
    sprintf(str, "Unable to initialize motor: %s - [FAIL]\r\n", rear_right->motorName());
    nh.logerror(str);
  }

  servo_left->setReference(90);
  servo_right->setReference(100);

  //servo_left->update(1);
  //servo_right->update(1);
  //rear_right->update(1);
  //rear_left->update(1);
  //front_left->update(1);
  //front_right->update(1);

 
  /*int16_t speed = 0;*/

  
  //led_set(200);



  while (1){
    debug_toggle();

    front_right->setReference(2);
    float s_ = front_right->update(0.1);

    //printf(hello, "Speed is: %d", (int)s_);
    str_msg.data = s_;
    chatter.publish( &str_msg );

    nh.spinOnce();

    delay(10);
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
 