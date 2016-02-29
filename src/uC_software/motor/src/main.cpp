#include <stm32f10x.h>

#include <stdio.h>

#include "servo.h"
#include "motors.h"
#include "led.h"
#include "pid.h"
#include "encoder.h"
#include "stm32_time.h"


#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>


#define RXBUFFERSIZE   0x02

/* Private function prototypes -----------------------------------------------*/



__IO uint8_t Rx_Idx = 0x00;
uint8_t slaveAddress;
uint8_t RxBuffer [RXBUFFERSIZE] = {0};
__IO uint8_t NumberOfByteToReceive = 0x00;


ros::NodeHandle nh;

void led_cb( const std_msgs::Empty& toggle_msg){
  uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5);
  if(led_bit == (uint8_t)Bit_SET)
      GPIO_ResetBits(GPIOB, GPIO_Pin_5);
  else
      GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo_set(0, cmd_msg.data);
  servo_set(1, cmd_msg.data);
}

void motor_cb( const std_msgs::Int16& cmd_msg){
  motor_set_speed(0, abs(cmd_msg.data), (cmd_msg.data > 0));
  motor_set_speed(1, abs(cmd_msg.data), (cmd_msg.data > 0));
  motor_set_speed(2, abs(cmd_msg.data), (cmd_msg.data > 0));
  motor_set_speed(3, abs(cmd_msg.data), (cmd_msg.data > 0));
}


ros::Subscriber<std_msgs::Int16> motor_sub("motor", &motor_cb);
ros::Subscriber<std_msgs::UInt16> servo_sub("servo", &servo_cb);
ros::Subscriber<std_msgs::Empty> led_sub("led", &led_cb);

std_msgs::String str_msg;
ros::Publisher chatter("encoder", &str_msg);

char hello[25];


int main(void){
  /* System Clocks Configuration */
  RCC_Configuration();

  I2C1_Init();
  SysTick_Init();
  DEBUG_Init();
  SERVO_Init();
  MOTOR_Init();
  LED_Init();

 

  //servo_set(0,90);
  //servo_set(1,90);

  led_set(0);
  /*int16_t speed = 0;

  nh.initNode();
  nh.subscribe(motor_sub);
  nh.subscribe(servo_sub);
  nh.subscribe(led_sub);
  nh.advertise(chatter);*/
  int cnt = 7000;
  int dir = 1;

  motor_set_speed(0, cnt, dir);
  motor_set_speed(1, cnt, dir);
  motor_set_speed(2, cnt, dir);
  motor_set_speed(3, cnt, dir);


  while (1){
    /*debug_toggle();
    led_set(cnt);
    if(dir)
    	cnt+=10;
    else
    	cnt-=10;
    if(cnt>255){
    	dir = 0;
    	cnt = 255;
    }else if(cnt<40){
    	dir = 1;
    	cnt = 40;
    }*/


    /*NumberOfByteToReceive = RXBUFFERSIZE;
    Rx_Idx = 0x00;

    slaveAddress = SLAVE_ADDRESS1;
    
    I2C_ITConfig(I2C1, I2C_IT_EVT , ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);


    // TODO: Add timeout here
    while ((Rx_Idx < RXBUFFERSIZE)); 

    speed = ((int16_t)((RxBuffer[0] << 8) | RxBuffer[1]));

    if(speed > 8000)
      speed = speed - 16384;


    sprintf(hello, "Speed is: %d",speed);
    str_msg.data = hello;
    chatter.publish( &str_msg );*/

    nh.spinOnce();


    delay(100);
  }
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
 