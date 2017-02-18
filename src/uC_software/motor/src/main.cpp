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
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <roberto_msgs/MotorState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>

#define CMDMSGTIMEOUT 500 // ms
//#define DEBUG

/* Private function prototypes -----------------------------------------------*/
#ifdef DEBUG
extern "C"{
  extern void initialise_monitor_handles(void);
}
#else
#define printf(...) 
#endif



motor *servo_left;
motor *servo_right;
motor *front_right;
motor *front_left;
motor *rear_right;
motor *rear_left;

float d, L;

float motorCmd[4];
float spinAngle = 0.0;
uint8_t currentMode = roberto_msgs::MotorState::DRIVE_MODE_PIVOT;
uint32_t lastMsg;
bool initialized = false;

ros::NodeHandle nh;

std_msgs::MultiArrayDimension odomDim;
std_msgs::MultiArrayDimension debugDim;

__IO bool shuttingDown = false;
__IO bool waitForServos = false;

bool spinningAutonomously = false;

void motor_cb( const roberto_msgs::MotorState& cmd_msg){
  if(!initialized)
    return;
  lastMsg = millis();
  uint8_t intMode = cmd_msg.mode;
  if(intMode == cmd_msg.DRIVE_MODE_AUTO){

  }

  if(currentMode != intMode){
    waitForServos = true;
  }

  if(intMode == cmd_msg.DRIVE_MODE_PIVOT){
    float speedMult[2] = {1,1};
    float angle[2] = {0, 0};
    if (cmd_msg.heading_angle != 0){
      float R = 0.05/sin(cmd_msg.heading_angle*M_PI/360);
      speedMult[0] = (2*R)/(2*R - (L/2 + d));
      speedMult[1] = (2*R)/(2*R + (L/2 + d));
      //angle[0] = atan(L/(B+R));   // B=L as robot is quadratic
      angle[0] = cmd_msg.heading_angle + atan(L/(L+R));
      angle[1] = cmd_msg.heading_angle + atan(L/(L-R));
    }else{
      angle[0] = 0;
      angle[1] = 0;
    }

    angle[0] = angle[0] > MAX_ANGLE_PIVOT? MAX_ANGLE_PIVOT : ( angle[0] < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : angle[0]);
    angle[1] = angle[1] > MAX_ANGLE_PIVOT? MAX_ANGLE_PIVOT : ( angle[1] < -MAX_ANGLE_PIVOT? -MAX_ANGLE_PIVOT : angle[1]);

    if(cmd_msg.heading_angle > 0){
      servo_left->setReference(angle[0]+135);
      servo_right->setReference(-angle[1]+135);
    }else{
      servo_left->setReference(angle[1]+135);
      servo_right->setReference(-angle[0]+135);
    }
    motorCmd[0] = cmd_msg.speed*speedMult[0];
    motorCmd[1] = cmd_msg.speed*speedMult[1];
    motorCmd[2] = cmd_msg.speed*speedMult[1];
    motorCmd[3] = cmd_msg.speed*speedMult[0];

  }else if(intMode == cmd_msg.DRIVE_MODE_SPIN){
    if(spinAngle == 0){
      if(cmd_msg.heading_angle != 0){
        spinningAutonomously = true;
        spinAngle = cmd_msg.heading_angle;
      }else{
        spinningAutonomously = false;
      }
      servo_left->setReference(90);
      servo_right->setReference(90);

      motorCmd[0] = cmd_msg.speed;
      motorCmd[1] = -cmd_msg.speed;
      motorCmd[2] = -cmd_msg.speed;
      motorCmd[3] = cmd_msg.speed;
    }

  }else if(intMode == cmd_msg.DRIVE_MODE_SIDEWAYS){
    servo_left->setReference(45);
    servo_right->setReference(45);

    motorCmd[0] = cmd_msg.speed;  //FR
    motorCmd[1] = -cmd_msg.speed; //FL
    motorCmd[2] = cmd_msg.speed; //RL
    motorCmd[3] = -cmd_msg.speed;  //RR
  }
  currentMode = intMode;
}


ros::Subscriber<roberto_msgs::MotorState> motor_sub("cmd_vel", &motor_cb);

std_msgs::Float32MultiArray debug_msg;
ros::Publisher debug_pub("debug", &debug_msg);

std_msgs::Float32MultiArray odom_msg;
ros::Publisher odom_pub("odom_vel", &odom_msg);

int main(){
  #ifdef DEBUG
  initialise_monitor_handles(); /* initialize handles */
  #endif
  /* System Clocks Configuration */
  RCC_Configuration();
  SysTick_Init();

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  I2C1_Init();
  DEBUG_Init();
  SHUTDOWN_Init();
  LED_Init();

  


  nh.initNode();


  nh.subscribe(motor_sub);
  nh.advertise(odom_pub);
  nh.advertise(debug_pub);

  /*while(!nh.connected()){
    nh.spinOnce();
  }*/

  shuttingDown = false;

  
  float FF;
  //if(!nh.getParam("serial_node/FF", &FF, 1)){
    FF = 1600;
  //}
  float KP;
  //if(!nh.getParam("serial_node/KP", &KP, 1)){
    KP = 6000;
  //}
  float KI;
  //if(!nh.getParam("serial_node/KI", &KI, 1)){
    KI = 4000;
  //}
  float KD;
  //if(!nh.getParam("serial_node/KD", &KD, 1)){
    KD = 40;
  //}


  float integralSaturation;
  //if(!nh.getParam("serial_node/integralSaturation", &integralSaturation, 1)){
    integralSaturation = 10;
  //}
  float wheelRadius;
  //if(!nh.getParam("serial_node/wheelRadius", &wheelRadius, 1)){
    wheelRadius = 0.04;
  //}

  //if(!nh.getParam("serial_node/widthBetweenMotorPivotPoints", &L, 1)){
    L = 0.17;
  //}

  //if(!nh.getParam("serial_node/motorPivotPointToWheel", &d, 1)){
    d = 0.045;
  //}

  delay(1000);


  motorSettings SL(MOTOR_TYPE_SERVO, "servo_left", TIM4, 4);
  SL.m_ServoPin = GPIO_Pin_9;
  SL.m_ServoPort = GPIOB;
  SL.m_ServoLimitMax = 1425;
  SL.m_ServoLimitMin = 7925;

  motorSettings SR(MOTOR_TYPE_SERVO, "servo_right", TIM4, 3);
  SR.m_ServoPin = GPIO_Pin_8;
  SR.m_ServoPort = GPIOB;
  SR.m_ServoLimitMax = 7925;
  SR.m_ServoLimitMin = 1825;

  motorSettings FR(MOTOR_TYPE_DC_MOTOR, "front_right", TIM1, 1);
  FR.setDCPins(GPIO_Pin_13, GPIOC, GPIO_Pin_14, GPIOC,
              GPIO_Pin_15, GPIOC, GPIO_Pin_0, GPIOA,
              GPIO_Pin_8, GPIOA);
  FR.encoderAddr = (0x10 | 0x08);
  FR.setRegulator(FF,KP,KI,KD,integralSaturation);
  FR.wheelRadius = wheelRadius;

  motorSettings FL(MOTOR_TYPE_DC_MOTOR, "front_left", TIM3, 4);
  FL.setDCPins(GPIO_Pin_2,GPIOB, GPIO_Pin_5,GPIOA,
              GPIO_Pin_4, GPIOA,GPIO_Pin_10,GPIOB,
              GPIO_Pin_1, GPIOB);
  FL.encoderAddr = (0x10);
  FL.setRegulator(FF,KP,KI,KD,integralSaturation);
  FL.wheelRadius = wheelRadius;

  motorSettings RL(MOTOR_TYPE_DC_MOTOR, "rear_left", TIM2, 4);
  RL.setDCPins(GPIO_Pin_11, GPIOB, GPIO_Pin_12, GPIOB,
    GPIO_Pin_13, GPIOB, GPIO_Pin_14, GPIOB,
    GPIO_Pin_3, GPIOA);
  RL.encoderAddr = (0x10 | 0x04);
  RL.setRegulator(FF,KP,KI,KD,integralSaturation);
  RL.wheelRadius = wheelRadius;

  motorSettings RR(MOTOR_TYPE_DC_MOTOR, "rear_right", TIM1, 4);
  RR.setDCPins(GPIO_Pin_15, GPIOB, GPIO_Pin_12,GPIOA,
              GPIO_Pin_15, GPIOA, GPIO_Pin_3, GPIOB,
              GPIO_Pin_11, GPIOA);
  RR.encoderAddr = (0x10 | 0x04 | 0x08);
  RR.setRegulator(FF,KP,KI,KD,integralSaturation);
  RR.wheelRadius = wheelRadius;

  //char error[60];
  servo_left = motor::createMotor(&SL);
  if(!servo_left->motorInit()){
    //sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", servo_left->motorName());
    //nh.logerror(error);
  }

  servo_right = motor::createMotor(&SR);
  if(!servo_right->motorInit()){
    //sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", servo_right->motorName());
    //nh.logerror(error);
  }

  front_left = motor::createMotor(&FL);
  if(!front_left->motorInit()){
    //sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", front_left->motorName());
    //nh.logerror(error);
  }

  front_right = motor::createMotor(&FR);
  if(!front_right->motorInit()){
    //sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", front_right->motorName());
    //nh.logerror(error);
  }

  rear_left = motor::createMotor(&RL);
  if(!rear_left->motorInit()){
    //sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", rear_left->motorName());
    //nh.logerror(error);
  }

  rear_right = motor::createMotor(&RR);
  if(!rear_right->motorInit()){
    //sprintf(error, "Unable to initialize motor: %s - [FAIL]\r\n", rear_right->motorName());
    //nh.logerror(error);
  }

  odom_msg.data = new float[3];
  odom_msg.layout.dim_length = 1;
  odom_msg.layout.dim = &odomDim;
  odom_msg.layout.dim[0].size = 3;
  odom_msg.layout.dim[0].stride = 3;
  odom_msg.data_length = 3;

  debug_msg.data = new float[6];
  debug_msg.layout.dim_length = 1;
  debug_msg.layout.dim = &debugDim;
  debug_msg.layout.dim[0].size = 6;
  debug_msg.layout.dim[0].stride = 6;
  debug_msg.data_length = 6;
  
  

  servo_left->setReference(135);
  servo_right->setReference(135);
  front_right->setReference(0);
  front_left->setReference(0);
  rear_right->setReference(0);
  rear_left->setReference(0);

  int cnt = 0;
  bool connected = false;

  uint32_t start_time;
  uint32_t dt = 20;
  float dt_s = ((int)dt*0.001f);

  //debug_toggle();


  // Initialize the servos one by one in order not to kill the ODroid
  while(cnt < 100){
    start_time = millis();
    if(cnt < 50)
      servo_left->update(dt_s, connected, true);
    else
      servo_right->update(dt_s, connected, true);
    while( millis() - start_time < dt);
    cnt++;
  }

  led_set(255);

  cnt = 0;


  float l = sqrt(pow((L/2),2)*2);

  //{fr, fl, rl, rr};
  float alpha[4] = {-M_PI/4, M_PI/4, 3*M_PI/4, -3*M_PI/4};
  uint32_t elapsed;
  initialized = true;


  while (1){
    start_time = millis();

    /*if(shuttingDown){
      connected = false;
      // TODO: Advertise shutdown??
    }else{*/
    if(cnt++%50 == 0)
      debug_toggle();
    connected = nh.connected();
    //}

    float sl = servo_left->update(dt_s, connected, true);
    float sr = servo_right->update(dt_s, connected, true);

    debug_msg.data[0] = (float)sl;
    debug_msg.data[1] = (float)sr;

    bool enable = true;

    if((waitForServos && ((int)sr == (int)servo_right->getReference() && (int)sl == (int)servo_left->getReference())) || lastMsg + CMDMSGTIMEOUT > start_time){
      waitForServos = false;
      front_right->setReference(motorCmd[0]);
      front_left->setReference(motorCmd[1]);
      rear_left->setReference(motorCmd[2]);
      rear_right->setReference(motorCmd[3]);
      delay(2);
    }else{
      enable = false;
      front_right->setReference(0);
      front_left->setReference(0);
      rear_left->setReference(0);
      rear_right->setReference(0);
    }
    
    float fr = front_right->update(dt_s, connected, enable);
    //udelay(500);
    float fl = front_left->update(dt_s, connected, enable);
    //udelay(500);
    float rl = rear_left->update(dt_s, connected, enable);
    //udelay(500);
    float rr = rear_right->update(dt_s, connected, enable);
    //udelay(1000);



    debug_msg.data[2] = fr;
    debug_msg.data[3] = fl;
    debug_msg.data[4] = rl;
    debug_msg.data[5] = rr;

    float speeds[4] = {fr, fl, rl, rr};
    float angles[4] = {sr-90, -sl+90, sl+90, -sr-90};

    int i;
    float x_dot = 0;
    float y_dot = 0;
    float theta_dot = 0;
    for(i = 0; i < 4; i++){
      angles[i] = angles[i]*M_PI/180;
      
      float wheelAngle = alpha[i]+angles[i];
      x_dot += cos(wheelAngle)*speeds[i];
      y_dot -= sin(wheelAngle)*speeds[i];
      
      float motorAngle = wheelAngle;
      if(i==2 || i==3){
        motorAngle += M_PI;
      }
      float deltaX = -(l*sin(alpha[i]) + d*sin(motorAngle));
      float deltaY = l*cos(alpha[i]) + d*cos(motorAngle);

      float deltaNorm = sqrt(deltaX*deltaX + deltaY*deltaY);
      float deltaXNorm = deltaX/deltaNorm;
      float deltaYNorm = deltaY/deltaNorm;

      float wAngle = wheelAngle+M_PI/2;
      float wX = cos(wAngle);
      float wY = sin(wAngle);

      theta_dot += (((deltaXNorm*wX + deltaYNorm*wY)*wX)/deltaNorm)*speeds[i];
    }
    x_dot /= 4;
    y_dot /= 4;
    theta_dot /= 4;

    // Magic constant
    /*theta_dot = theta_dot*1.7;

    if(currentMode == roberto_msgs::MotorState::DRIVE_MODE_SPIN && spinningAutonomously && !waitForServos){
      if(spinAngle > 0){
        lastMsg = start_time;
        spinAngle -= (abs(theta_dot)*180/M_PI) * dt_s;
      }else{
        spinAngle = 0;
        motorCmd[0] = 0.0;
        motorCmd[1] = 0.0;
        motorCmd[2] = 0.0;
        motorCmd[3] = 0.0;
      }
    }*/



    odom_msg.data[0] = x_dot;
    odom_msg.data[1] = y_dot;
    odom_msg.data[2] = theta_dot;
    //odom_msg.data[3] = spinAngle;

    //reader(odom_msg);
    //if(odom_msg.data[0] != NULL && odom_msg.data[1] != NULL && odom_msg.data[2] != NULL)
    odom_pub.publish(&odom_msg);
    debug_pub.publish(&debug_msg);

    nh.spinOnce();

    elapsed = millis() - start_time;
    if(elapsed > dt){
      //char elapsed_msg[18];
      //sprintf(elapsed_msg, "SLOW! %u", elapsed);
      //nh.logerror(elapsed_msg);
    }else{
      /*char elapsed_msg2[8];
      sprintf(elapsed_msg2, "%u", elapsed);
      nh.logerror(elapsed_msg2);*/
      while( (millis() - start_time) < dt){}
    }
  }
  //delete odom_msg.data;
  return 0;
}

extern "C" void EXTI2_IRQHandler(void){
  if(EXTI_GetITStatus(EXTI_Line2) != RESET){
    shuttingDown = true;
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}


extern "C" __attribute__((naked)) void HardFault_Handler(void){
        /*
         * Get the appropriate stack pointer, depending on our mode,
         * and use it as the parameter to the C handler. This function
         * will never return
         */


        __asm(  ".syntax unified\n"
                        "MOVS   R0, #4  \n"
                        "MOV    R1, LR  \n"
                        "TST    R0, R1  \n"
                        "BEQ    _MSP    \n"
                        "MRS    R0, PSP \n"
                        "B      hard_fault_handler_c      \n"
                "_MSP:  \n"
                        "MRS    R0, MSP \n"
                        "B      hard_fault_handler_c      \n"
                ".syntax divided\n") ;
}


extern "C" void hard_fault_handler_c (unsigned int * hardfault_args)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
 
  unsigned int stacked_r0 = ((unsigned long) hardfault_args[0]);
  unsigned int stacked_r1 = ((unsigned long) hardfault_args[1]);
  unsigned int stacked_r2 = ((unsigned long) hardfault_args[2]);
  unsigned int stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  unsigned int stacked_r12 = ((unsigned long) hardfault_args[4]);
  unsigned int stacked_lr = ((unsigned long) hardfault_args[5]);
  unsigned int stacked_pc = ((unsigned long) hardfault_args[6]);
  unsigned int stacked_psr = ((unsigned long) hardfault_args[7]);
 
  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
  printf ("PC [R15] = %x  program counter\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
  
  NVIC_SystemReset();
}


void assert_failed(uint8_t* file, uint32_t line){
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /*char str[150];

  sprintf(str, "Wrong parameters value: file %s on line %u\r\n", file, (unsigned int)line);
  nh.logerror(str);*/
  while (1){
    //debug_toggle();
    //delay(100);
    //nh.spinOnce();
  }
}
 