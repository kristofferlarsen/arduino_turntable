/*
 * Arduino + ROS stepper driver node
 * Used to control a turntable (0-360 deg) 
 * Input is position in step (0-12800)
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <rosserial_arduino/Test.h>
#define stp 3
#define dir 2
#define MS1 5
#define MS2 4
#define EN 6

ros::NodeHandle  nh;
using rosserial_arduino::Test;

int i = 0;
long current_position = 0;
long target_position = 0;
int vel;

std_msgs::Int32 current_angle;
ros::Publisher chatter("current_angle", &current_angle);

/*
 * Callback method for topic "set_turntable_angle"
 */
void messageCb(const std_msgs::Int32& control_msg){
  motor_enable(true);
  vel = 300;
  target_position = control_msg.data; 
}

ros::Subscriber<std_msgs::Int32> sub("set_turntable_angle",messageCb);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);
  
  //disable motors by default
  digitalWrite(EN,HIGH);
  //setup for microstepping (L,L is full step)
  digitalWrite(MS1,HIGH);
  digitalWrite(MS2,HIGH);
}

/* 
 * Main loop
 */
void loop()
{
  if(current_position > target_position)
  {
    if(vel > 40 && current_position - target_position > 260)
    {
      vel = vel-1;
    }
    if(current_position - target_position < 260 && vel < 300)
    {
      vel = vel + 1;
    }
    sted_Backwards();
  }
  if(current_position < target_position)
  {
    if(vel > 40 && target_position - current_position > 260)
    {
      vel = vel-1;
    }
    if(target_position - current_position < 260 && vel < 300)
    {
      vel = vel + 1;
    }
    step_Forwards();
  }
  
  i = i+1;
  if(i>100)
  {
    current_angle.data = current_position;
    chatter.publish(&current_angle);
    i = 0;
    if(current_position == target_position)
    {
      motor_enable(false);
    }
  }
  nh.spinOnce();
}

/*
 * Actuate one step in forward direction
 */
void step_Forwards(){
  //move one step forward
  digitalWrite(dir,LOW);
  digitalWrite(stp,HIGH);
  delayMicroseconds(vel);
  digitalWrite(stp,LOW);
  delayMicroseconds(vel);
  current_position = current_position + 1.0;
}

/*
 * Actuate one step in forward direction
 */
void sted_Backwards(){
  //move one step backwards
  digitalWrite(dir,HIGH);
  digitalWrite(stp,HIGH);
  delayMicroseconds(vel);
  digitalWrite(stp,LOW);
  delayMicroseconds(vel);
  current_position = current_position - 1.0;
}

/*
 * Enables and disables the stepper motor
 */
void motor_enable(bool state)
{
  if(state == true)
  {
    //enable motor
    digitalWrite(EN,LOW);
  }
  else
  {
    //disable motor
    digitalWrite(EN,HIGH);
  }
}
