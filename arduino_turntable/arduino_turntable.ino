/*
 * Arduino + ROS stepper driver node
 * Used to control a turntable (0-360 deg)
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>
#define stp 3
#define dir 2
#define MS1 5
#define MS2 4
#define EN 6


ros::NodeHandle  nh;
using rosserial_arduino::Test;

int i = 0;
long one_rotation = 100000;
long current_position = 0;
long target_position = 0;


std_msgs::Float32 current_angle;
ros::Publisher chatter("current_angle", &current_angle);

void messageCb(const std_msgs::Float32& control_msg){
  
  
  //this constant needs to be changed
  
  target_position = 1000.0*control_msg.data;
  current_angle.data = target_position;
}
ros::Subscriber<std_msgs::Float32> sub("set_turntable_angle",messageCb);


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
  digitalWrite(MS1,LOW);
  digitalWrite(MS2,LOW);
}

void loop()
{
  
  if(current_position > target_position)
  {
    motor_enable(true);
    sted_Backwards();
  }
  if(current_position < target_position)
  {
    motor_enable(true);
    step_Forwards();
  }
  else
  {
    motor_enable(false);
  }
  
  i = i+1;
  if(i>200)
  {
    chatter.publish(&current_angle);
    i = 0;
  }
  nh.spinOnce();
}

void step_Forwards(){
  //move one step forward
  digitalWrite(dir,LOW);
  digitalWrite(stp,HIGH);
  delay(1);
  digitalWrite(stp,LOW);
  delay(1);
}

void sted_Backwards(){
  //move one step backwards
  digitalWrite(dir,HIGH);
  digitalWrite(stp,HIGH);
  delay(1);
  digitalWrite(stp,LOW);
  delay(1);
}

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
