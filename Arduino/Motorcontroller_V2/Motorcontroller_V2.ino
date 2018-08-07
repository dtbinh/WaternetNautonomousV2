#include <SoftwareSerial.h>
#include <Sabertooth.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

int ch[7], chh[7], noSignalCheck, t;

int motorControlM1Jetson = 0;
int motorControlM2Jetson = 0;
int motorControlM1RC = 0;
int motorControlM2RC = 0;

float Ramptime = 1; // Ramptime [s]
int RampCommand;
int Deadband = 30;

SoftwareSerial SWSerial(NOT_A_PIN, 4); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial);

ros::NodeHandle  nh;

void messageCb( const geometry_msgs::Twist& cmd_vel_msg){
  motorControlM1Jetson = (int)(max(min(cmd_vel_msg.linear.x + cmd_vel_msg.angular.z,1),-1) * 127);
  motorControlM2Jetson = (int)(max(min(cmd_vel_msg.linear.x - cmd_vel_msg.angular.z,1),-1) * 127);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup() 
{
  // Open serial communications and wait for port to open:
  SWSerial.begin(115200);
  ST.autobaud();

  ST.setDeadband(Deadband);
  if (Ramptime > 0.256)
  {
    RampCommand = round(((256/Ramptime)/15.25) + 10.0);
  }
  else
  {
    RampCommand = round(256/(1000*Ramptime));  
  }
  ST.setRamping(RampCommand);
    
  attachInterrupt(digitalPinToInterrupt(2), read_channel_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), read_channel_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), read_channel_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), read_channel_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), read_channel_5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), read_channel_6, CHANGE);
  
  nh.initNode();
  nh.subscribe(sub);

  pinMode(13,OUTPUT);
  
}

void loop() 
{ 
  if (ch[6] > 1750 || (noSignalCheck > 50))
  {
    ST.stop();
    digitalWrite(13,HIGH);
  }
  else if (ch[5] > 1500)
  {
    ST.motor(1,motorControlM1Jetson);
    ST.motor(2,-motorControlM2Jetson);
    digitalWrite(13,LOW);
  }
  else
  {
    motorControlM1RC = (int)(max(min((((double)((ch[1] - 1500) + (ch[4] - 1500)) / 500.0)),1),-1) * 127);
    motorControlM2RC = (int)(max(min((((double)((ch[1] - 1500) - (ch[4] - 1500)) / 500.0)),1),-1) * 127);
    ST.motor(1,motorControlM1RC);
    ST.motor(2,-motorControlM2RC);
    digitalWrite(13,LOW);
  }

  noSignalCheck++;
  print();
  nh.spinOnce();
}

void read_channel_1()  
{
  t = micros();
  if (digitalRead(2) == HIGH)
  {
    chh[1] = t;
  }
  else
  {
    ch[1] = t - chh[1];
  }
  noSignalCheck = 0;
}

void read_channel_2()  
{
  t = micros();
  if (digitalRead(3) == HIGH)
  {
    chh[2] = t;
  }
  else
  {
    ch[2] = t - chh[2];
  }
  noSignalCheck = 0;
}

void read_channel_3()  
{
  t = micros();
  if (digitalRead(18) == HIGH)
  {
    chh[3] = t;
  }
  else
  {
    ch[3] = t - chh[3];
  }
  noSignalCheck = 0;
}

void read_channel_4()  
{
  t = micros();
  if (digitalRead(19) == HIGH)
  {
    chh[4] = t;
  }
  else
  {
    ch[4] = t - chh[4];
  }
  noSignalCheck = 0;
}

void read_channel_5()  
{
  t = micros();
  if ( digitalRead(20) == HIGH)
  {
    chh[5] = t;
  }
  else
  {
    ch[5] = t - chh[5];
  }
  noSignalCheck = 0;
}

void read_channel_6()  
{
  t = micros();
  if (digitalRead(21) == HIGH)
  {
    chh[6] = t;
  }
  else
  {
    ch[6] = t - chh[6];
  }
  noSignalCheck = 0;
}

void print()
{   
  Serial.print(ch[1]);Serial.print("\t");
  Serial.print(ch[2]);Serial.print("\t");
  Serial.print(ch[3]);Serial.print("\t");
  Serial.print(ch[4]);Serial.print("\t");
  Serial.print(ch[5]);Serial.print("\t");
  Serial.print(ch[6]);Serial.print("\t");
  Serial.print(noSignalCheck);Serial.print("\t");
  Serial.print(motorControlM1RC);Serial.print("\t");
  Serial.print(motorControlM2RC);Serial.print("\n");
}

