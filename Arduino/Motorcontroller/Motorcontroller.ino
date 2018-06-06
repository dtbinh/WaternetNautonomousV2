#include <SoftwareSerial.h>
#include <Sabertooth.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

unsigned long int a,b,c;
int x[15],ch1[15],ch[7],i;
//specifing arrays and variables to store values 

int motorControlM1Jetson = 0;
int motorControlM2Jetson = 0;
int motorControlM1RC = 0;
int motorControlM2RC = 0;

float Ramptime = 1; // Ramptime [s]
int RampCommand;
int Deadband = 30;

SoftwareSerial SWSerial(NOT_A_PIN, 3); // RX on no pin (unused), TX on pin 11 (to S1).
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
    
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING);
  
  nh.initNode();
  nh.subscribe(sub);
  
}

void loop() 
{ 
  read_rc();

  if (ch[5] > 750)
  {
    ST.stop();
  }
  else if (ch[4] > 500)
  {
    ST.motor(1,motorControlM1Jetson);
    ST.motor(2,-motorControlM2Jetson);
  }
  else
  {
    motorControlM1RC = (int)(max(min((((double)((ch[1] - 500) + (ch[3] - 500)) / 500.0)),1),-1) * 127);
    motorControlM2RC = (int)(max(min((((double)((ch[1] - 500) - (ch[3] - 500)) / 500.0)),1),-1) * 127);
    ST.motor(1,motorControlM1RC);
    ST.motor(2,-motorControlM2RC);
  }

 // print();
  nh.spinOnce();
}

void read_me()  
{
 //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
 //this code gives channel values from 0-1000 values 
  a=micros(); //store time value a when pin value falling
  c=a-b;      //calculating time inbetween two peaks
  b=a;        // 
  x[i]=c;     //storing 15 value in array
  i=i+1;
  if(i==15)
  {
    for(int j=0;j<15;j++) 
    {
      ch1[j]=x[j];
    }
    i=0;
  }
}//copy store all values from temporary array another array after 15 reading  
             
void read_rc()
{
  int i,j,k=0;
  for(k=14;k>-1;k--)
  {
    if(ch1[k]>10000)
    {
      j=k;
    }
  }  //detecting separation space 10000us in that another array                     
  for(i=1;i<=6;i++)
  {
    ch[i]=(ch1[i+j]-1000);
  }
}     //assign 6 channel values after separation space

void print()
{   
  Serial.print(ch[1]);Serial.print("\t");
  Serial.print(ch[2]);Serial.print("\t");
  Serial.print(ch[3]);Serial.print("\t");
  Serial.print(ch[4]);Serial.print("\t");
  Serial.print(ch[5]);Serial.print("\t");
  Serial.print(ch[6]);Serial.print("\t");
  Serial.print(motorControlM1RC);Serial.print("\t");
  Serial.print(motorControlM2RC);Serial.print("\n");
}

