
//pid function modified from http://andrewjkramer.net/pid-motor-control/

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define RIGHT_FORWARD_PIN 9
#define RIGHT_REVERSE_PIN 10
#define LEFT_FORWARD_PIN 6
#define LEFT_REVERSE_PIN 5

#define rightEnc 3
#define leftEnc 2

#define ticksPerRev 32.0
#define maxMotorRPM 3900

//time before we decide our control program has failed and we kill the motors
#define timeoutMs 500

//set to 1 to enable a subscriber for P, I and D values.
//note that D values are scaled by dividing by 1000, since rosserial can only do 32 bit floats
#define pidTuningMode 0

int rticks = 0;
int lticks = 0;
int rticksPID = 0;
int lticksPID = 0;
unsigned int l_rpm = 0;
unsigned int r_rpm = 0;

int rSetPoint = 0;
int lSetPoint = 0;
short int rReverse = 0;
short int lReverse = 0;

double rITerm = 0.0;
double lITerm = 0.0;
int rLastSpeed = 0;
int lLastSpeed = 0;
short int rPwm = 0;
short int lPwm = 0;

double _kP = .045;
double _kI = 0.006;
double _kD = 0.000000000625;

long nextUpdateR = 0;
long nextUpdateL = 0;
long nextUpdatePID = 0;
long nextUpdatePower = 0;

void motor_cb_right( const std_msgs::Int16& msg )
{
int speed = 0;
nextUpdateR = millis() + timeoutMs; 

speed = msg.data;
  
if (msg.data > maxMotorRPM)
  speed = maxMotorRPM;

if (msg.data < (maxMotorRPM * -1))
  speed = (maxMotorRPM * -1);

rSetPoint = abs(speed);

if (speed > 0)
  rReverse = 0;
else
  rReverse = 1;
 
return;
}

void motor_cb_left( const std_msgs::Int16& msg )
{
int speed = 0;

speed = msg.data;
nextUpdateL = millis() + timeoutMs; 
  
if (msg.data > maxMotorRPM)
  speed = maxMotorRPM;

if (msg.data < (maxMotorRPM * -1))
  speed = (maxMotorRPM * -1);

lSetPoint = abs(speed);

if (speed > 0)
  lReverse = 0;
else
  lReverse = 1;
 
return;

}

void motorPID_P_cb( const std_msgs::Float32& msg )
{
 _kP = msg.data;
 
}

void motorPID_I_cb( const std_msgs::Float32& msg )
{
 _kI = msg.data;
 rITerm = 0.0;
 lITerm = 0.0;
}

void motorPID_D_cb( const std_msgs::Float32& msg )
{
  _kD = msg.data / 10000.0;
}

//callbacks need to be defined before these statements

std_msgs::Int16 tick_msg;
std_msgs::Float32 battVoltage;
std_msgs::Float32 current;

ros::Subscriber<std_msgs::Int16> sub1("/rightMotor", motor_cb_right );
ros::Subscriber<std_msgs::Int16> sub2("/leftMotor", motor_cb_left );


ros::Subscriber<std_msgs::Float32> subPID_P("/motorPID_P", motorPID_P_cb );
ros::Subscriber<std_msgs::Float32> subPID_I("/motorPID_I", motorPID_I_cb );
ros::Subscriber<std_msgs::Float32> subPID_D("/motorPID_D", motorPID_D_cb );
    
ros::Publisher pub_rightMotorTicks("rightMotorTicks", &tick_msg);
ros::Publisher pub_leftMotorTicks("leftMotorTicks", &tick_msg);
  
ros::Publisher pub_bVoltage("battVoltage", &battVoltage);
ros::Publisher pub_currentDraw("currentDraw", &current);

ros::NodeHandle nh;


void setup()
{

nh.initNode();
nh.subscribe(sub1);
nh.subscribe(sub2);

if (pidTuningMode == 1)
  {
  nh.subscribe(subPID_P);
  nh.subscribe(subPID_I);
  nh.subscribe(subPID_D);
  }
  
nh.initNode();
nh.advertise(pub_rightMotorTicks);
nh.advertise(pub_leftMotorTicks);
nh.advertise(pub_bVoltage);
nh.advertise(pub_currentDraw);
nh.loginfo("Starting Teensy Motor Controller..");   

//disable unless debugging
//Serial.begin(57600);

pinMode(RIGHT_FORWARD_PIN, OUTPUT); 
pinMode(RIGHT_REVERSE_PIN, OUTPUT);
pinMode(LEFT_FORWARD_PIN, OUTPUT);
pinMode(LEFT_REVERSE_PIN, OUTPUT);

pinMode(rightEnc,INPUT ); 
pinMode(leftEnc,INPUT );

attachInterrupt(digitalPinToInterrupt(rightEnc), rpulse, CHANGE);
attachInterrupt(digitalPinToInterrupt(leftEnc), lpulse, CHANGE);
}

void loop()
{
  
  nh.spinOnce();

  if (millis() >= nextUpdateR)
     rSetPoint = 0;

  if (millis() >= nextUpdateL)
     lSetPoint = 0;   

if (millis() >= nextUpdatePID)
    {       
    nextUpdatePID = millis() + 50;

    tick_msg.data = rticks;
    //if (rticks != 0)
    pub_rightMotorTicks.publish(&tick_msg);
    tick_msg.data = lticks;
    //if (lticks != 0)
    pub_leftMotorTicks.publish(&tick_msg);

    //rpm = tickCount / ticksPerRev * 20 samples per sec * 60 sec per min
    r_rpm = (rticksPID / ticksPerRev) * 20.0 * 60.0;
    l_rpm = (lticksPID / ticksPerRev) * 20.0 * 60.0;

    //Serial.println(l_rpm);
    
    adjustPWM();

    rticks = 0;
    lticks = 0;
    rticksPID = 0;
    lticksPID = 0;
    }

 if (millis() >= nextUpdatePower)
    {       
    nextUpdatePower = millis() + 1000;
    int currentSensorValue = analogRead(A0);
    float currentSensorVoltage= currentSensorValue * (3.3 / 1023.0);
    //scale for voltage divider on ADC input
    currentSensorVoltage = currentSensorVoltage * 1.563966218;
    //current is 100 mv per amp, zero is 2.5V
    float ampsDrawn = (currentSensorVoltage - 2.50) * 10.0;
    int battVoltageValue = analogRead(A1);
    float voltage= battVoltageValue * (3.3 / 1023.0);
    //scale for voltage divider on ADC input
    voltage = 4.929143561 * voltage;
    
    battVoltage.data = voltage;
    pub_bVoltage.publish( &battVoltage );

    current.data = ampsDrawn;
    pub_currentDraw.publish( &current );
    }  

}

void rpulse()
{
rticksPID = rticksPID + 1;
  
if (rReverse == 0)
  rticks = rticks + 1;
else
  rticks = rticks - 1;
}

void lpulse()
{
lticksPID = lticksPID + 1;
  
if (lReverse == 0)
  lticks = lticks + 1;
else
  lticks = lticks - 1;
}

void adjustPWM()
{
  
  int error = rSetPoint - r_rpm;  // calculate error
  rITerm += (_kI * (double)error); // calculate integral term
  double dInput = r_rpm - rLastSpeed; // calculate derivative
  int adjustment = (_kP * (double)error) + rITerm - (_kD * dInput);
  rPwm += adjustment;
  //limit speed to range of pwm 0-255
  rPwm = limitPWM(rPwm);

  if (rSetPoint == 0)
      rPwm = 0;
 
  if (rReverse == 0)
     {
      analogWrite(RIGHT_FORWARD_PIN, rPwm);
      analogWrite(RIGHT_REVERSE_PIN, 0);
     }
  else
     {
     analogWrite(RIGHT_REVERSE_PIN, rPwm);
     analogWrite(RIGHT_FORWARD_PIN, 0);
     }
     
  rLastSpeed = r_rpm;

  error = lSetPoint - l_rpm;  // calculate error
  lITerm += (_kI * (double)error); // calculate integral term
  dInput = l_rpm - lLastSpeed; // calculate derivative
  adjustment = (_kP * (double)error) + lITerm - (_kD * dInput);
  lPwm += adjustment;
  //limit speed to range of pwm 0-255
  lPwm = limitPWM(lPwm);

  if (lSetPoint == 0)
      lPwm = 0;
 
  if (lReverse == 0)
     {
      analogWrite(LEFT_FORWARD_PIN, lPwm);
      analogWrite(LEFT_REVERSE_PIN, 0);
     }
  else
     {
     analogWrite(LEFT_REVERSE_PIN, lPwm);
     analogWrite(LEFT_FORWARD_PIN, 0);
     }
     
  lLastSpeed = l_rpm;
}

int limitPWM(int pwm)
{
  if (pwm < 0)
     pwm = 0;

  if (pwm > 255)
     pwm = 255;

  return pwm;   
}
