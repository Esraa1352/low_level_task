#define __STM32F1__

#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::NodeHandle nh;

// float current_yaw = 0.0;
float max_omega = 30 ; //TO BE DETERMINED

float R = 0.014; //TO BE DETERMINED
float L = 0; //TO BE DETERMINED
float W = 0; //TO BE DETERMINED

float Vx = 0.0;
float Vy = 0.0;
float yaw = 0.0;

// STM32 pins
#define ENA PA0  // PWM pin for Motor A
#define IN1 PB3   
#define ENB PA1  // PWM pin for Motor B
#define IN2 PB12  
#define ENC PB0  // PWM pin for Motor C
#define IN3 PB14  
#define END PB1  // PWM pin for Motor D
#define IN4 PB8 

void controlMotor(int EN, int INP1 ,  float omega) 
{

  float dutyCycle = (omega / max_omega);

  int pwm = static_cast<int>((dutyCycle) * 255);

  if (omega > 0) 
  {
    digitalWrite(INP1, HIGH);
    analogWrite(EN , pwm);
  } 
  else if (omega < 0)
  {
    digitalWrite(INP1, LOW);
    analogWrite(EN , pwm);
  }
  else if (omega = 0)
  {
    digitalWrite(INP1, LOW);
    analogWrite(EN , 0);
  }
  
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  Vx = cmd_msg.linear.x ;
  Vy = cmd_msg.linear.y ;
  yaw = cmd_msg.angular.z;
}

// Subscribe to cmd/vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

void setup() {
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(IN4, OUTPUT);
  // Initialize ROS node
  nh.initNode();
  nh.subscribe(sub);

  Wire.begin(); 
  Serial.begin(57600);
}

void loop() {
  inv_Kin(Vx,Vy,yaw);
  nh.spinOnce();
  delay(100);
}


void inv_Kin(float Vx, float Vy, float yaw) {
  float k = (L + W) / R;

  float omega_A = (Vx - Vy - k * yaw) / R;
  float omega_B = (Vx  + Vy + k * yaw) / R;
  float omega_C = (-Vx  + Vy + k * yaw) / R;
  float omega_D = (-Vx - Vy- k * yaw) / R;

  controlMotor(ENA, IN1, omega_A);
  controlMotor(ENB, IN2, omega_B);
  controlMotor(ENC, IN3, omega_C);
  controlMotor(END, IN4, omega_D);
}
