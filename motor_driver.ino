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
#define IN2 PB4  
#define ENB PA1  // PWM pin for Motor B
#define IN3 PB12  
#define IN4 PB13 
#define ENC PB0  // PWM pin for Motor C
#define IN5 PB14  
#define IN6 PB15  
#define END PB1  // PWM pin for Motor D
#define IN7 PB8  
#define IN8 PB9  

void controlMotor(int EN, int INP1 , int INP2, float omega) 
{

  float dutyCycle = (omega / max_omega);

  int pwm = static_cast<int>((dutyCycle) * 255);

  if (omega > 0) 
  {
    digitalWrite(INP1, HIGH);
    digitalWrite(INP2, LOW); 
    analogWrite(EN , pwm);
  } 
  else if (omega < 0)
  {
    digitalWrite(INP1, LOW);
    digitalWrite(INP2, HIGH);
    analogWrite(EN , pwm);
  }
  else if (omega = 0)
  {
    digitalWrite(INP1, LOW);
    digitalWrite(INP2, LOW);
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
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

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

  controlMotor(ENA, IN1, IN2, omega_A);
  controlMotor(ENB, IN3, IN4, omega_B);
  controlMotor(ENC, IN5, IN6, omega_C);
  controlMotor(END, IN7, IN8, omega_D);
}
