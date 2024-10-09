#define __STM32F1__

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
Adafruit_BNO055 bno = Adafruit_BNO055();
std_msgs::Float32 imu_msg;

ros::Publisher imu_pub("imu", &imu_msg);

void setup() {
  Wire.begin();
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(imu_pub);
}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);
  
  float yaw = event.orientation.z;

  if (yaw < 0) {
    yaw += 360;
  } else if (yaw >= 360) {
    yaw -= 360;
  }

  imu_msg.data = yaw;
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  delay(100);
}
