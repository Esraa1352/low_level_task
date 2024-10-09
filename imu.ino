#define __STM32F1__

#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
MPU6050 mpu;
std_msgs::Float32 imu_msg;

ros::Publisher imu_pub("imu", &imu_msg);

float yaw = 0.0;  // Initialize yaw
unsigned long last_time = 0;  // Variable to track time

int16_t ax, ay, az, gx, gy, gz, gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

void setup() {
  Wire.begin();
  Serial.begin(57600);

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  nh.initNode();
  nh.advertise(imu_pub);
 
  calibrateGyro();
;
}

void loop() {
  

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gx -= gyroXOffset;
  gy -= gyroYOffset;
  gz -= gyroZOffset;
  //conveting from LSB/S to degrees/sec
  float gz_dps = gz / 131.0;
  //calculating the yaw in degrees
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;
  yaw += gz_dps * dt;

  if (yaw < 0) {
    yaw += 360;
  } else if (yaw >= 360) {
    yaw -= 360;
  }

  // imu_msg.data = ax; 
  // imu_msg.data = ay; 
  // imu_msg.data = az; 
  // imu_msg.data = gx; 
  // imu_msg.data = gy; 
  imu_msg.data = yaw; 
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  delay(100); 
}

void calibrateGyro() {
  int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  const int numReadings =1000;
  for (int i = 0; i < numReadings; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    gyroXSum += gx;
    gyroYSum += gy;
    gyroZSum += gz;
    delay(10); 
  }

  // Calculate the average offsets
  gyroXOffset = gyroXSum / numReadings;
  gyroYOffset = gyroYSum / numReadings;
  gyroZOffset = gyroZSum / numReadings;
}