#include <Wire.h> //wire library to communicate
#include <TeensyThreads.h> //threads 
#include <PID_v1.h> //PID 
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //gps 
#include <Adafruit_MPL3115A2.h> //barometer 
#include <MPU9250.h> //Gyro, accel

SFE_UBLOX_GNSS myGNSS; //defy a gps instance 
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //defy a baro instance
MPU9250 mpu; //gyro, accel, compass

//GPS location
int latitude;
int longtitude;
float groundSpeed;

//Gyroscope
float roll;
float pitch;
float yaw;

//PID coefficients
float cP;
float cI;
float cD;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
