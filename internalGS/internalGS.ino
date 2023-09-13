#include <_Teensy.h>

#include <Wire.h> //wire library to communicate
#include <TeensyThreads.h> //threads 
#include <PID_v1.h> //PID 
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //gps 
#include <Adafruit_MPL3115A2.h> //barometer 
#include <MPU9250.h> //Gyro, accel

SFE_UBLOX_GNSS myGNSS; //defy a gps instance 
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //defy a baro instance
MPU9250 mpu; //gyro, accel, compass


//threads 
int accelThr;
int baroThr;
int gnssThr;
//mutex locks definition

Threads::Mutex I2CMutex;



//GPS location
int latitude;
int longitude;
float groundSpeed;

//Gyroscope
volatile float roll = 0.0;
volatile float pitch = 0.0;
volatile float yaw = 0.0;

//Accelerometer
volatile float accX =0.0;
volatile float accY =0.0;
volatile float accZ = 0.0;

//Barometer
float altitude = 0.0; //m
float pressure = 0.0; //Pa
float temperature_baro = 0.0; //Centigrade

//PID coefficients
float cP;
float cI;
float cD;

void accelThread()
{
  while(1)
  {
    I2CMutex.lock();
    if(mpu.update())
    {
      
      yaw = mpu.getYaw();
      roll = mpu.getRoll();
      pitch = mpu.getPitch();

      accX = mpu.getAccX();
      accY = mpu.getAccY();
      accZ = mpu.getAccZ();

    }
    I2CMutex.unlock();
  }
}

void baroThread()
{
  while(1)
  {
    I2CMutex.lock();
    altitude = baro.getAltitude();
    pressure = baro.getPressure();
    temperature_baro = baro.getTemperature();
    I2CMutex.unlock();
  }
}

void gnssThread()
{
  while(1)
  {
    I2CMutex.lock();
    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
    //groundSpeed= (float)(myGNSS.getGroundSpeed());
    I2CMutex.unlock();
    //delay(100);
  }
}


void setup()
{
  
  Serial.begin(9600); // Initialize the Serial Monitor

  Wire.begin(); // Initialize I2C communication

  if(baro.begin() == true)  //initialize barometer
  {
    Serial.println("Barometer module detected!");
  }
  else
  {
    Serial.println("Unable to detect Barometer module!");
  }

  if(mpu.setup(0x68) == true)
  {
    Serial.println("Accelerometer module detected!");
    Serial.println("Calibrating compass, please wait...");
    mpu.calibrateMag();
    Serial.println("Calibration done!");
    
  }
  else
  {
    Serial.println("Unable to detect Accelerometer module!");
  }
  if (myGNSS.begin() == true) //initialize GPS
  {
    Serial.println("GNSS module detected!");
    // Configure GNSS module
   // myGNSS.setUART1Output(COM_TYPE_UBX);
    
  }
  else
  {
    Serial.println("Unable to detect GNSS module!");
    //while (1);
  }
  accelThr = threads.addThread(accelThread);
  baroThr = threads.addThread(baroThread);
  gnssThr = threads.addThread(gnssThread);
}



void loop()
{
   
      Serial.print("Heading (deg): ");
      Serial.println(yaw);
      // threads.wait(accelThr, 1000);
      Serial.print("Baro Altitude (m): ");
      Serial.println(altitude);
      // Print latitude, longitude, and altitude
      Serial.print("Latitude: ");
      Serial.println(latitude);
      Serial.print("Longitude: ");
      Serial.println(longitude);
      // Serial.print("Altitude (m): ");
      // Serial.println(myGNSS.getAltitude()/1000);
      
      
 
      
      //Serial.println("---------------");
    delay(1000);    
  
}
