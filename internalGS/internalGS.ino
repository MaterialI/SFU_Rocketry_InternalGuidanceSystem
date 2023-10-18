#include <_Teensy.h>

#include <Wire.h> //wire library to communicate
#include <TeensyThreads.h> //threads 
#include <PID_v1.h> //PID 
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //gps 
#include <Adafruit_MPL3115A2.h> //barometer 
#include <MPU9250.h> //Gyro, accel
#include <arm_math.h> //complex math functions
#include <math.h>  //simple math functions

SFE_UBLOX_GNSS myGNSS; //defy a gps instance 
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //defy a baro instance
MPU9250 mpu; //gyro, accel, compass

// WireType* wire;



//threads 
int measurementThread;
int accelThr;
int gnssThr;
//mutex locks definition

Threads::Mutex I2CMutex;
Threads::Mutex GNSSrwMutex;
Threads::Mutex accelRWMutex;

//measurement decision variable (0 = gnss, 1 = accelerometer/gyro/mag, 2 = barometer)
volatile unsigned int measurementDesigion = 0;

//GPS location
int latitude = 0;
int longitude =0;
int groundSpeed;


// Modelling
int modelLatitude = 0;
int modelLongitude = 0;



///



//Magnetometer 
volatile float magX = 0.0;
volatile float magY = 0.0;
volatile float magZ = 0.0;

//Gyroscope
volatile float gyroX = 0.0;
volatile float gyroY = 0.0;
volatile float gyroZ = 0.0;

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



//PathConstruction--------------------------

// final locations
float finLat = 49.271437f;
float finLong = -123.260032f;

//initial location
float inLat = 0.000000;
float inLong = 0.000000;

//suggested point 
float sugLat = 0.000000;
float sugLong = 0.000000;

//coefficients to determine the eq of a line lat = m*long + b with tweaks in case if long initial = long final.
float ml = 0.0;
float b = 0.0;
bool invert = false;    // triggered true when it is impossible to describe a line using a parametric func, as it is vertical, thus the coordinate system is flipped.

///////////////////////////////////////////





///////////////////////////////////////////

void constructPath()
{
  GNSSrwMutex.lock();
  float currLat = (float)latitude / 10000000;
  float currLong = (float)longitude / 10000000;
  Serial.println(currLat, 7);
  Serial.println(currLong, 7);
  GNSSrwMutex.unlock();
  if(abs(currLong - finLong) > 0.000001)
  {
    ml = (currLat - finLat)/(currLong - finLong);
  }
  else{
    invert = true;
  }
  if(!invert)
  {
    b = currLat - ml*currLong;
  }
  else
  {
    ml = 0.0;
    b = currLong - ml*currLat;
  }
}

void modelSuggest() // generates a suggested point to follow through construction of normal
{

}




void accelThread()
{
  
    I2CMutex.lock();
    if(mpu.update())
    { 
      
      accX = mpu.getAccX();
      accY = mpu.getAccY();
      accZ = mpu.getAccZ();
      gyroX = mpu.getGyroX();
      gyroY = mpu.getGyroY();
      gyroZ = mpu.getGyroZ();
      magX = mpu.getMagX();
      magY = mpu.getMagY();
      magZ = mpu.getMagZ();
    }
    I2CMutex.unlock();
}

void baroThread()
{
  
    I2CMutex.lock(); // block other readings
    altitude = baro.getAltitude();  //
    temperature_baro = baro.getTemperature();
    delay(10); // to ensure the data came
    I2CMutex.unlock();
}

void gnssThread()
{

  while(1){
    GNSSrwMutex.lock();
    I2CMutex.lock();

    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
    // groundSpeed = myGNSS.getGroundSpeed();
    
    //groundSpeed= (float)(myGNSS.getGroundSpeed());
    I2CMutex.unlock();
    GNSSrwMutex.unlock();
    delay(1000);
  }
}
void callMeasurementsThread()
{
  while(1)
  {
    measurementDesigion = (measurementDesigion + 1)%2;
    delay(20);
    // if(measurementDesigion == 0)
    // {
    //   gnssThread();
    // }
    if(measurementDesigion == 0)
    {
      accelThread();
    }
    if(measurementDesigion == 1)
    {
      baroThread();
    }
    
  }
  
}

void setup()
{
  
  Serial.begin(9600); 
  Wire2.begin();
  Wire.begin(); 
  delay(2000);


  if(baro.begin() == true)    //initialize barometer
  {
    Serial.println("Barometer module detected!");
  }
  else
  {
    Serial.println("Unable to detect Barometer module!");
  }
 

  if(mpu.setup(0x68) == true)  //initialize IMU
  {
    Serial.println("Accelerometer module detected!");
    Serial.println("Calibrating compass, please wait...");
    // mpu.calibrateMag();
    Serial.println("Calibration done!");
    
  }
  else
  {
    Serial.println("Unable to detect Accelerometer module!");
  }
  if (myGNSS.begin(Wire2) == true) //initialize GPS
  {
    Serial.println("GNSS module detected!");
    
  }
  else
  {
    Serial.println("Unable to detect GNSS module!");
    //while (1);
  }
  
  measurementThread = threads.addThread(callMeasurementsThread);
  gnssThr = threads.addThread(gnssThread);
}



void loop()
{
      // if(latitude != 0)
      // {
      //   constructPath();
      // }
      Serial.print("Latitude: ");
      Serial.println(latitude);
      Serial.print("Longitude: ");
      Serial.println(longitude);
      Serial.print("Ground speed");
      Serial.println(groundSpeed);
      
      
      Serial.print("Roll (deg): ");
      Serial.println(accX);
      Serial.print("Pitch (deg): ");
      Serial.println(accY);
      Serial.print("Yaw (deg): ");
      Serial.println(accZ);
      

      // // threads.wait(accelThr, 1000);
      Serial.print("Baro Altitude (m): ");
      Serial.println(altitude);
      Serial.print("Baro Temperature (deg C): ");
      Serial.println(temperature_baro);
      
      // https://paulbourke.net/geometry/transformationprojection/#:~:text=Updated%20December%201999-,Stereographic,paper%20or%20a%20computer%20display.
      
 
      
      //Serial.println("---------------");
    delay(1000);    
  
}
//use different 