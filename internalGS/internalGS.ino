#include <_Teensy.h>

#include <Wire.h>                                 //wire library to communicate
#include <TeensyThreads.h>                        //threads
#include <PID_v1.h>                               //PID
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //gps
#include <Adafruit_MPL3115A2.h>                   //barometer
#include <MPU9250.h>                              //Gyro, accel
#include <arm_math.h>                             //complex math functions
#include <math.h>                                 //simple math functions
#include <SPI.h>                                  //spi library
#include <SD.h>                                   //sd card
#include <FeedBackServo.h>

// to store the datapoints of the path we use an array of pairs which belong to orthdrome path on sphere (lon, lat)
// between the points we use approximation to loxodrome path (heading is the same throughout the whole path)

// state machine looks like 1 -> 2 (5 -> 4 -> 6 (5 and 6 could repeat)) -> 3 (5)

#define BOOTUP_STATE 0
#define ASCEND_STATE 1
#define DESCEND_STATE 2
#define LANDING_STATE 3

#define PATH_CONSTRUCTION_SUBSTATE 4
#define COURSE_CORRECTION_SUBSTATE 5
#define PATH_FOLLOWING_SUBSTATE 6

#define TEST true;

// communication pins/ i2c adresses.
#define FEEDBACK_PIN 7
#define SERVO_PIN 8
#define IMU_ADDRESS 0x68

SFE_UBLOX_GNSS myGNSS;                             // defy a gps instance
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();    // defy a baro instance
MPU9250 mpu;                                       // gyro, accel, compass
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN); /////Servo---------------

short state = BOOTUP_STATE;
short DESCEND_SUBSTATE = COURSE_CORRECTION_SUBSTATE;

// threads

int accelThr;
// mutex locks definition

Threads::Mutex I2CMutex;
Threads::Mutex GNSSrwMutex;
Threads::Mutex accelRWMutex;

// GPS location
float gpsHeading = 0.0;
float latitude = 0.0;
float longitude = 0.0;
float groundSpeed = 0.0; // m/s
long int GNSSdeltaT = 0;
float gnssAltitude = 0.0;

float nextPointHeading = 0.0; // heading to the next point (determined between current and next point)

// Modelling
int modelLatitude = 0;
int modelLongitude = 0;

// Magnetometer
volatile float magX = 0.0;
volatile float magY = 0.0;
volatile float magZ = 0.0;

volatile float magDecl = 15.75; // in degrees to the West in Vancouver, October 2023

// Gyroscope
volatile float gyroX = 0.0;
volatile float gyroY = 0.0;
volatile float gyroZ = 0.0;

// Accelerometer
volatile float accX = 0.0;
volatile float accY = 0.0;
volatile float accZ = 0.0;

// Euler
volatile float roll = 0.0;
volatile float pitch = 0.0;
volatile float heading = 0.0;

// Barometer
float baroAltitude = 0.0;     // m
float pressure = 0.0;         // Pa
float temperature_baro = 0.0; // Centigrade

// barometer vertical speed calculation
float prevAltitude = 0.0; // m
float deltaT_baro = 0;    // used to measure the time between samples to calculate vertical speed.
float vspeed = 0.0;       // m/s

float prevGnssAltitude = 0.0;
float deltaT_gnss = 0;
float gnssVspeed = 0.0;

// PathConstruction--------------------------

// final locations
float finLat = 49.1951;
float finLong = -123.1788;

// initial location
float inLat = 0.000000;
float inLong = 0.000000;

// suggested point
float sugLat = 0.000000;
float sugLong = 0.000000;

// coefficients to determine the eq of a line lat = m*long + b with tweaks in case if long initial = long final.
float m = 0.0;
float b = -190.0;

float m1 = 0.0;
float b1 = -190.0;

bool invert = false; // triggered true when it is impossible to describe a line using a parametric func, as it is vertical, thus the coordinate system is flipped.

// PID ----- ///////////////////////////////

// PID coefficients //to change
double cP = 0.5;
double cI = 0.5;
double cD = 0.2;

double fP = 0.5;
double fI = 0.5;
double fD = 0.2;

// pid setting (Can be used to set offsets to follow the path well, can be paired with track to make less adjustments to save the energy and make the flight better)
double allowedGoalRot = 0.0;
double allowedGoalFollowing = 0.0;

double pidInput = 0.0;
double steeringOutput = 0.0;

PID courseCorrection(&pidInput, &steeringOutput, &allowedGoalRot, cP, cI, cD, DIRECT); /// init course correction PID

double pidDeltaDistanceInput = 0.0;
PID pathCorrection(&pidDeltaDistanceInput, &steeringOutput, &allowedGoalFollowing, fP, fI, fD, DIRECT); /// init path following PID

///////////////////////////////////////////

// navigation

float distance2Points(float lat1, float lon1, float lat2, float lon2) // todo
{
  // lon1 = lon1 / 180 * PI;
  // lat1  = lat1 / 180 * PI;
  // lon2 = lon2 / 180 * PI;
  // lat2 = lat2 / 180 * PI;
  // float pLat = pow(sin((lat1 - lat2)/2),2);
  // float pLong = cos(lat1)*cos(lat2)*pow(sin((lon1 - lon2)/2),2);
  // float d = 2 * asin(sqrt(pLat + pLong));
  float d = acos(cos(lat1) * cos(lon1) * cos(lat2 - lon2) + sin(lat1) * sin(lon2));
  return d * 6371.0;
}

float setCourse2Points(float finalLat, float finalLong)
{
  // get distance between 2 points d=2*asin(sqrt((sin((lat1-lat2)/2))^2 +  cos(lat1)*cos(lat2)*(sin((lon1-lon2)/2))^2))
  latitude = latitude / 180 * PI;
  longitude = longitude / 180 * PI;
  finalLat = finalLat / 180 * PI;
  finalLong = finalLong / 180 * PI;
  float pLat = pow(sin((latitude - finalLat) / 2), 2);
  float pLong = cos(latitude) * cos(finalLat) * pow(sin((longitude - finalLong) / 2), 2);
  float d = 2 * asin(sqrt(pLat + pLong));

  // get heading
  // edgecase on poles
  float course = 0.0;
  if (cos(latitude) < 0.0000001)
  {
    if (latitude > 0)
    {
      course = PI;
    }
    else
      course = 2 * PI;
  }

  // set course
  if (sin(finalLong - longitude) < 0)
  {
    course = acos((sin(finalLat) - sin(latitude) * cos(d)) / (sin(d) * cos(latitude)));
  }
  else
    course = 2 * PI - acos((sin(finalLat) - sin(latitude) * cos(d)) / (sin(d) * cos(latitude)));
  return 360.0 - ((course / PI) * 180);
}

void constructLine2Points(float currLong, float currLat, float termLong, float termLat) // constructs the line model between 2 points which are approximate on small areas to loxodrome. TODO: test
{
  if (abs(currLong - termLong) > 0.000001)
  {
    invert = false;
    m = (termLat - currLat) / (termLong - currLong);
    b = termLat - (m * termLong);
  }
  else
  {
    invert = true;
    m = (termLong - currLong) / (termLat - currLat);
    b = currLong - (m * currLat);
  }
}

///////////////////////////////////////////

void constructPath() // generates a set of points which belong to orthodrome. These points will be used as checkpoints. TODO: write the generation and calculate the distances between each point and the final destination.
{
  // take the current position to use as reference point to generate a great circle algorithm.

  float currLat = latitude;
  float currLong = longitude;
  Serial.println(currLat, 7);
  Serial.println(currLong, 7);

  constructLine2Points(currLong, currLat, finLong, finLat);
  // orthodrome
  //  lat=atan((sin(currLat)*cos(finLat)*sin(lon-finLong) -sin(finLat)*cos(currLat)*sin(lon-currLong))/(cos(currLat)*cos(finLat)*sin(currLong-finLong)))
}

void pidDeltaAngle() // provides angle delta value in range (-180 to 180) to PID to correct to 0
{

  int delta = nextPointHeading * 100;

  int iYaw = heading * 100;

  int newYaw = (delta - iYaw) % 36000;
  if (newYaw < 18000)
  {
    newYaw *= -1;
  }
  else
    newYaw = 18000 - (newYaw) % 18000;
  pidInput = (float)(newYaw) / 100;
}

void modelSuggest() // generates a suggested point to follow through construction of normal to the approximation of the path. Will be fed into PID to follow the path. TODO: implement
// add the function inverse if the angular coefficient is too large (the line is almost verical in non-inverted system)
{

  if (b != -190.0)
  {
    float currLat = latitude;
    float currLong = longitude;
    if (!invert)
    {
      if (m != 0)
      {

        m1 = -1 / m;
        b1 = latitude - m1 * longitude;
        sugLong = (b - b1) / (m1 - m);
        sugLat = m * sugLong + b;
      }
      else
      {
        sugLong = currLong;
        sugLat = m * sugLong + b;
      }
    }
    else
    {
      if (m != 0)
      {
        m1 = -1 / m;
        b1 = longitude - m1 * latitude;
        sugLat = (b - b1) / (m1 - m);
        sugLong = m * sugLat + b;
      }
      else
      {
        sugLat = currLat;
        sugLong = m * sugLat + b;
      }
    }
  }
}

void pidDeltaPath() // calculates the XTE from the path
{
  // lat - (m*lon + b) > 0 on the
  float currLat = latitude;
  float currLong = longitude;
  float d = sqrt(pow(currLong - sugLong, 2) + pow(currLat - sugLat, 2));
  bool sign = true; // true =  >0 false = <0
  if (!invert)
  {
    sign = currLat - (m * currLong + b) >= 0;
    sign = (finLong - currLong) >= 0 ? sign : !sign;
  }
  else
  {
    sign = currLong - (m1 * currLat + b1) >= 0;
    sign = (finLat - currLat) >= 0 ? sign : !sign;
  }
  pidDeltaDistanceInput = sign ? d : d * -1;
}

////////////////////////////////-----------------measurements------------------------------///////////////////////

void accelThread() // the thread runs continiously without interruptions from other sensors to let the data fusion algorithm run
{
  // todo: add mutex locks whenever using the data
  while (true)
  {

    if (mpu.update())
    {
      roll = mpu.getRoll();
      pitch = mpu.getPitch();
      heading = mpu.getYaw() + 180.0; // to convert from (-180, 180) to (0, 360)
      accX = mpu.getAccX();
      accY = mpu.getAccY();
      accZ = mpu.getAccZ();
    }
  }
}

void baroThread() // uses the library to calculate the altitude. Added timestamps to calculate the verical speed. TODO: add exponential averaging
{

  baroAltitude = baro.getAltitude();

  vspeed = (baroAltitude - prevAltitude) / (((float)(millis()) - deltaT_baro) / 1000.0);
  deltaT_baro = (float)(millis());
  temperature_baro = baro.getTemperature();
  prevAltitude = baroAltitude;
}

void gnssThread() // uses library to get the coordinates in (long, lat) format. Calculates the ground speed. TODO: add veritcal speed calculation and fuse with barometer readings.
{
  latitude = myGNSS.getLatitude();
  longitude = myGNSS.getLongitude();
  gnssAltitude = (float)(myGNSS.getAltitude());
  gpsHeading = myGNSS.getHeading();
  groundSpeed = (float)(myGNSS.getGroundSpeed()) / 1000.0;

  gnssVspeed = (gnssAltitude - prevGnssAltitude) / (((float)(millis()) - deltaT_gnss) / 1000.0);

  deltaT_gnss = (float)(millis());

  latitude = (float)latitude / 10000000;
  longitude = (float)longitude / 10000000;
  prevGnssAltitude = gnssAltitude;
}

// state machine

// bool detectFall()
// {
//   float a[3] = {accX, accY, accZ};
//   float pAcc = cbrt(pow(a[0], 3) + pow(a[1], 3) +pow(a[2], 3));
//   if(vspeed<-1.0 && pAcc < 0.1 && gnssVspeed < -1.0)
//   {
//     state = DESCEND_STATE;
//   }
// }

// void stateAction() //todo finish the state machine, develop descend state machine.
// {
//   switch(state)
//   {
//     case BOOTUP_STATE:
//     {
//       Serial.println("The system has not finished its bootup sequence, restart the system and check whether sensors are connected.")
//     } break;
//     case ASCEND_STATE:
//     {
//       detectFall();
//     } break;
//   };
// }

void setup()
{
  // initialize the communication with the sensors.
  Serial.begin(9600);
  Wire2.begin();
  Wire.begin();
  delay(2000);

  // set the setting for IMU module
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_10HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ;

  // intiailize the communication with the sensors

  if (baro.begin() == true) // initialize barometer
  {
    Serial.println("Barometer module detected!");
  }
  else
  {
    Serial.println("Unable to detect Barometer module!");
  }

  // initialize IMU on 2nd I2C bus
  if (mpu.setup(0x68, setting, Wire2) == true)
  {
    Serial.println("Accelerometer module detected!");
    // conduct calibration of the compass and accelerometers and gyroscope
    Serial.println("Calibrating compass, please, perform rotations around 3 axis ");
    mpu.calibrateMag();
    Serial.println("Calibrating accelerometer and gyroscope");
    mpu.calibrateAccelGyro();
    Serial.println("Calibration done!");
  }
  else
  {
    Serial.println("Unable to detect Accelerometer module!");
  }

  if (myGNSS.begin() == true) // initialize GPS
  {
    Serial.println("GNSS module detected!");
    // myGNSS.setMeasurementRate(50);
  }
  else
  {
    Serial.println("Unable to detect GNSS module!");
    // if(!TEST) //if not test, stall the execution.
    //   while (1);
  }

  // Call acceleromter thread
  accelThr = threads.addThread(accelThread);

  // call PID settings
  courseCorrection.SetMode(AUTOMATIC);
  pathCorrection.SetMode(MANUAL);
  state = ASCEND_STATE;

  servo.setServoControl(SERVO_PIN);
  servo.setKp(1.0);
}

void loop()
{

  // call measurements
  long testT = millis();

  gnssThread();

  Serial.print("gnss time: ");
  Serial.println(millis() - testT);
  testT = millis();

  baroThread();

  Serial.print("baro time: ");
  Serial.println(millis() - testT);

  testT = millis();
  // if(TEST)
  {
    Serial.print("Latitude: ");
    Serial.println(latitude, 7);
    Serial.print("Longitude: ");
    Serial.println(longitude, 7);
    Serial.print("GroundSpeed: ");
    Serial.println(groundSpeed);
    Serial.print("Course to final: ");
    nextPointHeading = setCourse2Points(finLat, finLong);
    Serial.println(nextPointHeading);
    Serial.print("DistanceToFinal: ");
    Serial.println(distance2Points(latitude, longitude, finLat, finLong));
    pidDeltaAngle();
    Serial.print("Delta angle: ");
    Serial.println(pidInput);
    courseCorrection.Compute();
    Serial.print("PID output: ");
    Serial.println(steeringOutput);

    Serial.print("cmpt and out time: ");
    Serial.println(millis() - testT);
    // servo.rotate((int)(steeringOutput), 4);
    // Serial.print("Servo Angle: ");
    // Serial.println(servo.Angle());

    ////////////////////////////
    // Serial.print("Roll (deg): ");
    // Serial.println(roll);
    // Serial.print("Pitch (deg): ");
    // Serial.println(pitch);
    // Serial.print("Yaw (deg): ");
    // Serial.println(yaw);
    // ////////////////////////////

    // ////////////////////////////
    // Serial.print("Baro Altitude (m): ");
    // Serial.println(altitude);
    // Serial.print("Baro Temperature (deg C): ");
    // Serial.println(temperature_baro);
    // Serial.print("Vertical speed (m/s): ");
    // Serial.println(vspeed);
    ////////////////////////////

    // plot drawing ---------------------
    //  nextPointHeading = setCourse2Points(finLat, finLong);
    //  Serial.print(nextPointHeading);
    //  Serial.print(",");
    //  pidDeltaAngle();
    //  Serial.print(pidInput);
    //  Serial.print(",");
    //  courseCorrection.Compute();
    //  Serial.print(steeringOutput);
    //  Serial.print(",");
    //  Serial.println();
  }

  // //   // https://paulbourke.net/geometry/transformationprojection/#:~:text=Updated%20December%201999-,Stereographic,paper%20or%20a%20computer%20display.
}
