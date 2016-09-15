// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9250
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelGyroMag;
uint8_t intCount = 0;

float accelGFact = 2.0/32768.0;
float gyroRadSecFact = PI*250.0/(180.0*32768.0);
float mu0 = 4.0*PI*1.0E-7;   // magnetic permeability [J/m^3]

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
// Factory mag calibration and mag bias
float magCalibration[3] = {0.0, 0.0, 0.0};
float magbias[3] = {0.0, 0.0, 0.0};
float magscale[3] = {1.0, 1.0, 1.0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   _selfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

#define LED_PIN 13
bool blinkState = false;
bool mpuDetected = false;

#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 5
#define I2C_AD0_PIN 4
#define I2C_nCS_PIN 3
#define I2C_INT_PIN 2

void setup() {
  pinMode(I2C_AD0_PIN, OUTPUT);
  digitalWrite(I2C_AD0_PIN, LOW);
  pinMode(I2C_nCS_PIN, OUTPUT);
  digitalWrite(I2C_nCS_PIN, HIGH);
  
  pinMode(I2C_INT_PIN, INPUT);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);
 // verify connection
  mpuDetected = accelGyroMag.testConnection();
  Serial.println("Testing device connections...");
  Serial.println(mpuDetected ? "MPU9250 connection successful" : "MPU9250 connection failed");
 
  // run self tests
//  Serial.println("SelfTest accelerometer and gyro I2C devices...");
//  accelGyroMag.selfTest(_selfTest);
//  Serial.print("x-axis self test: acceleration trim within : ");
//  Serial.print(_selfTest[0],1); Serial.println("% of factory value");
//  Serial.print("y-axis self test: acceleration trim within : ");
//  Serial.print(_selfTest[1],1); Serial.println("% of factory value");
//  Serial.print("z-axis self test: acceleration trim within : ");
//  Serial.print(_selfTest[2],1); Serial.println("% of factory value");
//  Serial.print("x-axis self test: gyration trim within : ");
//  Serial.print(_selfTest[3],1); Serial.println("% of factory value");
//  Serial.print("y-axis self test: gyration trim within : ");
//  Serial.print(_selfTest[4],1); Serial.println("% of factory value");
//  Serial.print("z-axis self test: gyration trim within : ");
//  Serial.print(_selfTest[5],1); Serial.println("% of factory value");
//  accelGyroMag.calibrateMPU9250(gyroBias, accelBias);
//  Serial.print("accel bias: (");
//  for (int i = 0; i < 3; i++)
//  {
//    Serial.print(accelBias[i]);
//    Serial.print(", ");
//  }
//  Serial.println(")");
//  Serial.print("gyro bias: (");
//  for (int i = 0; i < 3; i++)
//  {
//    Serial.print(gyroBias[i]);
//    Serial.print(", ");
//  }
//  Serial.println(")");

  magbias[0] = 0.0;
  magbias[1] = 0.0;
  magbias[2] = 0.0;
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();
  accelGyroMag.initAK8963(magCalibration);
  Serial.print("mag X-Axis sensitivity adjustment value ");
  Serial.println(magCalibration[0], 2);
  Serial.print("mag Y-Axis sensitivity adjustment value ");
  Serial.println(magCalibration[1], 2);
  Serial.print("mag Z-Axis sensitivity adjustment value ");
  Serial.println(magCalibration[2], 2);
  
  accelGyroMag.magcalMPU9250(magbias, magscale);
  Serial.print("AK8963 mag biases (uT): ");
  for (int i=0; i < 3; i++) {
    Serial.print(magbias[i]);
    if (i < 2) Serial.print(", ");
  }
  Serial.print("\nAK8963 mag scale fact: ");
  for (int i=0; i < 3; i++) {
    Serial.print(magscale[i]);
    if (i < 2) Serial.print(", ");
  }
  Serial.print("\n");

  uint8_t gyroFS = accelGyroMag.getFullScaleGyroRange();
  uint8_t accelFS = accelGyroMag.getFullScaleAccelRange();
  
  accelGFact = pow(2.0, double(1+accelFS))/32768.0;
  gyroRadSecFact = PI * 250.0 * pow(2.0, double(gyroFS))/(180.0*32768.0);
  
  Serial.print("full scale range: ");
  Serial.println(accelFS);
  Serial.print("gyro full scale range: ");
  Serial.println(gyroFS);

  float temp = RFduino_temperature(CELSIUS);
  int16_t mpu9250TempCount = accelGyroMag.getTemperature();
  float mpu9250Temp = ((float) mpu9250TempCount) / 333.87 + 21.0;
  Serial.print("RFduino temp: ");
  Serial.print(temp,1);
  Serial.print(", mpu9250 temp: ");
  Serial.print(mpu9250Temp,1);
  Serial.println(" deg C");

  //RFduino_pinWakeCallback(I2C_INT_PIN, HIGH, i2cIntCallback);
  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  float accel[3];
  float gyro[3];
  float magn[3];
  float maguTFact = accelGyroMag.getMaguTFact();
  // read raw accel/gyro/mag measurements from device
  accelGyroMag.getAcceleration(accelCount);
  accelGyroMag.getRotation(gyroCount);
  accelGyroMag.getMagnetometer(magCount);

  // display tab-separated accel/gyro/mag x/y/z values
//  Serial.print("int count: ");
//  Serial.print(intCount);
  Serial.print("a/g/m:\t");
  for (int i = 0; i < 3; i++) {
    accel[i] = accelGFact * accelCount[i];
    Serial.print(accel[i]); Serial.print("\t");
  }
  for (int i = 0; i < 3; i++) {
    gyro[i] = gyroRadSecFact * gyroCount[i];
    Serial.print(gyro[i]); Serial.print("\t");
  }
  accelGyroMag.getCompassMicroTeslas(magn);
  float mag = 0;
  for (int i = 0; i < 3; i++)
  {
//    magn[i] = maguTFact * magCalibration[i] * magCount[i] - magbias[i];
//    magn[i] *= magscale[i];
    Serial.print(magn[i]); Serial.print("\t");
    mag += pow(magn[i]*1.0E-6, 2.0);
  }
  mag /= 2.0 * mu0;
  
  Serial.println(mag);
//  Serial.print("\n");
//  for (int i=0; i < (int)mag; i++)
//    Serial.print("*");
//  Serial.print("\n");

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  intCount = 0;
  delay(100);
}


int i2cIntCallback(uint32_t pin)
{
  if (pin == I2C_INT_PIN) {
    intCount += accelGyroMag.getIntStatus();
//    Serial.print("interrupt: ");
//    Serial.println(intCount);
//    RFduinoBLE.send(1);
    // get a cpu temperature sample
    // degrees F (-198.00 to +260.00)
    // degrees C (-128.00 to +127.00)
//    float temp = RFduino_temperature(CELSIUS);

//    Serial.print(temp);
//    Serial.println(" deg C");
    // send the sample to the app
  //  RFduinoBLE.sendFloat(temp);
  }

  return 0;
}

