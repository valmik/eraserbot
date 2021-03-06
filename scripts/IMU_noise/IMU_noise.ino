#include <Wire.h>

/* Prints sensor information to serial
 * for use on Teensy
 * includes Encoder and IMU data
 * last modified 4.16.2018
 * Reads as "DISTANCE [mm],AcX,AcY,GyZ"
 */

#include <Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
// SCL Pin 19, SDA Pin 18;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

int imax = 10000;
int i = 0;
long AcXVals[10000];
long AcYVals[10000];
long GyZVals[10000];

void loop() {
  if (i > imax) {
    return;
  }
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  AcX = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  AcXVals[i] = AcX;
  AcYVals[i] = AcY;
  GyZVals[i] = GyZ;

  for (j = 0; j <= i; j++) {
    
  }

  Serial.println("Iterations: " + i);
  Serial.println("AcX:");
  Serial.println("    Value: " + AcX);
  //Serial.println("    Mean:  " + AcXMean);
  //Serial.println("    StDev: " + AcXStDev);
  Serial.println("AcY:");
  Serial.println("    Value: " + AcY);
  //Serial.println("    Mean:  " + AcYMean);
  //Serial.println("    StDev: " + AcYStDev);
  Serial.println("GyZ:");
  Serial.println("    Value: " + GyZ);
  //Serial.println("    Mean:  " + GyZMean);
  //Serial.println("    StDev: " + GyZStDev);
  
  delay(333);
  i++;
}
