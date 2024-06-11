#include <Wire.h>
#include "AccelStepper.h"
const int MPU = 0x68; // MPU6050 I2C address

float gyroAngleX {};
float gyroAngleY {};
float gyroAngleZ {};
float yaw{};
unsigned long elapsedTime {};
unsigned long currentTime {};
unsigned long previousTime {};

int prevTime{millis()};

// defines pins for motots
constexpr int stepPin { 5 };
constexpr int dirPin { 4 };

AccelStepper stepperL(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Reset the MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0x00);    
  Wire.endTransmission(true);  

  stepperL.setMaxSpeed(4000);
  stepperL.setSpeed(2400);

  delay(20);
}
void loop() {
  int currentTime{millis()};

  float pitch {getPitch()};
  Serial.print(pitch);
  Serial.print('\n');
  prevTime = currentTime;

  if(pitch > 1)
  {
    //Serial.print("PITCH IS < ONE\n");
    //Serial.flush();
    
    stepperL.runSpeed();
  }
  else if(pitch < -1)
  {
    //Serial.print("PITCH IS > MINUS ONE\n");
    //Serial.flush();
    stepperL.runSpeed();
  }
}

float getPitch()
{
  // Read in Acceleration Data //
  Wire.beginTransmission(MPU);
  //constexpr int accelRegister {0x3B};
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 
  // Datasheet says to divide raw accel data by 16384 when between +-2g
  float AccX {(Wire.read() << 8 | Wire.read()) / 16384.0};
  float AccY {(Wire.read() << 8 | Wire.read()) / 16384.0};
  float AccZ {(Wire.read() << 8 | Wire.read()) / 16384.0};

  float accAngleY {atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI};

  // Read in Gyro Data //
  previousTime = currentTime;        
  currentTime = millis();  
  elapsedTime = (currentTime - previousTime) / 1000;
  Wire.beginTransmission(MPU);
  //constexpr int gyroRegister {0x45};
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  // Datasheet says to divide raw gyro data by 131 for 250deg/s range
  float GyroY {(Wire.read() << 8 | Wire.read()) / 131.0};
  
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;

  return 0.96 * gyroAngleY + 0.04 * accAngleY;
}
