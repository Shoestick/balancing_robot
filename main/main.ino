#include <Wire.h>
#include "MobaTools.h"
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
constexpr int stepPinL { 5 };
constexpr int dirPinL { 4 };
constexpr int stepPinR { 3 };
constexpr int dirPinR { 2 };

constexpr int stepsPerRevolution { 800 };
MoToStepper stepperL(stepsPerRevolution, STEPDIR);
MoToStepper stepperR(stepsPerRevolution, STEPDIR);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Reset the MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0x00);    
  Wire.endTransmission(true);  

  constexpr int speed { 400 };
  stepperR.attach( stepPinR, dirPinR );
  stepperL.attach( stepPinL, dirPinL );
  stepperR.setSpeed( speed );
  stepperL.setSpeed( speed );

  delay(20);
}
void loop() {
  //time variables used in pitch calculation
  int currentTime{millis()};
  float pitch {getPitch()};
  Serial.print(pitch);
  Serial.print('\n');
  prevTime = currentTime;

  if(pitch > 0.5)
  {
    stepperR.doSteps( -10 );
    stepperL.doSteps( 10 );
  }
  else if(pitch < -0.5)
  {
    stepperR.doSteps( 10 );
    stepperL.doSteps( -10 );
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
