#include <ArduinoSTL.h>
#include <Wire.h>
#include <vector>
#include "MobaTools.h"
const int MPU = 0x68; // MPU6050 I2C address

float gyroAngleX {};
float gyroAngleY {};
float gyroAngleZ {};
float yaw{};
unsigned long elapsedTime {};
unsigned long currentTime {};
unsigned long previousTime {};

constexpr int avgLen { 90 };
std::vector<double> pitchData( avgLen );

int counter{ 0 };

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

  
  stepperR.attach( stepPinR, dirPinR );
  stepperL.attach( stepPinL, dirPinL );
  

  delay(20);
}

void loop() {
  //time variables used in pitch calculation
  int currentTime{millis()};
  double pitchRaw {getPitch()};
  //Serial.print("R: ");
  

  //calc average pitch
  pitchData[counter++ % avgLen] = pitchRaw;
  double pitchTotal { 0 };
  for(auto const &e : pitchData)
  {
    pitchTotal += e;
  }

  double pitch = pitchTotal / avgLen;

  Serial.println(elapsedTime);

  //print avg pitch
  //Serial.print("A: ");
  //Serial.print('\n');

  int speed { abs(pitch) * 2000 };
  stepperR.setSpeed( speed );
  stepperL.setSpeed( speed );

  if(pitch >= 0.01)
  {
    stepperR.rotate( -1 );
    stepperL.rotate( 1 );
  }
  else if(pitch <= -0.01)
  {
    stepperR.rotate( 1 );
    stepperL.rotate( -1 );
  }
}

double getPitch()
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
  elapsedTime = (currentTime - previousTime) ;
  Wire.beginTransmission(MPU);
  //constexpr int gyroRegister {0x45};
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  // Datasheet says to divide raw gyro data by 131 for 250deg/s range
  float GyroY {(Wire.read() << 8 | Wire.read()) / 131.0};
  
  gyroAngleY = gyroAngleY + GyroY * (elapsedTime / 1000);

  return 0.96 * gyroAngleY + 0.04 * accAngleY;
}
