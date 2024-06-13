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

//turn into array if this works
//possibly make doubles because of conversion
float pitch0 { 0 };
float pitch1 { 0 };
float pitch2 { 0 };
float pitch3 { 0 };
float pitch4 { 0 };

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
//lowest speed 100, stepSize 10, 5, 2 works
//let's try 1/20th of speed for step size
void loop() {
  //time variables used in pitch calculation
  int currentTime{millis()};
  float rawPitch {getPitch()};
  Serial.print("R: ");
  Serial.print(rawPitch);
  Serial.print('\n');
  prevTime = currentTime;

  //calc average pitch
  switch(counter++ % 5)
  {
    case 0:
      pitch0 = rawPitch;
      break;
    case 1:
      pitch1 = rawPitch;
      break;
    case 2:
      pitch2 = rawPitch;
      break;
    case 3:
      pitch3 = rawPitch;
      break;
    case 4:
      pitch4 = rawPitch;
      break;
  }
  float pitch = ( pitch0 + pitch1 + pitch2 + pitch3 + pitch4 ) / 5;

  //print avg pitch
  Serial.print("A: ");
  Serial.print(pitch);
  Serial.print('\n');

  int speed { abs(pitch) * 1000 };
  int stepSize { pitch * 50 };
  stepperR.setSpeed( speed );
  stepperL.setSpeed( speed );

  if(pitch > 0.2)
  {
    stepperR.doSteps( -stepSize );
    stepperL.doSteps( stepSize );
  }
  else if(pitch < -0.2)
  {
    stepperR.doSteps( -stepSize );
    stepperL.doSteps( stepSize );
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
