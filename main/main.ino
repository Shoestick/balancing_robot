#include <ArduinoSTL.h>
#include <Wire.h>
#include <vector>
#include "MobaTools.h"
const int MPU = 0x68; // MPU6050 I2C address

// PID coefficients
double kP { 1 };
double kI { 0 };
double kD { 0 };

double errorPrev { 0 };
double error { 0 };
double integral { 0 };

//how large pitch needs to be to start motors
constexpr double motorThreshold { 0.02 };

float gyroAngleX {};
float gyroAngleY {};
float gyroAngleZ {};
float yaw{};
unsigned long elapsedTime {};
unsigned long currentTime {};
unsigned long previousTime {};

// vector to average pitch
// high slow to respond vs. low jittery and noisy
constexpr int avgLen { 80 };
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

  double pitch {getPitch()};

  errorPrev = error;
  error = getPitchAvg(pitch);

  integral += error / elapsedTime;

  // differentials values are otherwise too low to detect. high makes noise, low makes undetectable
  constexpr int diffResolution { 20 };
  double differential = (error - errorPrev) * diffResolution / elapsedTime;

  double output { kP * error + kI * integral + kD * differential };

  //Serial.println(output);
  /*Serial.print(", ");
  Serial.println(elapsedTime);*/

  int speed { abs(output) * 2000 };
  int stepSize { output * 50 };
  stepperR.setSpeed( speed );
  stepperL.setSpeed( speed );

  if(output >= motorThreshold)
  {
    stepperR.doSteps( -stepSize );
    stepperL.doSteps( stepSize );
    //stepperR.rotate( -1 );
    //stepperL.rotate( 1 );
  }
  else if(output <= -motorThreshold)
  {
    stepperR.doSteps( -stepSize );
    stepperL.doSteps( stepSize );
    //stepperR.rotate( 1 );
    //stepperL.rotate( -1 );
  }
}

double getPitchAvg(double pitch)
{
  pitchData[counter++ % avgLen] = pitch;
  double pitchTotal { 0 };
  for(auto const &e : pitchData)
  {
    pitchTotal += e;
  }
  return pitchTotal / avgLen;
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
  elapsedTime = (currentTime - previousTime);
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
