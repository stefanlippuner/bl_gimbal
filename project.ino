/*
Brushless Gimbal Controller by 
Ludwig Färber 
Alexander Rehfeld
Christian Winkler

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version. see <http://www.gnu.org/licenses/>

Thanks To : 
-Miniolli float
-brettbeauregard
-jrowberg
-rgsteele
*/

// Serial Programming for Settings!!!
/* HOWTO:
- edit definitions.h, if you must (MPU Address).
- edit setDefaultParameters() in this file if you want to.
- Upload Firmware.
- Open Arduino Terminal and enable NL in the lower right corner of the window.
- Type in HE 
-... enjoy
*/


/* Change History
041 A: MAJOR UPDATE!!!
- removed "config.h", added serial protocol
-- configurable parameters now stored in eeprom
-- relevant parameters can be changes online now
- Still: floating point math!!!
-CAVEAT only 100%Power for now, was not able to finish that this weekend.

039 A: MAJOR REWORK!!!
- Removed usage of DMP completely
- Relay on raw Gyro and raw ACC only! 
Gyro is used at ~1kHz to counter movements, ACC vs set point
is mixed into gyro signal to ensure horizontal camera (IMU) position
- global max PWM duty cycle and Power devider per motor can be configured in config.h 
Hint: lower torque = lower power allows for higher P on Pitch for me.
- code cleanup, removed obsolete stuff.
- 32kHz PWM works now for motor movement updates for up to 8 kHz. 
( No more beeping :-), i dont care for energy loss at the moment )

038 A: Test version, not published
037 A:
- NEW: Motor Power Management. Two Options: 
  -- Fixed max Torque/Power (caveat: 
  -- Lower Torque/Power for slower Movements -> EXPERIMENTAL (removed in 039)
- NEW: Use DMP output for I and D Part in control loop at 100Hz (or 200Hz), use raw Gyro at 500Hz for P-Part 
  -- Caveat: setting the sample rate to 500Hz for gyro screws up the dmp algorithm for now. 
     Some more research required ro configure the mpu correctly.
     Therefore sample rate for gyro is set to 200Hz as well, resulting in loss of accuracy.
  --> Remove choice of DMP/RAW_GYRO
- Code cleanup
- Removed the wiring.c modificitaions, does not work as intented anyway --> back to CC_FACTOR usage
- Moved some definitions from config.h to definitions.h, dont change them for now.
- Switched off Gyro and Accel write to DMP-FIFO, speeding up the code by ~200us per DMP read

036 A:
- Choose between DMP and Raw Gyro Stabilisation
  (Raw Gyro is only a tech demo implementation for now: P controller only, no setpoint)
- Faster Motor routine using uint8_t overflow for counter. Sinus Array length therefore is fixed now to 256.
- Raw Gyro can be filtered with low pass (change alpha below)
- YOU HAVE TO CHANGE wiring.c, see below !!!!
- DOES IT WORK? Well....no fo now.
*/


#define VERSION_STATUS A // A = Alpha; B = Beta , N = Normal Release
#define VERSION 041


/*************************/
/* Include Header Files  */
/*************************/
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "definitions.h"
#include "MPU6050_6Axis_DMP.h"
#include "SerialCommand.h"
#include "EEPROMAnything.h"

/*************************/
/* Config Structure      */
/*************************/

struct config_t
{
uint8_t vers;
int16_t gyroPitchKp; 
int16_t gyroPitchKi;   
int16_t gyroPitchKd;
int16_t gyroRollKp;
int16_t gyroRollKi;
int16_t gyroRollKd;
int16_t accelWeight;
uint8_t nPolesMotorPitch;
uint8_t nPolesMotorRoll;
int8_t dirMotorPitch;
int8_t dirMotorRoll;
uint8_t motorNumberPitch;
uint8_t motorNumberRoll;
uint8_t maxPWMmotorPitch;
uint8_t maxPWMmotorRoll;
} config;

void setDefaultParameters()
{
  config.vers = VERSION;
  config.gyroPitchKp = 3400;
  config.gyroPitchKi = 5;
  config.gyroPitchKd = 20;
  config.gyroRollKp = 2800;
  config.gyroRollKi = 5;
  config.gyroRollKd = 150;
  config.accelWeight = 25;
  config.nPolesMotorPitch = 14;
  config.nPolesMotorRoll = 14;
  config.dirMotorPitch = -1;
  config.dirMotorRoll = -1;
  config.motorNumberPitch = 0;
  config.motorNumberRoll = 1;
  config.maxPWMmotorPitch = 2;
  config.maxPWMmotorRoll = 4;
}


/*************************/
/* Variables             */
/*************************/
SerialCommand sCmd; // Create SerialCommand object

// BL Controller Variables
prog_uint8_t pwmSin100[] = {127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,179,182,184,187,190,193,195,198,200,203,205,208,210,213,215,217,219,221,224,226,228,229,231,233,235,236,238,239,241,242,244,245,246,247,248,249,250,251,251,252,253,253,254,254,254,254,254,255,254,254,254,254,254,253,253,252,251,251,250,249,248,247,246,245,244,242,241,239,238,236,235,233,231,229,228,226,224,221,219,217,215,213,210,208,205,203,200,198,195,193,190,187,184,182,179,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,75,72,70,67,64,61,59,56,54,51,49,46,44,41,39,37,35,33,30,28,26,25,23,21,19,18,16,15,13,12,10,9,8,7,6,5,4,3,3,2,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,2,3,3,4,5,6,7,8,9,10,12,13,15,16,18,19,21,23,25,26,28,30,33,35,37,39,41,44,46,49,51,54,56,59,61,64,67,70,72,75,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124};
prog_uint8_t pwmSin75[] = {95,97,99,102,104,107,109,111,114,116,118,120,123,125,127,129,132,134,136,138,140,142,144,146,148,150,152,153,156,157,159,161,162,164,165,168,169,171,171,173,174,176,177,178,179,180,181,183,183,184,185,186,186,187,188,188,189,189,189,190,190,190,190,190,191,190,190,190,190,190,189,189,189,188,188,187,186,186,185,184,183,183,181,180,179,178,177,176,174,173,171,171,169,168,165,164,162,161,159,157,156,153,152,150,148,146,144,142,140,138,136,134,132,129,127,125,123,120,118,116,114,111,109,107,104,102,99,97,95,93,90,88,86,83,81,78,76,74,72,69,67,65,63,60,58,56,54,52,50,48,45,44,42,40,38,36,34,33,30,29,27,26,24,22,21,19,18,17,15,14,13,12,11,9,9,7,6,6,5,4,3,3,2,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,2,3,3,4,5,6,6,7,9,9,11,12,13,14,15,17,18,19,21,22,24,26,27,29,30,33,34,36,38,40,42,44,45,48,50,52,54,56,58,60,63,65,67,69,72,74,76,78,81,83,86,88,90,93};
prog_uint8_t pwmSin60[] = {76,78,79,81,83,85,87,89,91,93,94,96,98,100,102,103,105,107,109,110,112,114,115,117,118,120,121,123,124,126,127,129,130,131,132,134,135,136,137,138,139,141,141,142,143,144,145,146,147,147,148,148,149,150,150,150,151,151,151,152,152,152,152,152,153,152,152,152,152,152,151,151,151,150,150,150,149,148,148,147,147,146,145,144,143,142,141,141,139,138,137,136,135,134,132,131,130,129,127,126,124,123,121,120,118,117,115,114,112,110,109,107,105,103,102,100,98,96,94,93,91,89,87,85,83,81,79,78,76,74,72,70,69,66,64,63,61,59,57,55,54,52,50,48,46,45,43,42,40,38,36,35,33,32,30,29,27,26,24,23,22,21,19,18,16,15,15,13,12,11,10,9,9,7,7,6,5,4,4,3,3,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,3,3,4,4,5,6,7,7,9,9,10,11,12,13,15,15,16,18,19,21,22,23,24,26,27,29,30,32,33,35,36,38,40,42,43,45,46,48,50,52,54,55,57,59,61,63,64,66,69,70,72,74};
prog_uint8_t pwmSin45[] = {57,58,59,61,62,64,65,67,68,69,71,72,73,75,76,77,79,80,81,82,84,85,86,87,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,103,104,105,106,107,107,108,108,109,110,110,111,111,112,112,112,112,113,113,113,114,114,114,114,114,114,114,114,114,114,114,113,113,113,112,112,112,112,111,111,110,110,109,108,108,107,107,106,105,104,103,103,102,101,100,99,98,97,96,95,94,93,92,91,90,89,87,86,85,84,82,81,80,79,77,76,75,73,72,71,69,68,67,65,64,62,61,59,58,57,55,54,53,51,49,48,47,45,44,43,41,40,39,37,36,35,33,32,31,30,28,27,26,25,24,22,22,20,19,18,17,16,15,14,13,12,11,11,10,9,8,8,7,6,5,5,4,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,4,5,5,6,7,8,8,9,10,11,11,12,13,14,15,16,17,18,19,20,22,22,24,25,26,27,28,30,31,32,33,35,36,37,39,40,41,43,44,45,47,48,49,51,53,54,55};
prog_uint8_t pwmSin30[] = {38,39,39,40,41,42,43,44,45,46,47,48,49,50,51,51,52,53,54,55,56,57,57,58,59,60,60,61,62,63,63,64,65,65,66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,73,74,74,74,75,75,75,75,75,75,76,76,76,76,76,76,76,76,76,76,76,75,75,75,75,75,75,74,74,74,73,73,73,72,72,71,71,70,70,69,69,68,68,67,67,66,65,65,64,63,63,62,61,60,60,59,58,57,57,56,55,54,53,52,51,51,50,49,48,47,46,45,44,43,42,41,40,39,39,38,37,36,35,34,33,32,31,30,29,28,27,27,26,25,24,23,22,21,21,20,19,18,17,16,16,15,14,13,13,12,11,11,10,9,9,8,7,7,6,6,5,5,4,4,3,3,3,2,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,2,3,3,3,4,4,5,5,6,6,7,7,8,9,9,10,11,11,12,13,13,14,15,16,16,17,18,19,20,21,21,22,23,24,25,26,27,27,28,29,30,31,32,33,34,35,36,37};

float maxDegPerSecondPitch,maxDegPerSecondRoll;
uint8_t currentStepMotor0 = 0;
uint8_t currentStepMotor1 = 0;

int pitchDevider = 15000;
int rollDevider = 15000;

int8_t pitchDirection = 1;
int8_t rollDirection = 1;

int deviderCountPitch = 0;      
int deviderCountRoll = 0;

int freqCounter=0;



// Variables for MPU6050
float gyroPitch, gyroRoll; //in deg/s
float xGyroOffset,yGyroOffset,resolutionDevider;
float gyroPitchOld,gyroRollOld;
int16_t x_val, y_val, z_val;

MPU6050 mpu;// Create MPU object


//Define Variables for PIDs
float sampleTimePID = 0.01;
float sampleTimeACC = 0.1;

float pitchSetpoint = 0;
float pitchAngle = 0;
float pitchAngleACC = 0;
float pitchPID = 0;
float pitchErrorSum = 0;
float pitchErrorOld = 0;

float rollSetpoint = 0;
float rollAngle = 0;
float rollAngleACC = 0;
float rollPID = 0;
float rollErrorSum = 0;
float rollErrorOld = 0;

//general purpuse timer
unsigned long timer=0;   
unsigned long timerACC=0;






/*************************/
/* General Purpose Functs*/
/*************************/

int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}
/*************************/
/* MPU6050 Routines      */
/*************************/

// Read Raw gyro data
/* FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 */

void initResolutionDevider()
{
    if(MPU6050_GYRO_FS == 0x00) resolutionDevider = 131.0;
    if(MPU6050_GYRO_FS == 0x01) resolutionDevider = 65.5;
    if(MPU6050_GYRO_FS == 0x02) resolutionDevider = 32.8;
    if(MPU6050_GYRO_FS == 0x03) resolutionDevider = 16.4;
}

void updateRawGyroData(float* gyroX, float* gyroY)
{
  int16_t x,y;
  mpu.getRotationXY(&x,&y);
  
  *gyroX = (x-xGyroOffset)/resolutionDevider;//(mpu.getRotationY()- pitchGyroOffset)/resolutionDevider;
  *gyroY = (y-yGyroOffset)/resolutionDevider;//(mpu.getRotationX() - rollGyroOffset)/resolutionDevider;
}


// This functions performs an initial gyro offset calibration
// Board should be still for some seconds 
void gyroOffsetCalibration()
{
  xGyroOffset=0; yGyroOffset=0; 
  for(int i=0;i<500;i++)
  {
    xGyroOffset += mpu.getRotationX();
    yGyroOffset += mpu.getRotationY();
    delay(10);
  }
  
  xGyroOffset=xGyroOffset/500.0;
  yGyroOffset=yGyroOffset/500.0;
  
  Serial.print("X Ygro Offset = "); Serial.println(xGyroOffset);
  Serial.print("Y Ygro Offset = "); Serial.println(yGyroOffset);
}



/*
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
*/


/************************/
/* PID Controller       */
/************************/

// Simplified PID code
float ComputePID(float SampleTimeInSecs, float in, float setPoint, float *errorSum, float *errorOld, float Kp, float Ki, float Kd, float maxDegPerSecond)
{
  float error = setPoint - in;

  // Integrate Errors
  *errorSum += error;
  *errorSum = constrain(*errorSum, -maxDegPerSecond ,maxDegPerSecond);
 
  /*Compute PID Output*/
  float out = (Kp * error + SampleTimeInSecs * Ki * *errorSum + Kd * (error - *errorOld) / (SampleTimeInSecs + 0.000001))/1000.0;
  *errorOld = error;

  return constrain(out, -maxDegPerSecond ,maxDegPerSecond);
}




/********************************/
/* Motor Control Routines       */
/********************************/
ISR( TIMER1_OVF_vect )
{
  freqCounter++;
  if(freqCounter==(CC_FACTOR/MOTORUPDATE_FREQ))
  {

    // Move pitch and roll Motor
    deviderCountPitch++;
    if(deviderCountPitch  >= abs(pitchDevider))
    {
      fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin100); 
/*      if(config.maxPWMmotorPitch == 5) fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin100); 
      else if(config.maxPWMmotorPitch == 4) fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin75); 
      else if(config.maxPWMmotorPitch == 3) fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin60); 
      else if(config.maxPWMmotorPitch == 2) fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin45); 
      else if(config.maxPWMmotorPitch == 1) fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin30);
      else fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSin100); 
*/     deviderCountPitch=0;
    }
    
    deviderCountRoll++;
    if(deviderCountRoll >= abs(rollDevider))
    {
      fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin100);
/*      if(config.maxPWMmotorRoll == 5) fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin100);
      else if(config.maxPWMmotorRoll == 4) fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin75);
      else if(config.maxPWMmotorRoll == 3) fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin60);
      else if(config.maxPWMmotorRoll == 2) fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin45);
      else if(config.maxPWMmotorRoll == 1) fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin30);
      else fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSin100);
*/      deviderCountRoll=0;
    }
    freqCounter=0;
  }
}

/**********************************************/
/* Initialization                             */
/**********************************************/
void setup() 
{
  LEDPIN_PINMODE
    
  // Start Serial Port
  Serial.begin(115200);
  Serial.println("BLGC Starting Initialization ...");

  // Set Serial Protocol Commands
  Serial.println(F("Starting Serial Protocol ..."));
  setSerialProtocol();
  
  // Read Config or fill with default settings
  Serial.println(F("Initializing default values from EEPROM ..."));
  if(EEPROM.read(0)==VERSION)
  {
    EEPROM_readAnything(0, config);
  }
  else
  {
    setDefaultParameters();
    EEPROM_writeAnything(0, config);
  }
  
  // Initialize Motor Movement
  maxDegPerSecondPitch = MOTORUPDATE_FREQ * 1000.0 / N_SIN / (config.nPolesMotorPitch/2) * 360.0;
  maxDegPerSecondRoll = MOTORUPDATE_FREQ * 1000.0 / N_SIN / (config.nPolesMotorRoll/2) * 360.0;
  
  // Start I2C and Configure Frequency
  Wire.begin();
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;                            // enable twi module, no interrupt
 
  // Initialize MPU 
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       //+- 2G
  mpu.setDLPFMode(MPU6050_DLPF_BW);                     // Set Gyro Low Pass Filter to config.h
  mpu.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
  mpu.setSleepEnabled(false); 
  
  gyroOffsetCalibration();
  
  Serial.println(F("Init MPU done ..."));
  if(mpu.testConnection()) 
    Serial.println(F("MPU6050 connection successful"));
  
  LEDPIN_ON
  
  initResolutionDevider();
  Serial.print(F("Gyro resolution devider in LSB/deg/s = "));
  Serial.println(resolutionDevider);

   // Init BL Controller
  initBlController();
  delay(10 * CC_FACTOR);
  
  // Move Motors to ensure function
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(config.motorNumberRoll, 1,pwmSin100);
    fastMoveMotor(config.motorNumberPitch, 1,pwmSin100);
  }
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(config.motorNumberRoll, -1,pwmSin100); 
    fastMoveMotor(config.motorNumberPitch, -1,pwmSin100); 
    delay(15 * CC_FACTOR);
  }
  
  delay(40 * CC_FACTOR);
    
  Serial.println(F("Init BL Control done ..."));
  
 // Initialize timer
  timer=micros();

  Serial.println(F("READY! Type HE for help, activate NL in Arduino Terminal!!!..."));
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
int count=0;
void loop() 
{
  count++;

  sampleTimePID = (micros()-timer)/1000000.0/CC_FACTOR; // in Seconds!
  timer = micros();
   
  // Update raw Gyro
  updateRawGyroData(&gyroRoll,&gyroPitch);
    
  // Update ACC data approximately at 50Hz to save calculation time.
  if(count == 20)
  {
    mpu.getAcceleration(&x_val,&y_val,&z_val);
 
    sampleTimeACC = (micros()-timerACC)/1000.0/CC_FACTOR; // in Seconds * 1000.0 to account for factor 1000 in parameters
    timerACC=timer;
    //{Serial.print(sampleTimeACC,5);Serial.print(" ");Serial.println(sampleTimePID,5);}  
  }
  if(count == 21) rollAngleACC =atan2(-y_val,-z_val)*57.2957795;
  if(count == 22)
  {
     pitchAngleACC =-atan2(-x_val,-z_val)*57.2957795;
     count=0;
  }
  
  //Serial.println( (micros()-timer)/CC_FACTOR);
    
  gyroRoll = gyroRoll + config.accelWeight * (rollAngleACC - rollSetpoint)/sampleTimeACC;
  gyroPitch = gyroPitch + config.accelWeight * (pitchAngleACC - pitchSetpoint)/sampleTimeACC;
  pitchPID = ComputePID(sampleTimePID,gyroPitch,0.0, &pitchErrorSum, &pitchErrorOld,config.gyroPitchKp,config.gyroPitchKi,config.gyroPitchKd,maxDegPerSecondPitch);
  rollPID = ComputePID(sampleTimePID,gyroRoll,0.0, &rollErrorSum, &rollErrorOld,config.gyroRollKp,config.gyroRollKi,config.gyroRollKd,maxDegPerSecondRoll);

  pitchDevider = constrain(maxDegPerSecondPitch / (pitchPID + 0.000001), -15000,15000);
  pitchDirection = sgn(pitchDevider) * config.dirMotorPitch;
  rollDevider = constrain(maxDegPerSecondRoll / (rollPID + 0.000001), -15000,15000);
  rollDirection = sgn(rollDevider) * config.dirMotorRoll;

  sCmd.readSerial(); 
}


