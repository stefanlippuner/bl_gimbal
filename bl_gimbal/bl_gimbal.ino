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
042 A: 
- memory optimizations
- reintroduce a way to motor power control (use fixed progmem arrays):
PWM from 1 to 255

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
#define VERSION 42


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
  config.maxPWMmotorPitch = 120;
  config.maxPWMmotorRoll = 180;
}


/*************************/
/* Variables             */
/*************************/
SerialCommand sCmd; // Create SerialCommand object

uint8_t pwmSinMotorPitch[256];
uint8_t pwmSinMotorRoll[256];

float maxDegPerSecondPitch;
float maxDegPerSecondRoll;
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
float gyroPitch;
float gyroRoll; //in deg/s
float xGyroOffset;
float yGyroOffset;
float resolutionDevider;
float gyroPitchOld;
float gyroRollOld;
int16_t x_val;
int16_t y_val;
int16_t z_val;

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

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void calcSinusArray(uint8_t maxPWM, uint8_t *array)
{
  for(int i=0; i<N_SIN; i++)
  {
    array[i] = maxPWM / 2.0 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
  }  
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
      fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSinMotorPitch); 
      deviderCountPitch=0;
    }
    
    deviderCountRoll++;
    if(deviderCountRoll >= abs(rollDevider))
    {
      fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSinMotorRoll);
      deviderCountRoll=0;
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

  // Set Serial Protocol Commands
  setSerialProtocol();
  
  // Read Config or fill with default settings
  if(EEPROM.read(0)==VERSION)
  {
    EEPROM_readAnything(0, config);
  }
  else
  {
    setDefaultParameters();
    EEPROM_writeAnything(0, config);
  }

  // Init Sinus Arrays  
  calcSinusArray(config.maxPWMmotorPitch,pwmSinMotorPitch);
  calcSinusArray(config.maxPWMmotorRoll,pwmSinMotorRoll);

  
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
  
  if(mpu.testConnection()) 
    Serial.println(F("MPU6050 ok"));
  
  LEDPIN_ON
  
  initResolutionDevider();
  
   // Init BL Controller
  initBlController();
  delay(10 * CC_FACTOR);
  
  // Move Motors to ensure function
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(config.motorNumberRoll, 1,pwmSinMotorRoll);
    fastMoveMotor(config.motorNumberPitch, 1,pwmSinMotorPitch);
  }
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(config.motorNumberRoll, -1,pwmSinMotorRoll); 
    fastMoveMotor(config.motorNumberPitch, -1,pwmSinMotorPitch); 
    delay(15 * CC_FACTOR);
  }
  
  delay(40 * CC_FACTOR);
    
 // Initialize timer
  timer=micros();

  Serial.println(F("GO! Type HE for help, activate NL in Arduino Terminal!"));
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
//  Serial.println(freeRam ());

  sCmd.readSerial(); 
}


