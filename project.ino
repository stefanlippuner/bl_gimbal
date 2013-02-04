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

// USE config.h FOR SETTINGS !!!


/* Change History
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
#define VERSION 039


/*************************/
/* Include Header Files  */
/*************************/

#include "config.h"
#include "definitions.h"

#include <avr/interrupt.h>
#include <Wire.h>

#include "MPU6050_6Axis_DMP.h"

/*************************/
/* Variables             */
/*************************/

// BL Controller Variables
uint8_t pwmSin[N_SIN];
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

void updateAccelAngleData(float* angleX, float* angleY)
{
  int16_t x_val, y_val, z_val;
  mpu.getAcceleration(&x_val,&y_val,&z_val);
  
*angleX =atan2(-y_val,-z_val)*57.2957795;
*angleY =-atan2(-x_val,-z_val)*57.2957795;

}  




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
  float out = Kp * error + SampleTimeInSecs * Ki * *errorSum + Kd * (error - *errorOld) / (SampleTimeInSecs + 0.000001);
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
      fastMoveMotor(MOTOR_PITCH, pitchDirection); 
      deviderCountPitch=0;
    }
    
    deviderCountRoll++;
    if(deviderCountRoll >= abs(rollDevider))
    {
      fastMoveMotor(MOTOR_ROLL, rollDirection); 
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
  Serial.println("BLGC Starting Initialization ...");
  
  // Calculate Sinus Array
  Serial.println("Calculating Sinus Array ...:");
  for(int i=0; i<N_SIN; i++)
  {
    pwmSin[i] = maxPWM / 2.0 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
    Serial.print(pwmSin[i]); Serial.print(" ");
  }
  Serial.println();
  
  // Initialize Motor Movement
  maxDegPerSecondPitch = MOTORUPDATE_FREQ * 1000 / N_SIN / (N_POLES_MOTOR_PITCH/2) * 360.0;
  maxDegPerSecondRoll = MOTORUPDATE_FREQ * 1000 / N_SIN / (N_POLES_MOTOR_ROLL/2) * 360.0;
  
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
  
  Serial.println("Init MPU done ...");
  if(mpu.testConnection()) 
    Serial.println("MPU6050 connection successful");
  
  LEDPIN_ON
  
  initResolutionDevider();
  Serial.print("Gyro resolution devider in LSB/deg/s = ");
  Serial.println(resolutionDevider);

   // Init BL Controller
  initBlController();
  delay(10 * CC_FACTOR);
  
  // Move Motors to ensure function
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(MOTOR_ROLL, 1); 
    fastMoveMotor(MOTOR_PITCH, 1);
    delay(15 * CC_FACTOR);
  }
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(MOTOR_ROLL, -1); 
    fastMoveMotor(MOTOR_PITCH, -1);
    delay(15 * CC_FACTOR);
  }
  
  delay(40 * CC_FACTOR);
    
  Serial.println("Init BL Control done ...");
  
 // Initialize timer
  timer=micros();

  Serial.println("GoGoGo...");
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
  if(count > 20)
  {
    updateAccelAngleData(&rollAngleACC,&pitchAngleACC);
    sampleTimeACC = (micros()-timerACC)/1000000.0/CC_FACTOR; // in Seconds!
    timerACC=timer;
    count=0;
    
    {Serial.print(sampleTimeACC,5);Serial.print(" ");Serial.println(sampleTimePID,5);}  
  }
    

  gyroRoll = gyroRoll + ACC_WEIGHT * (rollAngleACC - rollSetpoint)/sampleTimeACC;
  gyroPitch = gyroPitch + ACC_WEIGHT * (pitchAngleACC - pitchSetpoint)/sampleTimeACC;
  pitchPID = ComputePID(sampleTimePID,gyroPitch,0.0, &pitchErrorSum, &pitchErrorOld,GYRO_PITCH_Kp,GYRO_PITCH_Ki,GYRO_PITCH_Kd,maxDegPerSecondPitch);
  rollPID = ComputePID(sampleTimePID,gyroRoll,0.0, &rollErrorSum, &rollErrorOld,GYRO_ROLL_Kp,GYRO_ROLL_Ki,GYRO_ROLL_Kd,maxDegPerSecondRoll);

  pitchDevider = constrain(maxDegPerSecondPitch / (pitchPID + 0.000001), -15000,15000);
  pitchDirection = sgn(pitchDevider) * DIR_MOTOR_PITCH;
  rollDevider = constrain(maxDegPerSecondRoll / (rollPID + 0.000001), -15000,15000);
  rollDirection = sgn(rollDevider) * DIR_MOTOR_ROLL;

  // Output stuff
//  if(count == 20)
//    {Serial.print(gyroPitch);Serial.print(" ");Serial.println((pitchAngleACC - pitchSetpoint));}
//    {Serial.print();Serial.print(" ");Serial.println(gyroRoll);}
//  {Serial.print(sampleTimeACC,5);Serial.print(" ");Serial.println(sampleTimePID,5);}
//    Serial.println(sampleTimeACC,6);

}


