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
037 A:
- NEW: Motor Power Management. Two Options: 
  -- Fixed max Torque/Power (caveat: 
  -- Lower Torque/Power for slower Movements -> EXPERIMENTAL
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
#define VERSION 037


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
int pitchDeviderTemp = 15000;
int rollDeviderTemp = 15000;
int pitchDeviderTempOld = 15000;
int rollDeviderTempOld = 15000;
int8_t pitchDirection = 1;
int8_t rollDirection = 1;
int deviderCountPitch = 0;      
int deviderCountRoll = 0;
int freqCounter=0;



// Variables for MPU6050
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
float gyroPitch, gyroRoll; //in deg/s
float pitchGyroOffset,rollGyroOffset,resolutionDevider;
float gyroPitchOld,gyroRollOld;
MPU6050 mpu;// Create MPU object


//Define Variables for PIDs
float sampleTimePID = 0.01;

float pitchSetpoint = 0;
float pitchAngle = 0;
float pitchP = 0;
float pitchID = 0;
float pitchPID = 0;
float pitchErrorSum = 0;
float pitchErrorOld = 0;

float rollSetpoint = 0;
float rollAngle = 0;
float rollP = 0;
float rollID = 0;
float rollPID = 0;
float rollErrorSum = 0;
float rollErrorOld = 0;

//general purpuse timer
unsigned long timer=0;   
unsigned long timer_old;






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

// Interrupt Handler
void dmpDataReady() 
{
  mpuInterrupt = true;
}

// Calculation of pitch and roll, optimized for speed!!! 
// No Error handling in DMP anymore, only first 16bytes of DMP FIFI are read, these are the quarternions
// then FIFO is reset to prevent wrong subsequent readings, but this costs about 260 micros
// Select correct two calculations for angles
void updatePositionFromDmpFast(float *pitch, float *roll) 
{
    mpu.getFIFOBytes(fifoBuffer, 18); // I2C 800000L : 1300-1308 micros fo 42 bytes, ~540 micros for 16bytes
    mpu.dmpGetQuaternion(&q, fifoBuffer); // I2C 800000L : 64-68 micros
//    mpu.resetFIFO(); // Has to be done in case, where less than 42 bytes a read from FIFO // I2C 800000L : 260 micros
    
    *pitch = asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI; // DMP yaw takes ~120 micros   // pitch for my sensor setup
    *roll = atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* 180/M_PI; // DMP pitch takes ~310 micros // roll for my sensor setup
    *roll = sgn(*roll) * 180.0 - *roll;
    //atan2(2*(q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * 180/M_PI ; // DMP roll takes ~310 micros
}

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

void updateRawGyroData()
{
  gyroPitch = (mpu.getRotationY()- pitchGyroOffset)/resolutionDevider;
  gyroRoll = (mpu.getRotationX() - rollGyroOffset)/resolutionDevider;
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

// Simplified I + D only code
float ComputeID(float SampleTimeInSecs, float in, float setPoint, float *errorSum, float *errorOld, float Ki, float Kd)
{
  float error = setPoint - in;

  // Integrate Errors
  *errorSum += error;

  /*Compute I + D Output*/
  float out = SampleTimeInSecs * Ki * *errorSum + Kd * (error - *errorOld) / (SampleTimeInSecs + 0.000001);
  *errorOld = error;

  return out; 
}



/********************************/
/* Motor Control Routines       */
/********************************/
ISR( TIMER1_OVF_vect )
{
  freqCounter++;

  // Move pitch and roll Motor
  deviderCountPitch++;
  if(deviderCountPitch  >= abs(pitchDevider))
  {
    #ifdef USE_POWER_REDUCTION_MOTOR_PITCH
      fastMoveMotorVariablePower(MOTOR_PITCH, pitchDirection,pitchPIDOutput); 
    #else
      fastMoveMotor(MOTOR_PITCH, pitchDirection); 
    #endif  
    deviderCountPitch=0;
  }
    
  deviderCountRoll++;
  if(deviderCountRoll >= abs(rollDevider))
  {
    #ifdef USE_POWER_REDUCTION_MOTOR_ROLL
      fastMoveMotorVariablePower(MOTOR_ROLL, rollDirection,rollPIDOutput); 
    #else
      fastMoveMotor(MOTOR_ROLL, rollDirection); 
    #endif  
    deviderCountRoll=0;
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
  maxDegPerSecondPitch = CC_FACTOR * 1000 / N_SIN / (N_POLES_MOTOR_PITCH/2) * 360.0;
  maxDegPerSecondRoll = CC_FACTOR * 1000 / N_SIN / (N_POLES_MOTOR_ROLL/2) * 360.0;
  delay(500);
  
  // Start I2C and Configure Frequency
  Wire.begin();
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;                            // enable twi module, no interrupt
 

  mpu.initialize();
  delay(500);
  Serial.println("Init MPU done ...");
  if(mpu.testConnection()) Serial.println("MPU6050 connection successful");
  
  LEDPIN_ON
  
  devStatus = mpu.dmpInitialize();
  Serial.println("Init DMP done ...");


  mpu.setDLPFMode(MPU6050_DLPF_BW);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);
  mpu.setRate(4); // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
  
  mpu.resetDMP();

  if (devStatus == 0) 
  {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    attachInterrupt(0, dmpDataReady, RISING);
    Serial.println("Init MPU6050 competely done ... :-)");
  }   

  initResolutionDevider();
  Serial.print("Gyro resolution devider in LSB/deg/s = ");
  Serial.println(resolutionDevider);
   
  pitchGyroOffset = 0;
  rollGyroOffset = 0;
  for(int i = 0; i < 200; i++)
  {
    rollGyroOffset += mpu.getRotationX();
    pitchGyroOffset += mpu.getRotationY();
    delay(5);
  }
  rollGyroOffset = rollGyroOffset / 200;
  pitchGyroOffset = pitchGyroOffset / 200;
  Serial.print("Gyro Offset Roll: ");
  Serial.println(rollGyroOffset);
  Serial.print("Gyro Offset Pitch: ");
  Serial.println(pitchGyroOffset);

   // Init BL Controller
  delay(500);
  initBlController();
  delay(500);
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(MOTOR_ROLL, 1); 
    fastMoveMotor(MOTOR_PITCH, 1);
    delay(500);
  }
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(MOTOR_ROLL, -1); 
    fastMoveMotor(MOTOR_PITCH, -1);
    delay(500);
  }
  
  delay(2000);
    
  Serial.println("Init BL Control done ...");
  
 // Initialize timer
  timer=micros();

  Serial.println("GoGoGo...");

  // Required to prevent NAN error from MPU

  mpu.resetDMP();
  mpu.resetFIFO(); 
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
long int count=0;
void loop() 
{
  if (mpuInterrupt == true)
  {
    sampleTimePID = max((micros()-timer)/1000000.0/CC_FACTOR,0.00001); // in Seconds!
    timer = micros();

    // Get Orientation
    updatePositionFromDmpFast(&pitchAngle,&rollAngle);

    // Calculate I and D Part only !!!
    pitchID = ComputeID(sampleTimePID,pitchAngle,pitchSetpoint, &pitchErrorSum, &pitchErrorOld,DMP_PITCH_Ki,DMP_PITCH_Kd);
    rollID = ComputeID(sampleTimePID,rollAngle,rollSetpoint, &rollErrorSum, &rollErrorOld,DMP_ROLL_Ki,DMP_ROLL_Kd);
    
    mpuInterrupt = false;    
  } 
 
  if(freqCounter > (CC_FACTOR*2)) // runs at ~500Hz
  {
    // Loop Time 0 us
    updateRawGyroData();
    // Loop Time 480 us


    count++;
    if(count%8==0)Serial.println(pitchAngle);

    
    // Optional Low Pass Filter for Gyro Signal
    // filter it with exponentially weighed moving average (EWMA)
    #ifdef LP_FILTER_GYRO
      gyroPitch = LP_ALPHA_GYRO * gyroPitch + (1.0-LP_ALPHA_GYRO) * gyroPitchOld;
      gyroPitchOld=gyroPitch;
      gyroRoll = LP_ALPHA_GYRO * gyroRoll + (1.0-LP_ALPHA_GYRO) * gyroRollOld;
      gyroRollOld=gyroRoll;
    #endif
    // Loop Time 530 us

    pitchP = gyroPitch * GYRO_PITCH_Kp;
    rollP = gyroRoll * GYRO_ROLL_Kp;   
    
    pitchPID = constrain((pitchP + pitchID),-maxDegPerSecondPitch,maxDegPerSecondPitch);
    rollPID = constrain((rollP + rollID),-maxDegPerSecondRoll,maxDegPerSecondRoll);
    // Loop Time 580 us
    

    pitchDevider = constrain(maxDegPerSecondPitch / (pitchPID + 0.000001), -15000,15000);
    rollDevider = constrain(maxDegPerSecondRoll / (rollPID + 0.000001), -15000,15000);
    pitchDirection = sgn(pitchDevider) * DIR_MOTOR_PITCH;
    rollDirection = sgn(rollDevider) * DIR_MOTOR_ROLL;
    
    freqCounter=0;
  }
}










/*    pitchDeviderTemp = constrain(maxDegPerSecondPitch / (pitchPID + 0.000001), -15000,15000);
    rollDeviderTemp = constrain(maxDegPerSecondRoll / (rollPID + 0.000001), -15000,15000);
    // Loop Time 680 us

    #ifdef LP_FILTER_MOTOR
      pitchDeviderTemp = LP_ALPHA_MOTOR * pitchDeviderTemp + (1.0-LP_ALPHA_MOTOR) * pitchDeviderTempOld;
      pitchDeviderTempOld = pitchDeviderTemp;
      rollDeviderTemp = LP_ALPHA_MOTOR * rollDeviderTemp + (1.0-LP_ALPHA_MOTOR) * rollDeviderTempOld;
      rollDeviderTempOld=rollDeviderTemp;
    #endif
    // Loop Time 750 us
    count++;   
  }

//  if(freqCounter % (CC_FACTOR * 4) == 0) // runs at ~250Hz
  if(count>9);
  {
count = 0;
    pitchDevider = pitchDeviderTemp;
    pitchDirection = sgn(pitchDevider) * DIR_MOTOR_PITCH;

    rollDevider = rollDeviderTemp;
    rollDirection = sgn(rollDevider) * DIR_MOTOR_ROLL;
    
  //  Serial.println(pitchDevider);
    freqCounter=0;
  }

*/
