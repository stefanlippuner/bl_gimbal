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

/* Change History
036 A:
- Choose between DMP and Raw Gyro Stabilisation
  (Raw Gyro is only a tech demo implementation for now: P controller only, no setpoint)
- Faster Motor routine using uint8_t overflow for counter. Sinus Array length therefore is fixed now to 256.
- Raw Gyro can be filtered with low pass (change alpha below)
- YOU HAVE TO CHANGE wiring.c, see below !!!!
- DOES IT WORK? Well....no fo now.
*/






/* 
ATTENTION:
Change Arduino wiring.c, ~Line 27, to:

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#ifndef MICROSECONDS_PER_TIMER0_OVERFLOW
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#endif
*/

#define VERSION_STATUS A // A = Alpha; B = Beta , N = Normal Release
#define VERSION 036


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
int pitchDevider,rollDevider;
int pitchDirection,rollDirection;
int deviderCountPitch = 0;      // Fast Interrupt driven loop for Motor control 
int deviderCountRoll = 0;
uint8_t freqCounter=0;

// Variables for MPU6050
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[16]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
float gyroPitch, gyroRoll; //in deg/s
float pitchGyroOffset,rollGyroOffset,resolutionDevider;
float gyroPitchOld,gyroRollOld;
MPU6050 mpu;// Create MPU object


//Define Variables for PIDs
float sampleTimePID;
float error;
float pitchSetpoint = 0;
float pitchInput = 0;
float pitchOutput = 0;
float pitchErrorSum = 0;
float pitchErrorOld = 0;
float rollSetpoint = 0;
float rollInput = 0;
float rollOutput = 0;
float rollErrorSum = 0;
float rollErrorOld = 0;

//general purpuse timer
unsigned long timer=0;   
unsigned long timer_old;








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
    mpu.getFIFOBytes(fifoBuffer, 16); // I2C 800000L : 1300-1308 micros fo 42 bytes, ~540 micros for 16bytes
    mpu.dmpGetQuaternion(&q, fifoBuffer); // I2C 800000L : 64-68 micros
    mpu.resetFIFO(); // Has to be done in case, where less than 42 bytes a read from FIFO // I2C 800000L : 260 micros
    
    *pitch = asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI; // DMP yaw takes ~120 micros   // pitch for my sensor setup
    *roll = atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* 180/M_PI; // DMP pitch takes ~310 micros // roll for my sensor setup
    *roll = sgn(*roll) * 180.0 - *roll;
    //atan2(2*(q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * 180/M_PI ; // DMP roll takes ~310 micros
}


// Read Raw gyro data and filter it with exponentially weighed moving average (EWMA)
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

float alpha = 0.25;
void updateRawGyroData()
{
  gyroPitch = (mpu.getRotationY()- pitchGyroOffset)/resolutionDevider;
  gyroPitch = alpha * gyroPitch + (1.0-alpha) * gyroPitchOld;
  gyroPitchOld=gyroPitch;
 
  gyroRoll = (mpu.getRotationX() - rollGyroOffset)/resolutionDevider;
  gyroRoll = alpha * gyroRoll + (1.0-alpha) * gyroRollOld;
  gyroRollOld=gyroRoll;
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
  float out = Kp * error + SampleTimeInSecs * Ki * *errorSum + Kd * (error - *errorOld) / SampleTimeInSecs;
  *errorOld = error;

  return constrain(out, -maxDegPerSecond ,maxDegPerSecond);
}


/********************************/
/* Motor Control Routines       */
/********************************/
ISR( TIMER1_OVF_vect )
{
  #ifdef USE_RAW_GYRO
    freqCounter++;
  #endif

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
    fastMoveMotor(MOTOR_ROLL, rollDirection ); 
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
  deviderCountPitch = 0;
  deviderCountRoll = 0;
  pitchDevider = 10000;
  rollDevider = 10000;
  pitchDirection = sgn(pitchDevider);
  rollDirection = sgn(rollDevider);
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
  
  #ifdef USE_DMP
    devStatus = mpu.dmpInitialize();
    Serial.println("Init DMP done ...");
 
    if (devStatus == 0) 
    {
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      attachInterrupt(0, dmpDataReady, RISING);
      Serial.println("Init MPU6050 competely done ... :-)");
    }   

    // Read a couple of values from DMP to ensure "good" values
    // Otherwise you might end up with NAN in PID calculations
    // Also the DMP needs a couple of seconds to calibrate itself
    timer=millis();
    delay((DMP_INIT_TIME-1)*1000);
    while((millis()-timer)/1000 <= DMP_INIT_TIME)
    {
      if (mpuInterrupt == true)
      {
        updatePositionFromDmpFast(&pitchInput,&rollInput);
        mpuInterrupt = false;    
      }
    }
  #endif
  
  #ifdef USE_RAW_GYRO
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);
    mpu.setDLPFMode(MPU6050_DLPF_BW);
    
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
  #endif

   // Init BL Controller
  delay(500);
  initBlController();
  delay(500);
  Serial.println("Init BL Control done ...");
  
 // Initialize timer
  timer=micros();

  Serial.println("GoGoGo...");
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/

#ifdef USE_DMP
void loop() 
{
  //if (!dmpReady) return; // Only Start the Programm if MPU are Ready ! 

  dmpReady=true;
  
  if (mpuInterrupt == true)
  {
    sampleTimePID = (micros()-timer)/1000000.0; // in Seconds!
    timer = micros();

    updatePositionFromDmpFast(&pitchInput,&rollInput);
    
    pitchOutput = ComputePID(sampleTimePID,pitchInput,pitchSetpoint, &pitchErrorSum, &pitchErrorOld,DMP_PITCH_Kp,DMP_PITCH_Ki,DMP_PITCH_Kd,maxDegPerSecondPitch);
    pitchDevider = constrain(maxDegPerSecondPitch / (pitchOutput + 0.000001), -15000,15000);
    pitchDirection = sgn(pitchDevider) * DIR_MOTOR_PITCH;

    rollOutput = ComputePID(sampleTimePID,rollInput,rollSetpoint, &rollErrorSum, &rollErrorOld,DMP_ROLL_Kp,DMP_ROLL_Ki,DMP_ROLL_Kd,maxDegPerSecondRoll);
    rollDevider = constrain(maxDegPerSecondRoll / (rollOutput + 0.000001), -15000,15000);
    rollDirection = sgn(rollDevider) * DIR_MOTOR_ROLL;

    mpuInterrupt = false;    
  }  
}
#endif

#ifdef USE_RAW_GYRO
void loop()
{
  if(freqCounter % CC_FACTOR == 0) // runs at ~1kHz
  {
    updateRawGyroData();

    // P-Part Pitch
    pitchDevider = constrain(maxDegPerSecondPitch / (gyroPitch * GYRO_PITCH_Kp + 0.000001), -15000,15000);
    pitchDirection = sgn(pitchDevider) * DIR_MOTOR_PITCH;

    // P-Part Roll
    rollDevider = constrain(maxDegPerSecondRoll / (gyroRoll * GYRO_ROLL_Kp + 0.000001), -15000,15000);
    rollDirection = sgn(rollDevider) * DIR_MOTOR_ROLL;
      
    Serial.println(gyroPitch);
  }
  if(freqCounter > CC_FACTOR * 4 == 0) // runs at ~250Hz
  {
    freqCounter=0;
  }
    

}
#endif


