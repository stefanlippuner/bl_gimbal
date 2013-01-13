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

byte VersionStatus = 'B'; // B = Beta , N = Normal Release
byte VersionNumber = 035; 

/*************************/
/* Include Header Files  */
/*************************/
#include <avr/interrupt.h>
#include <Wire.h>

#include "config.h"
#include "definitions.h"
#include "MPU6050_6Axis_DMP.h"


/************************/
/* BL Controller        */
/************************/
uint8_t pwmSin[N_SIN];
float maxDegPerSecondPitch,maxDegPerSecondRoll;

// Current status of Magnetic Field for Motors
int currentStep[2]={0,0};
int pitchDevider,rollDevider;
int pitchDirection,rollDirection;

// Fast Interrupt driven loop for Motor control */
int deviderCountPitch = 0;
int deviderCountRoll = 0;

int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

unsigned long testcounter = 0;
ISR( TIMER1_OVF_vect )
{
  // Move pitch and roll Motor
  // 8-16 micros overall :-)
  deviderCountPitch++;
  if(deviderCountPitch  >= abs(pitchDevider))
  {
    moveMotor(MOTOR_PITCH, pitchDirection); 
    deviderCountPitch=0;
  }
    
  deviderCountRoll++;
  if(deviderCountRoll >= abs(rollDevider))
  {
    moveMotor(MOTOR_ROLL, rollDirection ); 
    deviderCountRoll=0;
  }
}

/*************************/
/* MPU 6050 DMP          */
/*************************/
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[16]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// Create MPU object
MPU6050 mpu;

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
//    *roll = sgn(*roll) * (180.0 - sgn(*roll) * *roll);
    *roll = sgn(*roll) * 180.0 - *roll;
    
    //atan2(2*(q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * 180/M_PI ; // DMP roll takes ~310 micros
}


/************************/
/* PID Controllers      */
/************************/
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


/************************/
/* Common Variables     */
/************************/
//general purpuse timer
unsigned long timer=0;   
unsigned long timer_old;




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
  Serial.println("Init MPU done ...");
  
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
  else
  {/* ERROR */}

  delay(500);

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
  
  // Init BL Controller
  initBlController();
  Serial.println("Init BL Control done ...");

  // Initialize timer
  timer=micros();
  LEDPIN_ON
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/

void loop() 
{
  if (!dmpReady) return; // Only Start the Programm if MPU are Ready ! 

  if (mpuInterrupt == true)
  {
    sampleTimePID = (micros()-timer)/CC_FACTOR/1000000.0; // in Seconds!
    timer = micros();

    updatePositionFromDmpFast(&pitchInput,&rollInput);
    
    pitchOutput = ComputePID(sampleTimePID,pitchInput,pitchSetpoint, &pitchErrorSum, &pitchErrorOld,CONST_PITCH_Kp,CONST_PITCH_Ki,CONST_PITCH_Kd,maxDegPerSecondPitch);
    pitchDevider = constrain(maxDegPerSecondPitch / (pitchOutput + 0.000001), -15000,15000);
    pitchDirection = sgn(pitchDevider) * DIR_MOTOR_PITCH;

    rollOutput = ComputePID(sampleTimePID,rollInput,rollSetpoint, &rollErrorSum, &rollErrorOld,CONST_ROLL_Kp,CONST_ROLL_Ki,CONST_ROLL_Kd,maxDegPerSecondRoll);
    rollDevider = constrain(maxDegPerSecondRoll / (rollOutput + 0.000001), -15000,15000);
    rollDirection = sgn(rollDevider) * DIR_MOTOR_ROLL;

    Serial.print(" "); Serial.print(pitchInput);
//    Serial.print(" "); Serial.print(pitchDevider);
   
   Serial.print(" "); Serial.print(rollInput);
//   Serial.print(" "); Serial.print(rollDevider);

    Serial.println();
    
    mpuInterrupt = false;    
  }
  
}
