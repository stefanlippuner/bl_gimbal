// DEBUG Messages on/off
//#define DEBUG

/**********************************************/
/* Config PIDs                                */
/**********************************************/
// THOSE ARE DUMMY VALUES !!!
// PID Values for Pitch and Roll
#define CONST_PITCH_Kp 5.0   
#define CONST_PITCH_Ki 0.25    
#define CONST_PITCH_Kd 0.25 

#define CONST_ROLL_Kp 5.0
#define CONST_ROLL_Ki 0.25
#define CONST_ROLL_Kd 0.25

/**********************************************/
/* Configuration Timings and Motors           */
/**********************************************/
// The Three Values below give the maximum rotation velocity
// Example: PWM_4KHZ_PHASE, N_SIN = 255, 14 Poles
// Revolutions per second = 4000 / 255 / (14/2) = 2,24 
// NOT SURE IF THIS CALCULATION IS CORRECT !!!!
// = 806,4 °/s
// Or in "Servo-Speed" 0,074 sec for 60°


// Define Brushless PWM Mode, uncomment ONE setting
//#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
#define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM

// ATTENTION: Only works well for 4KHz for now.


// Number of sinus values for full 360 deg, should be devideable by 3
// 510 is max due to memory limitations
#define N_SIN 150

// Define Motor "Geometry" 
// -> 2 Poles = 1 revolution for one 2 PI Sinus
// -> 14 Poles = 1/7 revolution for one 2 PI Sinus
#define N_POLES_MOTOR_PITCH 14
#define N_POLES_MOTOR_ROLL 14


// Choose Direction for the Motors , Normal (1) or Reverse (-1)  
#define DIR_MOTOR_PITCH 1
#define DIR_MOTOR_ROLL 1

// Define Motor Numbers
#define MOTOR_PITCH 0
#define MOTOR_ROLL 1

// Hardware Abstraction for Motor connectors, 
// DO NOT CHANGE UNLES YOU KNOW WHAT YOU ARE DOING !!!
#define PWM_A_MOTOR2 OCR2A
#define PWM_B_MOTOR2 OCR1B
#define PWM_C_MOTOR2 OCR1A

#define PWM_A_MOTOR1 OCR0A
#define PWM_B_MOTOR1 OCR0B
#define PWM_C_MOTOR1 OCR2B


/**********************************************/
/* Configuration MPU6050                      */
/**********************************************/
// MPU Address Settings
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH

// DMP initialization time
// Recommendation: ~10-15 secs
#define DMP_INIT_TIME 15.0

// DMP Update frequency
//#define DMP_100HZ
#define DMP_200HZ

// I2C Frequency
//#define I2C_SPEED 100000L     //100kHz normal mode
//#define I2C_SPEED 400000L   //400kHz fast mode
#define I2C_SPEED 800000L   //800kHz ultra fast mode

