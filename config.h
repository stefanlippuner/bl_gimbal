// DEBUG Messages on/off
//#define DEBUG

/**********************************************/
/* Config PID Constants                       */
/* DMP is used for P,I and D                  */
/* Raw Gyro is used for P Part                */
/**********************************************/

#define GYRO_PITCH_Kp 3.4 // Can be increased in 
#define GYRO_PITCH_Ki 0.005    
#define GYRO_PITCH_Kd 0.02

#define GYRO_ROLL_Kp 2.8
#define GYRO_ROLL_Ki 0.005
#define GYRO_ROLL_Kd 0.15

#define ACC_WEIGHT 0.025



// Do not change for now
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_250  // +-250,500,1000,2000 deg/s
#define MPU6050_DLPF_BW MPU6050_DLPF_BW_256 //0x07   //MPU6050_DLPF_BW_256 //256    // 5,10,20,42,98,188,256 Hz



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
#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
//#define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM
//#define NO_PWM_LOOP

#define MOTORUPDATE_FREQ 8 //in kHz 1,2,4,8 for 32kHz, 1,2,4 for 4kHz


// Define Motor "Geometry" 
// -> 2 Poles = 1 revolution for one 2 PI Sinus
// -> 14 Poles = 1/7 revolution for one 2 PI Sinus
#define N_POLES_MOTOR_PITCH 14
#define N_POLES_MOTOR_ROLL 14


// Choose Direction for the Motors , Normal (1) or Reverse (-1)
// You can also just swap two cables of the motor-controller connection
#define DIR_MOTOR_PITCH -1
#define DIR_MOTOR_ROLL -1

// Define Motor Numbers
#define MOTOR_PITCH 0
#define MOTOR_ROLL 1

// Acts as a devider (!) for Motor PWM Power
#define MAX_POWER_MOTOR_0 2               // 1,2,3,4
#define MAX_POWER_MOTOR_1 1
#define maxPWM 180 // max value for PWM (0-255) 255=100% duty cycle



/**********************************************/
/* Configuration MPU6050                      */
/**********************************************/

// MPU Address Settings
#define MPU6050_ADDRESS_AD0_LOW     0x68 // default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // Drotek MPU breakout board
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH

// DMP initialization time
// Recommendation: ~10-15 secs
#define DMP_INIT_TIME 5.0




