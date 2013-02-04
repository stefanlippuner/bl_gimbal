// DEBUG Messages on/off
//#define DEBUG

/**********************************************/
/* Config PID Constants                       */
/* DMP is used for I and D                    */
/* Raw Gyro is used for P Part                */
/**********************************************/
#define GYRO_PITCH_Kp 0.7
#define DMP_PITCH_Ki 0.05    
#define DMP_PITCH_Kd 0.1

#define GYRO_ROLL_Kp 0.7
#define DMP_ROLL_Ki 0.05
#define DMP_ROLL_Kd 0.1


/**********************************************/
/* Low pass filter for Raw Gyro Signal        */
/**********************************************/
#define LP_FILTER_GYRO // uncomment if not required
#define LP_ALPHA_GYRO 0.5

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
// ATTENTION: Only works well for 4KHz for now.

//#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
#define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM


// Define Motor "Geometry" 
// -> 2 Poles = 1 revolution for one 2 PI Sinus
// -> 14 Poles = 1/7 revolution for one 2 PI Sinus
#define N_POLES_MOTOR_PITCH 14
#define N_POLES_MOTOR_ROLL 14


// Choose Direction for the Motors , Normal (1) or Reverse (-1)
// You can also just swap two cables of the motor-controller connection
#define DIR_MOTOR_PITCH 1
#define DIR_MOTOR_ROLL 1

// Define Motor Numbers
#define MOTOR_PITCH 0
#define MOTOR_ROLL 1

// Uncomment if you want to have Power/Torque Reduction in nearly stabilized conditions
// and power limitation to prevent overheating
// If uncommented then the MAX_POWER_MOTOR_X Value is used as a constant

//#define USE_POWER_REDUCTION_MOTOR_PITCH
//#define USE_POWER_REDUCTION_MOTOR_ROLL

// Devide max PWM-Power per Motor 
#define MIN_POWER_MOTOR_0 0.2                // 0-1
#define MAX_POWER_MOTOR_0 0.35                // 0-1
#define POWER_REDUCTION_SLOPE_MOTOR_0 50.0   // in deg/s

#define MIN_POWER_MOTOR_1 0.5
#define MAX_POWER_MOTOR_1 1.0
#define POWER_REDUCTION_SLOPE_MOTOR_1 50.0


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




