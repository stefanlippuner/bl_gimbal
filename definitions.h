/*************************/
/* Definitions           */
/*************************/

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

// Do not Change for now, has negative impact on DMP performance
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_2000  // +-250,500,1000,2000 deg/s
#define MPU6050_DLPF_BW MPU6050_DLPF_BW_42 //0x07   //MPU6050_DLPF_BW_256 //256    // 5,10,20,42,98,188,256 Hz

// DMP Update frequency, 100Hz should be enough for repositioning
//#define DMP_100HZ
#define DMP_200HZ

// I2C Frequency
//#define I2C_SPEED 100000L     //100kHz normal mode
//#define I2C_SPEED 400000L   //400kHz fast mode
#define I2C_SPEED 800000L   //800kHz ultra fast mode



// Hardware Abstraction for Motor connectors, 
// DO NOT CHANGE UNLES YOU KNOW WHAT YOU ARE DOING !!!
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B


#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif


#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 32  
  #define maxPWM 255 // max value for PWM  
#endif
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 4  
  #define maxPWM 255 // max value for PWM
#endif 
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 8 
  #define maxPWM 255 // max value for PWM
#endif 


#define LEDPIN_PINMODE             pinMode (8, OUTPUT);
#define LEDPIN_SWITCH              digitalWrite(8,!bitRead(PORTB,0));
#define LEDPIN_OFF                 digitalWrite(8, LOW);
#define LEDPIN_ON                  digitalWrite(8, HIGH);












