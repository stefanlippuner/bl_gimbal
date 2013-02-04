/*************************/
/* Definitions           */
/*************************/
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

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 32  
  #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(1 * 256)) // Correction Factor for delay(), millis() and micros(), works with modified wiring.c
  #define maxPWM 255 // max value for PWM  
#endif
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 4  
  #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(16 * 256)) // Correction Factor for delay(), millis() and micros(), works with modified wiring.c
  #define maxPWM 255 // max value for PWM
#endif 
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 8 
  #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256)) // Correction Factor for delay(), millis() and micros(), works with modified wiring.c
  #define maxPWM 255 // max value for PWM
#endif 


#define LEDPIN_PINMODE             pinMode (8, OUTPUT);
#define LEDPIN_SWITCH              digitalWrite(8,!bitRead(PORTB,0));
#define LEDPIN_OFF                 digitalWrite(8, LOW);
#define LEDPIN_ON                  digitalWrite(8, HIGH);












