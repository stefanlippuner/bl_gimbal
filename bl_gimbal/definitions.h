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



#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 31,25 // Correction Factor for delay(), millis() and micros() 
  #define maxPWM 255 // max value for PWM  
#endif
#ifdef PWM_16KHZ_PHASE
  #define CC_FACTOR 15.625 // Correction Factor for delay(), millis() and micros() 
  #define maxPWM 63 // max value for PWM
#endif 
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 3.90625 // Correction Factor for delay(), millis() and micros() 
  #define maxPWM 255 // max value for PWM
#endif 
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 7.8125 // Correction Factor for delay(), millis() and micros()
  #define maxPWM 255 // max value for PWM
#endif 


#define LEDPIN_PINMODE             pinMode (8, OUTPUT);
#define LEDPIN_SWITCH              digitalWrite(8,!bitRead(PORTB,0));
#define LEDPIN_OFF                 digitalWrite(8, LOW);
#define LEDPIN_ON                  digitalWrite(8, HIGH);












