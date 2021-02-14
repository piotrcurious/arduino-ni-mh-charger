
/*
 * NiMH dual cell smart charger
 * using the Arduino328p
 * 
 * GameInstance.com + chaoscrawler@protonmail.com (2021) 
 * 2017
 * 
 * chaoscrawler porting notes:
 * code is almost portable, to port to something other than arduino uno : 
 * -comment out #define USE_CUSTOM_PWM
 * -sort out voltage reference in ReadMultiDecimated. Currently there is boolean flag enabling 1.1V reference to read Vbat (only)
 *  no idea what board You will use. Perhaps char variable and switch should be added, or just using analogRead directly without decimation if board has decent ADC.
 * -comment or uncomment features. After commenting almost everything out, You can fit the code on ATMEGA8 , still having two ports and LCD (but no serial debug for graph)
 *  You can also go other way, porting to ATMEGA2560 , due or blackpill allows You to use four charging ports (but You need to add missing code and sort out display issues)
 *  for ATMEGA8 note that custom PWM uses _less_ code space than plain analogWrite, no idea why. 
 * -#pragma GCC optimize does not really work for some reason. You can copy the options to shave ~100bytes or bit more, esp. -ftree-vectorize -funroll-loops and -fsched-pressure. 
 */

//#pragma GCC optimize ("Os,mcall-prologues,fno-split-wide-types,finline-limit=3,ffast-math")
//#pragma GCC optimize ("Os,mcall-prologues,fno-split-wide-types,finline-limit=3,ffast-math,fwhole-program")

//#pragma GCC optimize ("O3")
//#pragma GCC optimize ("Os,fira-loop-pressure")
//#pragma GCC optimize ("Os") // -Os {compiler.warning_flags} -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mc
      //all-prologues -fsplit-wide-types -finline-limit=1 -ffast-math -ffunction-sections -fwhole-program -flto -fmerge-all-constants -fmodulo-sched -fmodulo
      //-sched-allow-regmoves -fdeclone-ctor-dtor -fira-loop-pressure -fsched-pressure -fsched-spec-load -Wl,--gc-sections -Wl,--relax -ftree-vectorize -funroll-loops 

#define USE_GLOBAL_TIMEOUT // enabling will enable global timeout function , scalable by set current knob . default is for 3000mAh at 1A, about 3h. 
                            // the smaller current, the longer timeout will be, f.e. 0.5A will wait for 6h etc. 
                            // note startup ramp prolongs the time. 
//#define USE_TONE // enabling will produce audio effects on "speaker pin" defined below (default pin2)
//#define USE_NEWTONE // enabling will use new tone library. You have to disable tone first!. note You have to re-config PWM after each beep. Still shaves 1k of bloat. 
#define USE_DUMBTONE // enabling this will use "dumb" tone - based on crude delays . tone frequency might vary if you change chip/speed!
#define SERIAL_DEBUG_EVENTS  // enabling will cause all events like battery insertion or charge finish being emit to serial port. verbose. 
#define SERIAL_DEBUG_GRAPH // this will produce all data needed for gnuplot for nice charge graph on serial output
#define SERIAL_GENERAL // disable only if all serial debug defines are undefined. 
#define LED_DEBUG // enabling produces code blinking internal led for amount of time spent in servicing channel0 . time evaluation purposes. 
#define USE_LED_FOR_MESSAGES // signal key events using built-in LED.
                             // so far implemented :
                             // -battery insertion detection/testing  
#define KNOB_HIGH_PRECISION // if disabled, set_current will be calculated crude way, and not displayed from setup(). beware, it can set too high current! look at main loop. 
#define USE_NTC_SERIES_SKEW // if enabled, You can fine tune series resistors of temperature sensors.
                            // it does not help in anything , but if You have spare memory - why not. 
                            // Charger will then work as nice and precise thermometer when not charging ;)
                            // this also adds display of temperature on welcome screen.
                            // skew values are signed 16bit.
                            // uses 892bytes of PROGMEM and 8 bytes of RAM
                            
#define USE_NTC_COOLING     // if You have spare progmem bytes, You can use thermistor cooling mode. 
                            // it works by shunting thermistor to ground , preventing any current to flow thru it when it is not read. 
                            // this reduces auto-heating by thermistor being part of resistor divider. 
                            // if You plan to use reverse topology (series resistance to ground) , invert all the code related. 
                            // uses 360 bytes of PROGMEM and 0 bytes of RAM                           
//#define DISCHARGER // enabling it will produce broken code, most is commented out hard way and not updated --fixme!
#define CURRENT_DIVISOR // enabling it will produce code reducing current automatically if very high internal resistance cell is detected.
//#define LOW_SRAM_VERSION // if ram below 1024 bytes, calculate averages once every 2 min, and use less memory for Tslope
#define LCD_VERBOSE //-550B enabling it will display more messages on LCD . otherwise just bare minimum is included . 
#define USE_LCD //-1900B decide wheter to use LCD at all. 

#define USE_FAN_AMBIENT // use simple code to control one , 'ambient' fan , turning on when ambient temp swings too much.
                        // thus equalizing temperature of the cells.  

//#define USE_KALMAN_FOR_TEMPERATURE // wheter use kalman filter for temperature reading. allows lowering deltaT detection 
                                      // uses 256 bytes of PROGMEM and 32 bytes of RAM
  // !use either kalman or dynamic kalman, or none. You cannot use "both". 
#define USE_DYNAMIC_KALMAN_FOR_TEMPERATURE // You can comment it out aswell. Dynamic version applies Kalman filters for each sensor, plus updates covariance settings based on 
                                          // measured ambient temperature swings and applies corrected Kalman for temperature delta .
                                          // this means three filter instances plus extra glue code per each charging slot object. 
                                          // if all Kalman filtering is commented out, no filtering at all is applied, but You can increase temperature oversampling for averaging
                                          // without any extra memory penalty - it only costs You time - each minute oversampling takes over 1sec, inside statistics gathering loop.  
                                          // uses  1602 bytes PROGMEM and 144 bytes of RAM
                                          // it is expensive, so remember to benefit from it - it allows You to lower deltaT detection threshold by at least 1C

#define USE_CUSTOM_PWM  // if You want to use PWM larger than standard 8bit. standard PWM adds 86 bytes for some reason... perhaps analogWrite() ?
                          // but code is almost fully portable if You use standard PWM (ST boards, esp32, arduino due, etc)
#define USE_EXTRA_SMOOTH_PWM // if defined, PWM regulation occurs twice per entry, 
                             // first a rough read and regulation step is performed, 
                             // then high precission measurement is taken and regulation is corrected. 
                             // if defined, 270 bytes PROGMEM is used, 0 bytes of RAM
                             

                          
#ifdef USE_CUSTOM_PWM
static const uint16_t PWM_MAX_VALUE = 1024; //define max value of PWM 
  // 512 max counter                , 32Khz frequency // fixme - those values are certainly wrong...
  // 1024  as the max counter value, 16Khz frequency
  // 16385 as the max counter value, 1Khz frequency
  // 65535 as the max counter value, 250Hz frequency
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
static const uint16_t PWM_MAX_VALUE = 256 ;
#endif  USE_CUSTOM_PWM

#ifdef USE_KALMAN_FOR_TEMPERATURE
static const float KALMAN_Rk_FOR_TEMPERATURE = 0.4;     // process covariance noise        
static const float KALMAN_Qk_FOR_TEMPERATURE = 0.2;     // observation covariance noise
#endif USE_KALMAN_FOR_TEMPERATURE
                         
#ifdef LOW_SRAM_VERSION
static const uint8_t TEMPERATURE_SLOPE_SEQUENCE_LENGTH_GLOBAL = 60 ;
static const uint32_t MINUTE_TS_REAL = 120000;   
#endif LOW_SRAM_VERSION
#ifndef LOW_SRAM_VERSION
static const uint8_t TEMPERATURE_SLOPE_SEQUENCE_LENGTH_GLOBAL = 140 ; 
static const uint16_t MINUTE_TS_REAL = 60000;   
#endif LOW_SRAM_VERSION

//#include "NTCThermistor.h" // no longer in use. inlined as function. 
#ifdef USE_TONE
#include <Tone.h>             // used for audible beeps
Tone tone0 ;
#endif 

#ifdef USE_NEWTONE
#include <NewTone.h>             // used for audible beeps
#endif 

#ifdef USE_LCD
/*
 
The circuit:
 * LCD RS pin to digital pin      3
 * LCD Enable pin to digital pin  4
 * LCD D4 pin to digital pin      5
 * LCD D5 pin to digital pin      6
 * LCD D6 pin to digital pin      7
 * LCD D7 pin to digital pin      8
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3, 4, 5, 6, 7, 8);
#endif USE_LCD

#ifdef USE_GLOBAL_TIMEOUT
static const uint32_t CHARGE_TIMEOUT_VALUE = 14400000 ;//4*60*60*1000; //4h*60min*60s*1000ms  - 3h is max  , assuming 3000mah cell 
#endif USE_GLOBAL_TIMEOUT
static const byte V_BAT0 = A1;
static const byte V_BAT1 = A0;
static const byte I_BAT0 = A6;
static const byte I_BAT1 = A7;
static const byte NTC0 = A3;
static const byte NTC1 = A2;
#ifdef USE_FAN_AMBIENT
static const byte FAN_AMBIENT = 3 ; // fan used to equalize temperature
static uint8_t fan_ambient_state ; // state of the fan , updated in main loop, set by other routines
static const uint32_t STARTUP_FAN_TIMEOUT = 600000 ; // 10 minutes, fan cooling time after inserting new battery to equalize it's temperature. 
#endif USE_FAN_AMBIENT 

#ifdef USE_NTC_SERIES_SKEW  
//#define KALMAN_AUTOSKEW // if defined, only routine performing autoskew is executed instead of everything else
                        // it allows filling in skew values. 
                        // it prints T delta vs ambient for each sensor, plus slowly converges the skew. 
#define KALMAN_AUTOSKEW_FREEZE // if set, skew values are not changed - usefull for testing found values, and just testing kalman filter parameters/fine tuning  .  

//static int16_t NTC_skew[2]= {1450,950}; // ntc series resistor skew, by ID-1
#ifdef KALMAN_AUTOSKEW // if autoskew is on, use floats for skew for easier convergence. ntc thermistor function switches to floats via same define.
//static float NTC_skew[2]= {1234,690}; // ntc series resistor skew, by ID-1
//static float NTC_skew[2]= {1250,720}; // ntc series resistor skew, by ID-1
//static float NTC_skew[2]= {1266,674}; // ntc series resistor skew, by ID-1
static float NTC_skew[2]= {1150,453}; // ntc series resistor skew, by ID-1 // NOTE THIS VARIABLE IS ONLY USED DURING AUTOSKEW PHASE. COPY YOUR FOUND VALUSE TO STATIC CONST BELOW!
//static float NTC_skew[2]= {1251,869}; // ntc series resistor skew, by ID-1

#endif KALMAN_AUTOSKEW
#ifndef KALMAN_AUTOSKEW // if autoskew is off, resistor values are stored permanently in progmem. 
static const int16_t NTC_skew[2]= {1150,453}; // ntc series resistor skew, by ID-1 // this is autoskew value normally used. put Your final values here. 
#endif KALMAN_AUTOSKEW

#endif USE_NTC_SERIES_SKEW
static const uint8_t  NTC_ambient       = A4;
#ifdef USE_NTC_SERIES_SKEW
static const int16_t NTC_ambient_skew = 0 ; // ntc_ambient series resistor skew (global)
#endif USE_NTC_SERIES_SKEW

static const uint8_t  CHARGE_RATE_KNOB  = A5;  

//static const uint32_t NTC_SERIES_RESISTOR = 3*9660; 
inline static const uint16_t NTC_SERIES_RESISTOR = 3*9660;
inline static const uint32_t NTC_NOMINAL_RESISTANCE = 100000; 
inline static const uint16_t NTC_BETA_COEFFICIENT = 3950;
inline static const uint8_t  NTC_NOMINAL_TEMPERATURE = 25; 
//      ntc.Config(100000, 25, 3950, NTC_SERIES_RESISTOR, 65536); //9.6k

static const byte CH_PWM0 = 9;
static const byte CH_PWM1 = 10;

#ifdef USE_DISCHARGER
//static const byte DSCH_0 = 12;
//static const byte DSCH_1 = 11;
#endif USE_DISCHARGER 

#ifdef USE_TONE
static const byte speaker_pin = 2 ;
#endif USE_TONE
#ifdef USE_NEWTONE
static const uint8_t speaker_pin = 2 ;
#endif USE_NEWTONE
#ifdef USE_DUMBTONE
static const uint8_t speaker_pin = 2 ;
#endif USE_DUMBTONE

#ifdef LED_DEBUG
static const uint8_t  LED_pin           = 13; 
#endif LED_DEBUG
float set_current ;       // current set by user by knob 

static const float V_BAT_MIN = 0.5; // volts
static const float V_BAT_CHARGE = 1.0; // volts
static const float V_BAT_MAX_NIMH = 1.475; // volts
static const float V_BAT_MAX = 2; // volts
//static const float VOUT_MAX = 5.0; // volts
static const float VCC = 5.14; //4.72; //5.00; // volts
//static const float VREF = 1.138 ; // for precise battery measurement // 1.140
//static const float VREF_Vbat = 2.24 ;// voltage divider of 954/1150 ohms
static const float VREF_Vbat = 2.05 ;// voltage divider of 954/1150 ohms
//static const float SHUNT_RESISTOR = 1.8; // ohms
static const float SHUNT_RESISTOR = 1.3 ; // ohms , 3V max / 1.7ohm = 1.76A max current

static const float I_HIGH_CURRENT_CHARGING = 0.6; // amps, defines max current possible
static const float CURRENT_INCREASE_RATE = 200; //divide set_current by this to get ramp rate// amps, defines A/min increase rate of current for ramp-based charging
#ifndef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
static const uint8_t MAX_CUMULATIVE_TEMPERATURE_INCREASE = 3; // if we detect delta T of such magnitude across sliding window, end charge
static const uint8_t MAX_STATIC_TEMPERATURE_DELTA = 7 ; // if battery temperature delta over ambient T exceeds this amount , end charge
static const uint16_t STABILIZATION_PERIOD = 5000; 
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE // Thanks to Dynamic Kalman it is possible to detect smaller temperature deltas wihout false positives
static const uint8_t MAX_CUMULATIVE_TEMPERATURE_INCREASE = 2; // if we detect delta T of such magnitude across sliding window, end charge
static const uint8_t MAX_STATIC_TEMPERATURE_DELTA = 3 ; // if battery temperature delta over ambient T exceeds this amount , end charge
static const uint16_t STABILIZATION_PERIOD = 30000; 
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE

#ifdef USE_CUSTOM_PWM
void ConfigPWM() {
  // fast PWM non-inverting and no prescaling
  // on pins 9 and 10
  DDRB |= _BV(PB1) | _BV(PB2);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = PWM_MAX_VALUE-1;
}

#endif USE_CUSTOM_PWM

#ifdef USE_CUSTOM_PWM
void SetPWM(uint8_t pin, uint16_t value) {
  // sets the PWM level
  switch (pin) {
    //
    case 9: 
      // 
      OCR1A = value;
      break;
      
    case 10:
      // 
      OCR1B = value;
      break;
  }
}
#endif USE_CUSTOM_PWM


class BatteryCharger {

  public:
  
    /// default constructor
    BatteryCharger() : 
      state(0) {
      // 
//      ntc.Config(100000, 25, 3950, 100250, 65536);
//      ntc.Config(100000, 25, 3950, NTC_SERIES_RESISTOR, 65536); //9.6k // no longer used, defined as global constants . all thermistors had to be alike anyway. 

    }
    /// destructor
    virtual ~BatteryCharger() {
      // 
    }

    /// configures the object
    void Config(
      unsigned char vbat_pin, 
      unsigned char ibat_pin, 
      unsigned char tbat_pin, 
      unsigned char pwm_pin, 
//      unsigned char dschrg_pin, 
      unsigned char id) {
      // 
      pin_vbat = vbat_pin;
      pin_ibat = ibat_pin;
      pin_tbat = tbat_pin;
      pin_pwm = pwm_pin;
//    pin_dschrg = dschrg_pin;
      iid = id;
    }
    /// executes the automata
    void Execute() {
      // 
      switch (state) {
        // 
        case 0: {
          // initial state
//          value_vbat = 0;
          capacity_in = 0;
#ifdef DISCHARGER
          capacity_out = 0;
#endif DISCHARGER
#ifdef SERIAL_DEBUG_EVENTS          
//          Serial.print(F("INIT "));
//          Serial.println(iid);
#endif SERIAL_DEBUG_EVENTS          
//          delay(2000);
          
          state = 1;
          break;
        }

        // BATTERY DETECTION AND EVALUATION
        {
          
          case 1: {
            // battery presence test          
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);
//            voltage_vbat = GetVoltage(value_vbat, 65536,VREF_Vbat);
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat,10);
//            voltage_vbat = GetVoltage(value_vbat,1024,VREF_Vbat);
#ifdef USE_CUSTOM_PWM
            SetCharger(PWM_MAX_VALUE/16); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,PWM_MAX_VALUE/16);
#endif USE_CUSTOM_PWM            

            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,10,true),1024,VREF_Vbat);
//            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,16,true),65536,VREF_Vbat);
//            SetCharger(0);
#ifdef USE_CUSTOM_PWM
            SetCharger(0); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,0);
#endif USE_CUSTOM_PWM            

//only for debug
#ifdef USE_NTC_SERIES_SKEW
#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef KALMAN_AUTOSKEW
#ifdef USE_LCD
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING
            temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10,false),1024,NTC_skew[iid-1]),1);
            temperature_ambient = filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10,false),1024,NTC_ambient_skew),2);
            float deltaT = temperature-temperature_ambient; 
/*
            if (fabs (deltaT) < 0.05) {
            temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,14,false),16384,NTC_skew[iid-1]),1);
            temperature_ambient = filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,14,false),16384,NTC_ambient_skew),2);
            }
            deltaT = temperature-temperature_ambient; 
            if (fabs (deltaT) < 0.005) {
            temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,15,false),32768,NTC_skew[iid-1]),1);
            temperature_ambient = filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,15,false),32768,NTC_ambient_skew),2);
            deltaT = temperature-temperature_ambient;
            }
*/
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 

            lcd.setCursor(0,iid-1);
//            lcd.print(voltage_vbat,4);
//            lcd.print(F("V"));
            lcd.print((temperature-temperature_ambient),6);
            lcd.print(F(" "));
//            if (iid==1) {lcd.print(temperature_ambient,4);}
            lcd.print((float)NTC_skew[iid-1],2);          
//            lcd.print(NTC_skew[iid-1],2);          
             lcd.print(F(" "));
#ifndef KALMAN_AUTOSKEW_FREEZE             
//            if (temperature > filter_update(temperature_ambient,3)) {NTC_skew[iid-1]+=fabs(deltaT);}
//            if (temperature < filter_update(temperature_ambient,3)) {NTC_skew[iid-1]-=fabs(deltaT);}

            if (temperature > filter_update(temperature_ambient,2)) {NTC_skew[iid-1]+=fabs(deltaT*1000);}
            if (temperature < filter_update(temperature_ambient,2)) {NTC_skew[iid-1]-=fabs(deltaT*1000);}

#endif KALMAN_AUTOSKEW_FREEZE            
//            Serial.print(temperature,7);
            Serial.print(deltaT*1000,7);

            Serial.print(F(" "));
//            Serial.print(iid);
//            Serial.print(F(" "));
            if (iid == 2) {
             Serial.print((temperature_ambient-21)*100.0,7);
             Serial.println(F(" "));
            }
//            delay(100);
            break;
#endif USE_LCD
#endif KALMAN_AUTOSKEW
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#endif USE_NTC_SERIES_SKEW

            if (voltage_vbat > V_BAT_MAX) {
              // device is powered and the ouput voltage got HIGH 
              // slowly through the high, yet finite Rds of closed MOSFET
              break;
            }
            // some battery was connected
//            if (voltage_vbat >= V_BAT_MAX_NIMH) {
              // alkaline battery detected or a fresh-from-the-charger NiMH
              // anyway, not needing discharging or recharging

//              Serial.println("DEVICE " + (String)iid + ": INVALID BATTERY DETECTED ");
//#ifdef USE_LCD
//              lcd.setCursor(0,iid-1);
//              lcd.print("Invalid Battery");
//#endif USE_LCD
//              state = 100;
//              tone0.play(120,500);
//              break; // just exit, like too low voltage, as this may happen when nothing is connected 
//            }
            // 
            if (voltage_vbat > V_BAT_MIN) {
//              SetCharger(10) ;//fixme - use some constant?
#ifdef USE_CUSTOM_PWM
            SetCharger(PWM_MAX_VALUE/16); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,PWM_MAX_VALUE/16);
#endif USE_CUSTOM_PWM            
              
              voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,10,true),1024,VREF_Vbat);
//              SetCharger(0); // disable charger
#ifdef USE_CUSTOM_PWM
            SetCharger(0);
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,0);
#endif USE_CUSTOM_PWM            

              if (voltage_vbat > V_BAT_MAX ) {
                break; // air = break. 
              }// otherwise valid battery
              
              // some decent voltage available at the battery slot pins

              state = 2;
            }
            break;
          }
  
          case 2: {
            // waiting for temperature and voltage to stabilize
            end_ts = millis() + STABILIZATION_PERIOD;
#ifdef USE_FAN_AMBIENT
              fan_ambient_state = 255; 
#endif USE_FAN_AMBIENT

            state = 3;
            break;

          }
  
          case 3: {
            // waiting for temperature and voltage to stabilize
#ifdef USE_LED_FOR_MESSAGES
  digitalWrite(LED_pin,HIGH); //turn on the LED to measure time spent in routine  
#endif USE_LED_FOR_MESSAGES

#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
              temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024,NTC_skew[iid-1]),1) - filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024,NTC_ambient_skew),2);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
              temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024),1) - filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024),2);
#endif USE_NTC_SERIES_SKEW

#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
            // temperature readout is not used, but it updates kalman filter initial state. 
            
            if (millis() > end_ts/2) {
              // thermometers did settle, so start updating kalman of delta 
              temperature = filter_update(temperature,3); 
            }

            if (millis() > end_ts) {
              // time to end the test
              state = 4;
            }

            // battery presence test
//            SetCharger(10); // some constant? //fixme
#ifdef USE_CUSTOM_PWM
            SetCharger(PWM_MAX_VALUE/16); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,PWM_MAX_VALUE/16);
#endif USE_CUSTOM_PWM            
            delay(30); 
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);
//            voltage_vbat = GetVoltage(value_vbat, 65536,VREF_Vbat);
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat,10);
//            voltage_vbat = GetVoltage(value_vbat, 1024,VREF_Vbat);
            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,10,true), 1024,VREF_Vbat);
//            SetCharger(0);
#ifdef USE_CUSTOM_PWM
            SetCharger(0); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,0);
#endif USE_CUSTOM_PWM            
            if (voltage_vbat > V_BAT_MAX) {
              // device is powered and the ouput voltage got HIGH 
              // slowly through the high, yet finite Rds of closed MOSFET
            state = 1; 
              break;
            }
            // some battery was connected
            // this check is only making trouble when nothing is connected. 
/*
            if (voltage_vbat >= V_BAT_MAX_NIMH) {
              // alkaline battery detected or a fresh-from-the-charger NiMH
              // anyway, not needing discharging or recharging
#ifdef SERIAL_DEBUG_EVENTS
              Serial.print(F("DEVICE ") );
              Serial.print(iid);
              Serial.println(F(": INVALID BATTERY "));
#endif SERIAL_DEBUG_EVENTS
#ifdef USE_LCD
#ifdef LCD_VERBOSE              
              lcd.setCursor(0,iid-1);
              lcd.print(F("Invalid Battery"));
#endif LCD_VERBOSE
#endif USE_LCD 
              state = 100;
#ifdef USE_TONE
              tone0.play(120,500);
#endif USE_TONE
#ifdef USE_NEWTONE
              NewTone(speaker_pin,120,500);
              delay(500);
              noNewTone();
#ifdef USE_CUSTOM_PWM
              ConfigPWM();
#endif USE_CUSTOM_PWM              
#endif USE_NEWTONE

              break;
            }
*/
#ifdef USE_LED_FOR_MESSAGES
  digitalWrite(LED_pin,LOW); // turn off the LED.   
#endif USE_LED_FOR_MESSAGES            
            break;
          }
  
          case 4: {
            // probing for open-circuit voltage and temperature
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);
//            voltage_vbat = GetVoltage(value_vbat, 65536,VREF_Vbat);

//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);
              voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,10,true), 1024,VREF_Vbat);
#ifdef SERIAL_DEBUG_EVENTS  
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.println (F(": NiMH DETECTED "));
#endif SERIAL_DEBUG_EVENTS  
#ifdef USE_TONE
              tone0.play(500,100);
              delay(200);
              tone0.play(800,100);
              delay(200);
              tone0.play(1000,100);
#endif USE_TONE               
#ifdef USE_NEWTONE
              NewTone(speaker_pin,500,100);
              delay(200);
              NewTone(speaker_pin,800,100);
              delay(200);
              NewTone(speaker_pin,1000,100);
              delay(100);
              noNewTone();
#ifdef USE_CUSTOM_PWM              
              ConfigPWM();
#endif USE_CUSTOM_PWM              
#endif USE_NEWTONE               
//           voltage_vbat_noload = voltage_vbat;
#ifdef SERIAL_DEBUG_EVENTS 
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
            temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10,false),1024,NTC_skew[iid-1]);
            temperature_ambient = ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10,false),1024,NTC_ambient_skew); 
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
            temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10,false),1024);
            temperature_ambient = ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10,false),1024); 
#endif USE_NTC_SERIES_SKEW
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 
            Serial.print(F("DEVICE "));
            Serial.print(iid);
            Serial.print(F(": Vbat: "));
            Serial.print(voltage_vbat, 4);
            Serial.print(F(" V; Tbat: "));
            Serial.print(temperature, 4);
            Serial.print(F("C "));
            Serial.print(F("; Tamb: "));
            Serial.print(temperature_ambient,4);
            Serial.print(F("C "));
#endif SERIAL_DEBUG_EVENTS             
#ifdef USE_LCD
#ifdef LCD_VERBOSE
#ifndef SERIAL_DEBUG_EVENTS
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
            temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10,false),1024,NTC_skew[iid-1]);
            temperature_ambient = ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10,false),1024,NTC_ambient_skew); 
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
            temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10,false),1024);
            temperature_ambient = ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10,false),1024); 
#endif USE_NTC_SERIES_SKEW
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 

#endif SERIAL_DEBUG_EVENTS
            lcd.setCursor(0,iid-1);
            lcd.print(voltage_vbat,4);
            lcd.print(F("V "));
            lcd.print(temperature_ambient,0);
            lcd.print(F("C-")); 
            lcd.print(temperature,0);
            lcd.print(F("C ")); 
#endif LCD_VERBOSE
#endif USE_LCD
            current_target = set_current ;   // set current target 
            current_current = current_target/4; // set current current to low value 
            state = 40 ;  // straight to "high current charging" code 
            break;
          }
  
//          case 5: {
//            // probing for closed-circuit voltage
//            SetDischarger(true);
//            end_ts = millis() + 2000;
//            state = 6;
//            break;
//          }
  
//          case 6: {
//            // waiting for discharge voltage and current to stabilize
//            if (millis() > end_ts) {
//              // time to end the test
//              value_ibat = ReadMultiDecimated(pin_ibat);
//              voltage_ibat = GetVoltage(value_ibat, 65536);
//              SetDischarger(false);
//              state = 7;
//            }
//            break;
//          }
  
//          case 7: {
//            // DEBUG
//            // state = 8; // DEBUG
//            if (voltage_vbat > V_BAT_CHARGE + 0.01) {
//              // discharging
//              state = 20;
// charge anyway, because discharging is stupid. 
//              state = 8; 
//            } else {
//              // charging
//              state = 8;
//            }
//            break;
//          }

//          case 8: {
//            // 
//            Serial.println("DEVICE " + (String)iid + ": " + (String)voltage_vbat + " V; " + (String)temperature + " C");
//              state = 40;
//            break;
//          }
//        }

//         DISCHARGING
//        {
  /*        
          case 20: {
            // discharging the battery
            SetDischarger(true);
            
            start_ts = millis();
            end_ts = start_ts + 18000000; // 5 hours
            ts = start_ts + 5000; // 5 seconds
            
            delay(500);
  
            Serial.println("DEVICE " + (String)iid + ": DISCHARGER ON ");
            
            state = 21;
            break;
          }
  
          case 21: {
            // battery discharge current test: PROBING
            value_vbat = ReadMultiDecimated(pin_vbat);
            voltage_vbat = GetVoltage(value_vbat, 65536);
            
            if (voltage_vbat <= V_BAT_CHARGE) {
              // steep voltage drop
              
              Serial.println("DEVICE " + (String)iid + ": VOLTAGE " + (String)voltage_vbat + " BELOW THRESHOLD");
              
              state = 22;
              break;
            }

            now_ts = millis();
            if (now_ts > end_ts) {
              // time to end the discharging
              
              Serial.println("DEVICE " + (String)iid + ": DISCHARGE TIMEOUT");
              
              state = 22;
              break;
            }

            if (now_ts > ts) {
              // 
              //ts = now_ts + 5000;// - (now_ts - ts);
              value_ibat = ReadMultiDecimated(pin_ibat);
              voltage_ibat = GetVoltage(value_ibat, 65536);
              current = (voltage_vbat - voltage_ibat) / SHUNT_RESISTOR;
              capacity_out += current * 5.0 / 3.6;
              ts += 5000;
  
              temperature = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));
              if ((temperature < 10) || (temperature > 40)) {
                // temperature error

                Serial.println("DEVICE " + (String)iid + ": ABNORMAL DISCHARGE TEMPERATURE");
                
                state = 23;
                break;
              }

              if (0 && (current < 0.01)) {
                // abnormally small discharge current

                Serial.println("DEVICE " + (String)iid + ": ABNORMAL DISCHARGE CURRENT: " + (String)current);
                
                state = 22;
                break;
              }
              
              double Rmosfet = voltage_ibat / current;
              double Rint = voltage_vbat_noload * (SHUNT_RESISTOR + Rmosfet) / voltage_vbat - SHUNT_RESISTOR - Rmosfet;

              Serial.print("DEVICE " + (String)iid + ": ");
              Serial.print(now_ts);
              Serial.print(" Vbat = ");
              Serial.print(voltage_vbat, 6);
              Serial.print(" V; Vshunt = ");
              Serial.print(voltage_vbat - voltage_ibat, 6);
              Serial.print(" V; Ishunt = ");
              Serial.print(current, 6);
              Serial.print(" A; Ploss = ");
              Serial.print(voltage_vbat * current, 6);
              Serial.print(" W; Rmosfet = ");
              Serial.print(Rmosfet, 6);
              Serial.print(" Ohm; Rbat = ");
              Serial.print(Rint, 6);
              Serial.print(" Ohm; Tbat = ");
              Serial.print(temperature, 6);
              Serial.print(" C; Cout = ");
              Serial.print(capacity_out, 6);
              Serial.println(" mAh");
              
            }
            break;
          }
  
          case 22: {
            // battery discharge current test: END
            SetDischarger(false);
            
            Serial.println("DEVICE " + (String)iid + ": DISCHARGER OFF ");
  
            delay(1000);
            state = 5;
            break;
          }
  
          case 23: {
            // battery discharge current test: END
            SetDischarger(false);
            
            Serial.println("DEVICE " + (String)iid + ": DISCHARGER OFF - ERROR");
  
            state = 24;
            break;
          }
  
          case 24: {
            // battery discharge error
            
            break;
          }
        }
*/

        }

        // CHARGING 
        {
          
          case 40: {
            // turning on the charger at minimum PWM level
//            SetDischarger(false);
    //        delay(1000);
            temperature_slope = 0;
            temperature_avg = 0;
            voltage_vbat_avg = 0; 
            level_pwm = 1; 
#ifdef CURRENT_DIVISOR // initalize current divisor.
            set_current_divisor = 1; 
#endif CURRENT_DIVISOR // initalize current divisor.
            
//            temperature_last = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));
#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
              temperature_last = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024,NTC_skew[iid-1]),1) - filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024,NTC_ambient_skew),2);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
              temperature_last = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024),1) - filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024),2);
#endif USE_NTC_SERIES_SKEW

#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 


#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifndef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
            temperature_last = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024,NTC_skew[iid-1]) - ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024,NTC_ambient_skew);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
            temperature_last = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024) - ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024);
#endif USE_NTC_SERIES_SKEW
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 

#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE

//initalize temperature_last so we will not get initial high slope. 
            for (byte i = 0; i < TEMPERATURE_SLOPE_SEQUENCE_LENGTH; i ++) {
              // 
              temperature_slope_sequence[i] = 0;
            }
            voltage_max = 0;
            voltage_drop = 0;
 //           SetCharger(level_pwm == 0 ? 2 : level_pwm);
//            SetCharger(1); // no need to. 
//            level_pwm_increment = 16;
            current_avg = 0;

            start_ts = millis();
#ifdef USE_GLOBAL_TIMEOUT            
            end_ts = start_ts + CHARGE_TIMEOUT_VALUE / current_target ; // max time/charge current 
#endif USE_GLOBAL_TIMEOUT
            ts = start_ts + 5000; // 5 seconds
            minute_ts = start_ts + MINUTE_TS_REAL; // 1 minute

#ifdef SERIAL_DEBUG_EVENTS
            Serial.print(F("DEVICE "));
            Serial.print(iid);
            Serial.print(F(": CURRENT TARGET : "));
            Serial.print(current_target,3);
#endif SERIAL_DEBUG_EVENTS            
#ifdef USE_GLOBAL_TIMEOUT 
#ifdef SERIAL_DEBUG_EVENTS           
            Serial.print(F("A : CHARGE TIMEOUT : "));
            Serial.print((float)end_ts/3600000,2);
            Serial.println(F(" h"));
#endif SERIAL_DEBUG_EVENTS            
#endif USE_GLOBAL_TIMEOUT
            state = 41;
            break;
          }

          case 41: {
            // 
//            value_vbat = ReadMultiDecimated(pin_vbat);
//            voltage_vbat = GetVoltage(value_vbat, 65536); // 5V vref

//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);  
//            voltage_vbat = GetVoltage(value_vbat, 65536,VREF_Vbat);
//too slow
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat,11);
//            voltage_vbat = GetVoltage(value_vbat, 2048,VREF_Vbat);

//            value_vbat = ReadMultiDecimated_1_1(pin_vbat,11);

#ifndef USE_EXTRA_SMOOTH_PWM // high resolution read , because it happens only once. 
            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,12,true), 4096,VREF_Vbat);
            voltage_ibat = GetVoltage(ReadMultiDecimated(pin_ibat,12), 4096);
            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;
            current_avg = (11 * current_avg + current) / 12;
#endif USE_EXTRA_SMOOTH_PWM // try to do extra regulation step. 
#ifdef USE_EXTRA_SMOOTH_PWM // low resolution read , because it gives only rough estimate. . 
            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,10,true), 1024,VREF_Vbat);
            voltage_ibat = GetVoltage(ReadMultiDecimated(pin_ibat,10), 1024);
            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;
//            current_avg = (11 * current_avg + current) / 12; do not include rough estimate in stats, it will be corrected and measured in right place
#endif USE_EXTRA_SMOOTH_PWM // try to do extra regulation step. 

#ifdef USE_EXTRA_SMOOTH_PWM // try to do extra regulation step. 

#ifdef CURRENT_DIVISOR
//          if ( (current - current_current/set_current_divisor) < 0.01 ){
            if ( current < current_current/set_current_divisor ){
#endif CURRENT_DIVISOR
#ifndef CURRENT_DIVISOR
//          if ( (current - current_current) < 0.01){
            if ( current < current_current) {
#endif  CURRENT_DIVISOR
            if (level_pwm < (PWM_MAX_VALUE - PWM_MAX_VALUE/16))  { level_pwm++;} // leaving some dead time to prevent inductor saturation
            }
#ifdef CURRENT_DIVISOR
          if (current > current_current/set_current_divisor){
#endif CURRENT_DIVISOR
#ifndef CURRENT_DIVISOR
          if (current > current_current){
#endif  CURRENT_DIVISOR
            if (level_pwm > 0) { level_pwm--;}
            }
//           SetCharger(level_pwm);
#ifdef USE_CUSTOM_PWM
            SetCharger(level_pwm); //some constant? //fixme // divide by four to add averaging 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,level_pwm);
#endif USE_CUSTOM_PWM            

#endif USE_EXTRA_SMOOTH_PWM


//perform all reads in one place

//            temperature = ntc.TemperatureC(ReadMultiDecimated(pin_tbat)) - ntc.TemperatureC(ReadMultiDecimated(NTC_ambient));
 // no need to read it every cycle

// if this happens... who cares? 
//            if (voltage_vbat > VOUT_MAX) {
              // HARDWARE PROTECTION
//              SetDischarger(false);
//              SetCharger(0);
//
//              Serial.print(F("DEVICE "));
//              Serial.print(iid);
//              Serial.println(F(": ABNORMALY HIGH OUTPUT VOLTAGE"));
//#ifdef USE_LCD
//              lcd.setCursor(0,iid-1);
//              lcd.print(F("Voltage abnormal"));
//              //         1234567890123456              
//#endif USE_LCD
//              state = 100;
//              break;
//            }
            
#ifdef CURRENT_DIVISOR // if cell has too high internal resistance, enable current divisor.
                      // without that function, just keep going... 
            if (voltage_vbat > V_BAT_MAX) {
              // voltage above max limit
#ifdef SERIAL_DEBUG_EVENTS              
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.print(F(": CHARGE VOLTAGE EXCEEDED LIMIT, INCREASING CURRENT DIVISOR : "));
#endif SERIAL_DEBUG_EVENTS

#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.setCursor(0,iid-1);
              lcd.print(F("Vbat>2V,I/"));
              //         1234567890123456
#endif LCD_VERBOSE                            
#endif USE_LCD
              set_current_divisor++;
#ifdef SERIAL_DEBUG_EVENTS               
              Serial.println(set_current_divisor);
#endif SERIAL_DEBUG_EVENTS
#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.print(set_current_divisor); 
#endif LCD_VERBOSE
#endif USE_LCD
#ifdef USE_CUSTOM_PWM
            SetCharger(0); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,0);
#endif USE_CUSTOM_PWM            
//              SetCharger(0);
              delay(5000);              
              if (set_current_divisor > 10) {
#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.setCursor(0,iid-1);
              lcd.print(F("OVERVOLT   EXIT"));
#endif LCD_VERBOSE
#endif USE_LCD
              state = 42; 
               }
              break;
            }
#endif CURRENT_DIVISOR

            now_ts = millis();
#ifdef USE_GLOBAL_TIMEOUT            
            if (now_ts > end_ts) {
              // time to end the charging
#ifdef SERIAL_DEBUG_EVENTS              
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.println(F(": CHARGING TIMEOUT"));
#endif SERIAL_DEBUG_EVENTS
#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.setCursor(0,iid-1);
              lcd.print(F("F charge timeout"));
              //         1234567890123456              
#endif LCD_VERBOSE
#endif USE_LCD              
              state = 42;
              break;
            }
#endif USE_GLOBAL_TIMEOUT
            
//            value_ibat = ReadMultiDecimated(pin_ibat);
//            voltage_ibat = GetVoltage(value_ibat, 65536);
// too slow
//            value_ibat = ReadMultiDecimated(pin_ibat,11);
//            voltage_ibat = GetVoltage(value_ibat, 2048);

//            value_ibat = ReadMultiDecimated(pin_ibat,11);

//            voltage_ibat = GetVoltage(ReadMultiDecimated(pin_ibat,10), 1024);
//            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;
// moved up to make it happen in one place 
            
/* // sorry, this magic is too black...
  //          if (current < I_HIGH_CURRENT_CHARGING) {
              if (current < current_current/set_current_divisor) {
  
              if (level_pwm_increment < 0) {
                // was falling
                level_pwm_increment = (abs(level_pwm_increment) < 2) ? 
                  -level_pwm_increment : -level_pwm_increment / 2;
              }
              if ((int16_t)( level_pwm + level_pwm_increment) > 0) {
                SetCharger(level_pwm + level_pwm_increment);
                } else {SetCharger(1);}
            } else {
              // was rising
              if (level_pwm_increment > 0) {
                // 
                level_pwm_increment = (abs(level_pwm_increment) < 2) ? 
                  -level_pwm_increment : -level_pwm_increment / 2;
              }
              if ((int16_t)( level_pwm + level_pwm_increment) > 0) {
                SetCharger(level_pwm + level_pwm_increment);
                }
*/

#ifdef USE_EXTRA_SMOOTH_PWM
            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,12,true), 4096,VREF_Vbat);
            voltage_ibat = GetVoltage(ReadMultiDecimated(pin_ibat,12), 4096);
            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;
            current_avg = (11 * current_avg + current) / 12;
#endif USE_EXTRA_SMOOTH_PWM

#ifdef CURRENT_DIVISOR
//          if ( (current - current_current/set_current_divisor) < 0.01 ){
            if ( current < current_current/set_current_divisor ){
#endif CURRENT_DIVISOR
#ifndef CURRENT_DIVISOR
//          if ( (current - current_current) < 0.01){
            if ( current < current_current) {
#endif  CURRENT_DIVISOR
            if (level_pwm < (PWM_MAX_VALUE - PWM_MAX_VALUE/16))  { level_pwm++;} // leaving some dead time to prevent inductor saturation
            }
#ifdef CURRENT_DIVISOR
          if (current > current_current/set_current_divisor){
#endif CURRENT_DIVISOR
#ifndef CURRENT_DIVISOR
          if (current > current_current){
#endif  CURRENT_DIVISOR
            if (level_pwm > 0) { level_pwm--;}
            }
//           SetCharger(level_pwm);
#ifdef USE_CUSTOM_PWM
            SetCharger(level_pwm); //some constant?
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,level_pwm);
#endif USE_CUSTOM_PWM            


// this should never happen. and if - we do not care. 
//            if (level_pwm >= 2047) {
//              // abnormal PWM level
//
//              Serial.print(F("DEVICE "));
//              Serial.print(iid);
//              Serial.println(F(": ABNORMAL PWM LEVEL"));
//#ifdef USE_LCD
//              lcd.setCursor(0,iid-1);
//              lcd.print(F("PWM abnormal    "));
//              //         1234567890123456
//#endif USE_LCD              
//              state = 42;
//              break;
//            }
  // why on earth would we care? if charge does not progress then sth is sure wrong, and we have all diag data out...          
//            if ((level_pwm > 1000) && (current < 0.1)) {
//             // voltage drop on shunt does not keep-up with the PWM level
//
//              Serial.print(F("DEVICE "));
//              Serial.print(iid);
//              Serial.println(F(": CHARGING CURRENT NOT KEEPING UP WITH PWM LEVEL"));
//#ifdef USE_LCD
//              lcd.setCursor(0,iid-1);
//              lcd.print(F("PWM MALFUNCTION "));
//              //         1234567890123456              
//#endif USE_LCD              
//              state = 42;
//              break;
//            }

#ifndef USE_FAN_AMBIENT //  if not using ambient fan
            if (now_ts > minute_ts) { // one minute elapsed. 
#endif USE_FAN_AMBIENT 
#ifdef USE_FAN_AMBIENT
            if ((now_ts > minute_ts) && (now_ts > STARTUP_FAN_TIMEOUT)) { // one minute elapsed, and we are past initial startup fan cooling phase             
#endif USE_FAN_AMBIENT

//             temperature = SimpleKalmanTemperature.updateEstimate(ntcTemperatureC(ReadMultiDecimated(pin_tbat,14),16384) - ntcTemperatureC(ReadMultiDecimated(NTC_ambient,14),16384));
//not really used now, as we use average anyway... 
              // time for stats gathering
              //minute_ts = now_ts + 60000;// - (now_ts - minute_ts);
              minute_ts += MINUTE_TS_REAL;
              
#ifdef USE_FAN_AMBIENT // do not calculate temperature slope until we get enough recent averages - another 10 minutes of dead time. 
              if (now_ts > (STARTUP_FAN_TIMEOUT*2)) {
              temperature_slope = temperature_avg - temperature_last;
              }
#endif USE_FAN_AMBIENT

#ifndef USE_FAN_AMBIENT
              temperature_slope = temperature_avg - temperature_last;
#endif USE_FAN_AMBIENT


              temperature_last = temperature_avg;

              if (current_current < current_target) { 
                current_current += current_target/CURRENT_INCREASE_RATE;                      // increase current , linearly. 
                }
              
              bCorrectSequence = 0;
              for (byte i = 0; i < TEMPERATURE_SLOPE_SEQUENCE_LENGTH - 1; i ++) {
                // 
                if (temperature_slope_sequence[i] < temperature_slope_sequence[i + 1]) {
                    bCorrectSequence  += temperature_slope_sequence[i];  // this is positive slope in positive slope event , recorded twice
                }                
                 bCorrectSequence += temperature_slope_sequence[i] ; // this is normal positive slope event. 
//                if (temperature_slope_sequence[i] < 0) {
//                  // 
//                  bCorrectSequence  -= temperature_slope_sequence[i] ;  // all negative slopes get accumulated twice too
//                } // no idea how this ever worked... 

                temperature_slope_sequence[i] = temperature_slope_sequence[i + 1];
              }
              temperature_slope_sequence[TEMPERATURE_SLOPE_SEQUENCE_LENGTH-1] = temperature_slope;
#ifdef USE_FAN_AMBIENT
              if (now_ts > STARTUP_FAN_TIMEOUT) {
              fan_ambient_state = 0; 
               }
#endif USE_FAN_AMBIENT
              
              if ( (bCorrectSequence) > MAX_CUMULATIVE_TEMPERATURE_INCREASE  ) {      // if temperature slopes on average exceed predefined delta.  
                // a steady growing temperature slope
                // consistent with dT end condition
#ifdef SERIAL_DEBUG_EVENTS
                Serial.print(F("DEVICE "));
                Serial.print(iid);
                Serial.println(F(": CHARGING COMPLETE: dT event, Tslope sequence:"));
                for (byte i = 0; i < TEMPERATURE_SLOPE_SEQUENCE_LENGTH - 1; i ++) {
                  // 
                  Serial.print(temperature_slope_sequence[i], 6);
                  Serial.print(", ");
                }
                Serial.print(F("; size = "));
                Serial.print(TEMPERATURE_SLOPE_SEQUENCE_LENGTH);
                Serial.println(F(" "));
#endif SERIAL_DEBUG_EVENTS
#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.setCursor(0,iid-1);
              lcd.print(capacity_in,0);
              lcd.print(F("mAh dT="));
              //         1234567890123456 
              lcd.print(bCorrectSequence,1);             
#endif LCD_VERBOSE
#ifndef LCD_VERBOSE
              lcd.setCursor(0,iid-1);
              lcd.print(capacity_in,0);
              lcd.print(F("mAh dT"));
              //         1234567890123456 
#endif  LCD_VERBOSE
#endif USE_LCD
                state = 42;
                break;
              }
            }
            
            if (now_ts > ts) {
              // each "ts" (usually 5 seconds)
              //ts = now_ts + 5000;// - (now_ts - ts);
              ts += 5000;
//better precision measurement for stats and graphs
//            value_vbat = ReadMultiDecimated_1_1(pin_vbat,15);
//            voltage_vbat = GetVoltage(value_vbat, 32768,VREF_Vbat);

            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,12,true), 4096,VREF_Vbat);
//            value_ibat = ReadMultiDecimated(pin_ibat,15);
//            voltage_ibat = GetVoltage(value_ibat, 32768);
            voltage_ibat = GetVoltage(ReadMultiDecimated(pin_ibat,12), 4096);
            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;
//              if (current_avg == 0) {
//                current_avg = current;
//              } else {
                current_avg = (11 * current_avg + current) / 12;
//               }

              capacity_in = (double)(now_ts - start_ts) * current_avg / 3600;
//              temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,15),32768) - ntcTemperatureC(ReadMultiDecimated(NTC_ambient,15),32768);
#ifndef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
              temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024,NTC_skew[iid-1]) - ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024,NTC_ambient_skew);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
              temperature = ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024) - ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024);
#endif USE_NTC_SERIES_SKEW

#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
// read temperature once each 5 secs is more wise as it is slow. 
//only 12 bit decimation for speedup , this is only for graph. 

#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
              temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024,NTC_skew[iid-1]),1) - filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024,NTC_ambient_skew),2);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
              temperature = filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024),1) - filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024),2);
#endif USE_NTC_SERIES_SKEW


#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE

              if (temperature > MAX_STATIC_TEMPERATURE_DELTA) {
                // temperature too high for high current charging
#ifdef SERIAL_DEBUG_EVENTS
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.println(F(": OVER TEMPERATURE"));
#endif SERIAL_DEBUG_EVENTS
#ifdef USE_LCD
#ifdef LCD_VERBOSE              
              lcd.setCursor(0,iid-1);
              lcd.print(F("T too high "));
              //         1234567890123456             
              lcd.print(temperature,4); 
              lcd.print(F("C"));
#endif LCD_VERBOSE
#endif USE_LCD                
                state = 42;
                break;
              }
              if (voltage_vbat_avg > voltage_max) {
                voltage_max = voltage_vbat_avg;
              }
              voltage_drop = (voltage_drop * 11 + (voltage_max - voltage_vbat_avg)) / 12;

              if (temperature_avg == 0) {
                temperature_avg = temperature;
              } else {
#ifdef USE_KALMAN_FOR_TEMPERATURE
                temperature_avg = (temperature_avg * 11.0 + filter_update(temperature)) / 12.0;
#endif USE_KALMAN_FOR_TEMPERATURE

#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
//        temperature_avg = (temperature_avg * 11.0 + filter_update(temperature,3)) / 12.0;
        temperature_avg = filter_update(temperature,3) ;

#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE


#ifndef USE_KALMAN_FOR_TEMPERATURE  
#ifndef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
                temperature_avg = (temperature_avg * 11.0 + temperature) / 12.0;
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
#endif USE_KALMAN_FOR TEMPERATURE

              }

              if (voltage_vbat_avg == 0) {
                voltage_vbat_avg = voltage_vbat;
              } else {
                voltage_vbat_avg = (voltage_vbat_avg * 11.0 + voltage_vbat) / 12.0;
              }

              if (voltage_drop >= 0.01) {
                // -dV end condition

#ifdef SERIAL_DEBUG_EVENTS
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.println(F(":CHARGING COMPLETE: -dV"));
#endif SERIAL_DEBUG_EVENTS
#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.setCursor(0,iid-1);
              lcd.print(F("-dV="));
              //         1234567890123456             
              lcd.print(voltage_drop,7);
              lcd.print(F("V"));
#endif LCD_VERBOSE
#endif USE_LCD                
                state = 42;
                break;
              }

/* this is exactly what we want to avoid - tripping on simply one delta. 
 
              if (temperature_slope >= 2.0) {
                // dT end condition

#ifdef SERIAL_DEBUG_EVENTS
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.print(F(": CHARGING COMPLETE: dT="));
              Serial.println(temperature_slope);
#endif SERIAL_DEBUG_EVENTS

#ifdef USE_LCD
#ifdef LCD_VERBOSE
              lcd.setCursor(0,iid-1);             
              lcd.print(F("rapid dT="));
              //         1234567890123456
              lcd.print(temperature_slope,4);             
#endif LCD_VERBOSE
#endif USE_LCD                
                state = 42;
                break;
              }
*/

#ifdef SERIAL_DEBUG_GRAPH              
              Serial.print(F("DEVICE "));
              Serial.print(iid);
              Serial.print(F(": "));
              Serial.print(now_ts);
              Serial.print(F("; V = "));
              Serial.print(voltage_vbat_avg, 6);
              Serial.print(F(" V; -dV = "));
              Serial.print(voltage_drop, 6);
              Serial.print(F(" V; Vr = "));
              Serial.print(voltage_ibat - voltage_vbat, 6);
              Serial.print(F(" V; I = "));
              Serial.print(current_avg, 6);
              Serial.print(F(" A; PWM = "));
              Serial.print(level_pwm);
              Serial.print(F("; Ploss = "));
              Serial.print(voltage_vbat_avg * current_avg, 6);
              Serial.print(F(" W; Tbat = "));
              Serial.print(temperature_avg, 6);
              Serial.print(F(" C; Ts = "));
              Serial.print(temperature_slope, 6);
              Serial.print(F(" C/min; "));
              Serial.print(F("dSlope[t] = "));
              Serial.print(bCorrectSequence,2);
              Serial.print(F(" ; Cout = "));
              Serial.print(capacity_in, 3);
              Serial.print(F(" mAh"));
              Serial.print(F(" ;Ierr = "));
              Serial.print(current_current-current_avg,6);
#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
              Serial.print(F(" ;Tamb = "));
#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,INPUT);
            pinMode(NTC_ambient,INPUT);
#endif USE_NTC_COOLING 
#ifdef USE_NTC_SERIES_SKEW
              Serial.print(filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024,NTC_ambient_skew),2),5);
              Serial.print(F(" ;Tbat = "));
              Serial.print(filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024,NTC_ambient_skew),1),5);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
              Serial.print(filter_update(ntcTemperatureC(ReadMultiDecimated(NTC_ambient,10),1024),2),5);
              Serial.print(F(" ;Tbat = "));
              Serial.print(filter_update(ntcTemperatureC(ReadMultiDecimated(pin_tbat,10),1024),1),5);
#endif USE_NTC_SERIES_SKEW


#ifdef USE_NTC_COOLING
            pinMode(pin_tbat,OUTPUT);
            pinMode(NTC_ambient,OUTPUT);
            digitalWrite(pin_tbat,LOW);         // short thermistor to ground to prevent self-heating
            digitalWrite(NTC_ambient,LOW);   // short thermistor to ground to prevent self-heating
#endif USE_NTC_COOLING 
#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE
              Serial.println(F(" "));
#endif SERIAL_DEBUG_GRAPH

#ifdef USE_LCD
              lcd.setCursor(0,iid-1);
              lcd.print(capacity_in,0);
              lcd.print(F("mAh "));
              lcd.print(voltage_vbat,3);
              lcd.print(F("V ")); 
#endif USE_LCD

            }
            break;
          }

          case 42: {
            // stopping charger
//            SetDischarger(false);
//            SetCharger(0); 
#ifdef USE_CUSTOM_PWM
            SetCharger(0); 
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,0);
#endif USE_CUSTOM_PWM            
                       
            state = 100;
            break;
          }
        }
          
        case 100: {
          // battery removal notice
//          SetCharger(0);
#ifdef USE_CUSTOM_PWM
            SetCharger(0);
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
            analogWrite(pin_pwm,0);
#endif USE_CUSTOM_PWM            

//          SetDischarger(false);
#ifdef SERIAL_DEBUG_EVENTS          
          Serial.print(F("DEVICE "));
          Serial.print(iid);
          Serial.println(F(": REMOVE BATTERY"));
#endif SERIAL_DEBUG_EVENTS
//#ifdef USE_LCD
//          lcd.setCursor(0,iid-1);
//          lcd.print("REMOVE   BATTERY"); // this obstructs other messages. 
          //         1234567890123456
//#endif USE_LCD
          state = 101;
          break;
        }
          
          
        case 101: {
          // waiting for battery removal
//          value_vbat = ReadMultiDecimated(pin_vbat);
//          voltage_vbat = GetVoltage(value_vbat, 65536); //5V vref
 //           delay(5000);

//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);
//            voltage_vbat = GetVoltage(value_vbat, 65536,VREF_Vbat);

//            value_vbat = ReadMultiDecimated_1_1(pin_vbat);
            voltage_vbat = GetVoltage(ReadMultiDecimated(pin_vbat,12,true), 4096,VREF_Vbat);

            now_ts = millis();

#ifdef SERIAL_DEBUG_GRAPH
//          value_ibat = ReadMultiDecimated(pin_ibat);
//          voltage_ibat = GetVoltage(value_ibat, 65536);

//          value_ibat = ReadMultiDecimated(pin_ibat);
          voltage_ibat = GetVoltage(ReadMultiDecimated(pin_ibat,10), 1024);
          current = (voltage_vbat - voltage_ibat) / SHUNT_RESISTOR;
#endif SERIAL_DEBUG_GRAPH
          if (voltage_vbat < V_BAT_MIN - 0.1) {
            // device not powered
            // battery removed
            state = 0;
          } else if (voltage_vbat > V_BAT_MAX) {
            // device powered 
            // battery removed
            state = 0;
          } else {
            // 
#ifdef USE_TONE
            if (now_ts > ts) {
              // each "ts" (usually 5 seconds)
              //ts = now_ts + 5000;// - (now_ts - ts);
              ts += 5000;

            tone0.play(1023,100);
            delay(100); 
            tone0.play(440,100);
//            delay(5000);
#ifdef SERIAL_DEBUG_GRAPH            
            Serial.print(F("DEVICE "));
            Serial.print(iid);
            Serial.print(F(": "));
            Serial.print(now_ts);
            Serial.print(F("; V = "));
            Serial.print(voltage_vbat, 6);
            Serial.print(F(" V, I = "));
            Serial.print(current, 6);
            Serial.println(F(" A"));
#endif SERIAL_DEBUG_GRAPH            

            } //if (now_ts > ts) {

#endif USE_TONE
#ifdef USE_NEWTONE
            if (now_ts > ts) {
              // each "ts" (usually 5 seconds)
              //ts = now_ts + 5000;// - (now_ts - ts);
              ts += 5000;

            NewTone(speaker_pin,1023,100);
            delay(100); 
            NewTone(speaker_pin,440,100);
//            delay(5000);
            noNewTone();
#ifdef USE_CUSTOM_PWM            
            ConfigPWM();
#endif USE_CUSTOM_PWM            
#ifdef SERIAL_DEBUG_GRAPH            
            Serial.print(F("DEVICE "));
            Serial.print(iid);
            Serial.print(F(": "));
            Serial.print(now_ts);
            Serial.print(F("; V = "));
            Serial.print(voltage_vbat, 6);
            Serial.print(F(" V, I = "));
            Serial.print(current, 6);
            Serial.println(F(" A"));
#endif SERIAL_DEBUG_GRAPH            

            } //if (now_ts > ts) {

#endif USE_NEWTONE

#ifdef USE_DUMBTONE
            if (now_ts > ts) {
              // each "ts" (usually 5 seconds)
              //ts = now_ts + 5000;// - (now_ts - ts);
              ts += 5000;

                for (uint8_t beep = 12 ; beep-->0;){
                  digitalWrite(speaker_pin,HIGH);
                  delay(1);
                  digitalWrite(speaker_pin,LOW);
                  delay(1);
                }//for (uint8_t beep = 12 ; beep-->0;){                
#ifdef SERIAL_DEBUG_GRAPH            
            Serial.print(F("DEVICE "));
            Serial.print(iid);
            Serial.print(F(": "));
            Serial.print(now_ts);
            Serial.print(F("; V = "));
            Serial.print(voltage_vbat, 6);
            Serial.print(F(" V, I = "));
            Serial.print(current, 6);
            Serial.println(F(" A"));
#endif SERIAL_DEBUG_GRAPH            

            } //if (now_ts > ts) {

#endif USE_DUMBTONE
          }
          break;
        }        
      }
      
        }
      
  private:

//    /// reads and accumulates multiple samples and decimates the result
//    static uint16_t ReadMultiDecimated(uint8_t pin, uint8_t bits = 16, bool vref_internal = false) {
//      // 
//
//      uint32_t total = 0;
//      bits -= 10;
//      uint16_t N = B00000001 << (2 * bits);
//      for (uint16_t i = 0; i < N; i++) {
//        // 
//        total += analogRead(pin);
//      }
//      return total >> bits;
//   }

        /// reads and accumulates multiple samples and decimates the result -1.1V VREF 
    static uint16_t ReadMultiDecimated(uint8_t pin, uint8_t bits = 16, bool vref_internal = false) {
      if (vref_internal){
      analogReference(INTERNAL); // switch to 1.1V internal reference      
      analogRead(pin);
      delay(6);
      analogRead(pin);
      }
      uint32_t total = 0;

      bits -= 10;
      uint16_t N = B00000001 << (2 * bits);
      for (uint16_t i = 0; i < N; i++) {
        // 
        total += analogRead(pin);
      }
      analogReference(DEFAULT); 
      analogRead(pin);
//      delay(4);
//     analogRead(pin);
      
      return total >> bits;
    }

    /// gets the voltage from the given ADC value
    static float GetVoltage(uint16_t value, uint16_t resolution = 1024, float vcc = VCC) {
      // 
      return (float) value / (resolution - 1) * vcc;
    }
    /// sets the PWM level for the charger's buck converter

#ifdef USE_CUSTOM_PWM
    void SetCharger(uint16_t level) { 
//      level_pwm = level;
      SetPWM(pin_pwm, level);
    }
#endif USE_CUSTOM_PWM      

 
    /// turns on and off the discharger
/*
    void SetDischarger(bool on) {
      // 
      digitalWrite(pin_dschrg, on ? HIGH : LOW);
    }
*/
/*
  static float ntcTemperatureC(uint16_t value,uint16_t ADCResolution = 65536) {
        //
        float tmpValue = ((ADCResolution - 1) / (float)value) - 1;
//      float tmpValue = ((ADCResolution - 1) / value) - 1;
//      tmpValue = NTC_SERIES_RESISTOR * tmpValue;
        tmpValue = NTC_SERIES_RESISTOR / tmpValue;
        float steinhart = tmpValue / NTC_NOMINAL_RESISTANCE;
        steinhart = log(steinhart);
        steinhart /= NTC_BETA_COEFFICIENT;
        steinhart += 1.0 / (NTC_NOMINAL_TEMPERATURE + 273.15);
        steinhart = 1.0 / steinhart;
        steinhart -= 273.15;
        return steinhart;
}
*/
#ifdef USE_NTC_SERIES_SKEW
#ifndef KALMAN_AUTOSKEW
   float ntcTemperatureC(uint16_t value,uint16_t ADCResolution = 65536,int16_t series_R_skew = 0 ) {
#endif KALMAN_AUTOSKEW
#ifdef KALMAN_AUTOSKEW // if kalman autoskew is on, use float for series_R_skew to allow easier convergence
   float ntcTemperatureC(uint16_t value,uint16_t ADCResolution = 65536,float series_R_skew = 0 ) {
#endif KALMAN_AUTOSKEW
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
   float ntcTemperatureC(uint16_t value,uint16_t ADCResolution = 65536 ) {
#endif  USE_NTC_SERIES_SKEW        
//
//        float tmpValue = ((ADCResolution - 1) / (float)value) - 1;
//        float tmpValue = (NTC_SERIES_RESISTOR + series_R_skew) / ((ADCResolution - 1) / (float)value) - 1;

//        float steinhart = (float)(((NTC_SERIES_RESISTOR + series_R_skew) / (((ADCResolution - 1) / (float)value) - 1)) / NTC_NOMINAL_RESISTANCE;
#ifdef USE_NTC_SERIES_SKEW
        float steinhart = log((float)(((float)NTC_SERIES_RESISTOR + series_R_skew) / (((ADCResolution - 1) / (float)value) - 1)) /(float) NTC_NOMINAL_RESISTANCE);
#endif USE_NTC_SERIES_SKEW
#ifndef USE_NTC_SERIES_SKEW
        float steinhart = log((float)(((float)NTC_SERIES_RESISTOR) / (((ADCResolution - 1) / (float)value) - 1)) /(float) NTC_NOMINAL_RESISTANCE);
#endif USE_NTC_SERIES_SKEW
        steinhart /= (float)NTC_BETA_COEFFICIENT;
        steinhart += 1.0 / (NTC_NOMINAL_TEMPERATURE + 273.15);
        steinhart = 1.0 / steinhart;
        steinhart -= 273.15;
        return steinhart;
}

//kalman filter

#ifdef USE_KALMAN_FOR_TEMPERATURE

static constexpr float k    = 1;
  static constexpr float Bk   = 0;
  static constexpr float uk   = 0;
  static constexpr float Fk   = 1;
  static constexpr float T    = 1;
  static constexpr float Fk_T = pow(Fk, T);
  static constexpr float Hk   = 1;
  static constexpr float Hk_T = pow(Hk, T);
  static constexpr float  I    = 1;

  const float Rk = KALMAN_Rk_FOR_TEMPERATURE;        
  const float Qk = KALMAN_Qk_FOR_TEMPERATURE;     

  float xk_last;
  float Pk_last;
float filter_update(float zk) {
    float xk       = (Fk * xk_last) + (Bk * uk); // Predicted (a priori) state estimate
    float Pk       = (Fk * Pk_last * Fk_T) + Qk; // Predicted (a priori) error covariance
    float yk       = zk - (Hk * xk);             // Innovation or measurement pre-fit residual
    float Sk = Rk + (Hk * Pk * Hk_T);      // Innovation (or pre-fit residual) covariance
    float Kk = (Pk * Hk_T) / Sk;           // Optimal Kalman gain
    xk         = xk + (Kk * yk);             // Updated (a posteriori) state estimate
    Pk         = (I - (Kk * Hk)) * Pk;       // Updated (a posteriori) estimate covariance (a.k.a Joseph form)

#if 0 // unused part
    float yk       = zk - (Hk_T * xk);           // Measurement post-fit residual
#endif

    xk_last = xk;
    Pk_last = Pk;

    return xk;
  }

#endif USE_KALMAN_FOR_TEMPERATURE

#ifdef USE_DYNAMIC_KALMAN_FOR_TEMPERATURE

static constexpr float k    = 1;
  static constexpr float Bk   = 0;
  static constexpr float uk   = 0;
  static constexpr float Fk   = 1;
  static constexpr float T    = 1;
  static constexpr float Fk_T = pow(Fk, T);
  static constexpr float Hk   = 1;
  static constexpr float Hk_T = pow(Hk, T);
  static constexpr float  I    = 1;

#ifndef KALMAN_AUTOSKEW
  float Rk[3] = {0.5,1.5,1.0}; // process covariance for each filter, initial values       
#endif KALMAN_AUTOSKEW
#ifdef KALMAN_AUTOSKEW // we make it longer for autoskew convergence as it's too chaotic
  float Rk[3] = {0.5,1,0.5}; // process covariance for each filter, initial values       
#endif KALMAN_AUTOSKEW

//               1 = battery temp
//                   2 = ambient temp
//                       3 = delta of inside box and battery

// 4 = delta of outside box (ambient) and inside box //fixme (currently not implemented, requires extra analog input -stm32 port and above)
#ifndef KALMAN_AUTOSKEW
  float Qk[3] = {0.0001,0.00001,0.001}; // measurement covariance for each filter, initial values
#endif KALMAN_AUTOSKEW
#ifdef KALMAN_AUTOSKEW
  float Qk[3] = {0.000001,0.00001,0.1}; // measurement covariance for each filter, initial values
#endif KALMAN_AUTOSKEW

//               1 = battery temp
//                      2 = ambient temp
//                             3 = delta of inside box and battery

//4 = delta of outside box (ambient) and inside box //fixme (currently not implemented, requires extra analog input -stm32 port and above)


  float xk_last[3] = {20,20,0} ;
  float Pk_last[3] = {0.1,0.1,1} ;
float filter_update(float zk,uint8_t filter_id) {
    float xk       = (Fk * xk_last[filter_id-1]) + (Bk * uk); // Predicted (a priori) state estimate
    float Pk       = (Fk * Pk_last[filter_id-1] * Fk_T) + Qk[filter_id-1]; // Predicted (a priori) error covariance
    float yk       = zk - (Hk * xk);             // Innovation or measurement pre-fit residual
    float Sk = Rk[filter_id-1] + (Hk * Pk * Hk_T);      // Innovation (or pre-fit residual) covariance
    float Kk = (Pk * Hk_T) / Sk;           // Optimal Kalman gain
    xk         = xk + (Kk * yk);             // Updated (a posteriori) state estimate
    Pk         = (I - (Kk * Hk)) * Pk;       // Updated (a posteriori) estimate covariance (a.k.a Joseph form)

#if 0 // unused part
    float yk       = zk - (Hk_T * xk);           // Measurement post-fit residual
#endif

    xk_last[filter_id-1] = xk;
    Pk_last[filter_id-1] = Pk;

    return xk;
  }

#endif USE_DYNAMIC_KALMAN_FOR_TEMPERATURE


//endof kalman filter



    /// temperature slope sequence length
    static const byte TEMPERATURE_SLOPE_SEQUENCE_LENGTH = TEMPERATURE_SLOPE_SEQUENCE_LENGTH_GLOBAL;
    float bCorrectSequence ; 
    /// state
    uint8_t state = 0;
    /// instance id
    uint8_t iid ;
    /// the ADC read value for Vbat
  //  uint16_t value_vbat , value_ibat ; // not used, placed reads directly into function...
    /// measured voltages 
    float voltage_vbat , voltage_ibat ;
    /// end-charging primary variables
    float voltage_max ;
    float temperature_last ;
    /// end-charging secondary variables
    float temperature_slope , temperature_avg, voltage_vbat_avg , voltage_drop ;
    /// temperature slope sequence
    float temperature_slope_sequence[TEMPERATURE_SLOPE_SEQUENCE_LENGTH] ;
    /// measured temperature
    float temperature , temperature_ambient  ; 
    /// determined shunt current
    float current , current_avg ;
    /// current divisor set by empirical tests - per slot, thus array
#ifdef CURRENT_DIVISOR
    uint8_t set_current_divisor  ; 
#endif CURRENT_DIVISOR     
    float current_target  ;   
    float current_current ;   
    /// determined powers
 
    float capacity_in ;
#ifdef DISCHARGER
    float capacity_out ; 
#endif DISCHARGER
    /// input Vbat, Ibat, Tbat pins
    uint8_t pin_vbat, pin_ibat, pin_tbat, pin_pwm;
    /// control charge_pwm and discharge pins
//    unsigned char pin_pwm, pin_dschrg;

    /// timestamps
    uint32_t now_ts , start_ts , ts , minute_ts ;
#ifdef USE_GLOBAL_TIMEOUT
    uint32_t end_ts; 
#endif USE_GLOBAL_TIMEOUT
    /// pwm level
#ifdef USE_CUSTOM_PWM    
    uint16_t level_pwm;
#endif USE_CUSTOM_PWM
#ifndef USE_CUSTOM_PWM
    uint8_t level_pwm;  // standard pwm uses 8bit
#endif USE_CUSTOM_PWM
    /// pwm level increment
//    int8_t level_pwm_increment = 0;
    /// thermistor probe
//    NTCThermistor ntc;
}; //switch (state) {
   

BatteryCharger charger_0, charger_1;
//BatteryCharger charger_0;
void setup() {
  // 

  pinMode(V_BAT0, INPUT);
  pinMode(V_BAT1, INPUT);
  pinMode(NTC0, INPUT);
  pinMode(NTC1, INPUT);
  pinMode(NTC_ambient, INPUT); 
  pinMode(I_BAT0, INPUT);
  pinMode(I_BAT1, INPUT);
  pinMode(CHARGE_RATE_KNOB,INPUT);

#ifdef USE_FAN_AMBIENT
  analogWrite(FAN_AMBIENT,0);
#endif USE_FAN_AMBIENT

#ifdef USE_CUSTOM_PWM
  pinMode(CH_PWM0,OUTPUT);
  digitalWrite(CH_PWM0,LOW);
  pinMode(CH_PWM1,OUTPUT);
  digitalWrite(CH_PWM1,LOW);
#endif USE_CUSTOM_PWM 

#ifdef LED_DEBUG
  pinMode(LED_pin,OUTPUT);
#endif LED_DEBUG  
//  charger_0.Config(V_BAT0, I_BAT0, NTC0, CH_PWM0, DSCH_0, LED_0, 1);
//  charger_1.Config(V_BAT1, I_BAT1, NTC1, CH_PWM1, DSCH_1, LED_1, 2);
  charger_0.Config(V_BAT0, I_BAT0, NTC0, CH_PWM0, 1);
  charger_1.Config(V_BAT1, I_BAT1, NTC1, CH_PWM1, 2);

#ifdef USE_CUSTOM_PWM
  ConfigPWM();
//  SetPWM(CH_PWM0, 0);
//  SetPWM(CH_PWM1, 0);
#endif USE_CUSTOM_PWM

#  
#ifdef SERIAL_GENERAL
  Serial.begin(57600);
#endif SERIAL_GENERAL
#ifdef USE_LCD
  lcd.begin(16, 2);
//  lcd.clear();
#endif USE_LCD
#ifdef USE_TONE
  tone0.begin(speaker_pin);
  tone0.play(analogRead(CHARGE_RATE_KNOB)+40,300);
#endif USE_TONE
#ifdef USE_NEWTONE
  NewTone(speaker_pin,analogRead(CHARGE_RATE_KNOB)+40,300);
  delay(300);
  noNewTone();
#ifdef USE_CUSTOM_PWM  
  ConfigPWM();
#endif USE_CUSTOM_PWM  
#endif USE_NEWTONE

#ifdef USE_DUMBTONE
  pinMode(speaker_pin,OUTPUT);
  for (uint8_t beep = 255 ; beep-->0;){
    digitalWrite(speaker_pin,HIGH);
    for (uint8_t delay_in_beep = 0-((analogRead(CHARGE_RATE_KNOB)>>2)) ; delay_in_beep-->0;){digitalWrite(speaker_pin,LOW);};
    digitalWrite(speaker_pin,LOW);
    delay(1);
  }
#endif USE_DUMBTONE

#ifdef KNOB_HIGH_PRECISION
  set_current = (float)map(analogRead(CHARGE_RATE_KNOB), 0, 1024, (0.1*1000),(I_HIGH_CURRENT_CHARGING*1000)) /1000; 
#ifdef USE_LCD
//  lcd.print(map(analogRead(CHARGE_RATE_KNOB), 0, 1024, (0.1*1000),(I_HIGH_CURRENT_CHARGING*1000)));
//  lcd.print(set_current*1000,0);
  lcd.print(set_current,3);
  lcd.print(F("A ,"));
#ifdef USE_GLOBAL_TIMEOUT  
  lcd.print(((CHARGE_TIMEOUT_VALUE/set_current/(1000))/3600) );
  lcd.print(F("h"));
#endif USE_GLOBAL_TIMEOUT
#endif USE_LCD
#endif KNOB_HIGH_PRECISION

#ifdef USE_FAN_AMBIENT
  fan_ambient_state = 255;
#endif USE_FAN_AMBIENT


//Serial.println(F("T1 T2 Tamb"));
}

void loop()  {
  // 
#ifdef LED_DEBUG
  digitalWrite(LED_pin,HIGH); //turn on the LED to measure time spent in routine  
#endif LED_DEBUG
  charger_0.Execute();
#ifdef LED_DEBUG
  digitalWrite(LED_pin,LOW); // turn off the LED to measure time spent in routine
#endif 
  charger_1.Execute();

  charger_0.Execute();
  charger_1.Execute();
  charger_0.Execute();
  charger_1.Execute();
#ifdef USE_FAN_AMBIENT
  analogWrite(FAN_AMBIENT,fan_ambient_state);
#endif USE_FAN_AMBIENT
  
#ifdef KNOB_HIGH_PRECISION
  set_current = (float)map(analogRead(CHARGE_RATE_KNOB), 0, 1024, (0.1*1000),(I_HIGH_CURRENT_CHARGING*1000)) /1000; 
#endif KNOB_HIGH_PRECISION
#ifndef KNOB_HIGH_PRECISION
  set_current = (float)analogRead(CHARGE_RATE_KNOB)/2000+1;
#endif KNOB_HIGH_PRECISION


}
