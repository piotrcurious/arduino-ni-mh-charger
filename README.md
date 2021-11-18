# arduino-ni-mh-charger
Ni-MH charger in c++, portable to almost any arduino-compatible microcontroller

This is based on project stolen from https://github.com/gameinstance/NiMH_smart_charger

I do it as coding excercise, but it works good enough so it can have some use to others. 

There are many fixes and changes, to sum it up :

* Currently discharger code is removed completely.
* Main good change is using additional NTC thermistor to detect ambient temperature.
This way temperature of each cell is difference inbetween ambient and batt temp.
This, plus removing spurious noise by usage of Kalman filter , allows to detect delta as small as 2C, 
allowing to charge even old, small capacity AAA batteries with high internal resistance. 
* extended Vbat resolution by using 1.1V internal reference for Vbat reading. 
 to switch inbetween internal 1.1 and 5V Vcc , extra boolean is passed to ReadMultiDecimated. 
 default is 'false' 
  static uint16_t ReadMultiDecimated(uint8_t pin, uint8_t bits = 16, bool vref_internal = false) {

This allows more robust -deltaV detection. 
Note that this also requires voltage divider being used, so for stand-alone devices it's better to revert to 
direct method, or using uC with 2.56V internal reference. 

* Analog knob allowing setting up charge current. 
changing current changes total timeout value aswell, so slow charging will not trip timeout protection prematurely. 
currently it is set to quite high values.
* ramping up charging current 
 this feature slowly increases charging current to set value over time. 
 starting current as well as current increase is defined by current set by analog knob. 
 It connects both worlds, allowing slow charge of battery, while ensuring solid deltaT or deltaV detection . 
 it seems that also low current charging at early phase ensures forming of good first layers of electroplated material, 
 and proper polarisation of electrolyte. 
* kalman filter , either just for temp delta, or for all thermal sensors. 
  it provides less random swings than heavy averaging, allowing better detection of dT event 
* LCD display 
* IR modulated serial terminal display - for even cheaper display. 
  now You can ditch LCD display completely. 
  Just connect IR diode to D13 (via 100ohm) to transmit 36khz modulated 1200baud serial data.
  connect 36khz IR demodulator (TSOP31236, can be ripped from old TV) to serial input of serial terminal, 
  like VT100 or https://www.serasidis.gr/circuits/TV_terminal/Small_TV_terminal.htm (default)
  You can either #define VT100_compatible_terminal
  or #define ATMEGA8_TVTERM_TERMINAL. 
  This way You can have many chargers for various sizes of batteries , and only one terminal/display. 
  no cables. no fuss. 
* few audio notification options up to choice (standard tone, newtone library (-1300k) , or 'dumbtone' - beeping without timers
* most of extra functions got #ifdef'd , so one can strip most of non-essential stuff, allowing stuffing the charger code
  into attiny8, and still allowing two channels and ambient temp sensor!
  rationale for porting to attiny/atmega8/small micros is that it allows building single-cell BMS cheaply. 
  This allows solutions like wind/solar/nuclear charger for extreme conditions , 
  where other battery chemistries cannot keep up, for use in repeaters, weather stations, cameras, sattelites. 
  
  or other way round, if all bells and whistles get included, it allows porting to high tech micros like ESP32, ST32 series, 
  atmega2650 etc. allowing more channels , better sensitivity and eyecandy on OLED display. 
  on atmega or blue pill , one can use four charging channels easily - amount of analog inputs is the limit.
  
* it just works. original code had some issues. 
  This version is extensively tested. Sliding window accumulates temperature deltas twice so any positive dT occuring as 
  continous increase is amplified. Sliding window can store up to 120minutes of data on atmega328. 
  On attiny8 one should use LOW_MEMORY_VERSION which cuts some corners, like collects data once per 2 minutes to allow
  120minutes of data collected. This allows charging at very slow rates, with good thermal insulation 100mA is possible. 
  For tabletop with crude battery holder and open box,  200mA is recommended minimum. 

*   USE_NTC_SERIES_SKEW option allows to 'fine tune' values of series resistors of NTC thermistors.
As it is difficult to get small tolerance thermistors and their series resistors, one can just use 
whatever is in the junk box, and then adjust to the real value by using this option. 

*  //#define KALMAN_AUTOSKEW 
  if You uncomment that, while USE_NTC_SERIES_SKEW option is also active, and You have LCD display enabled,
  device will try to find values of resistors automatically , assuming ambient NTC sensor is the reference. 
  It will also output temperature values to serial output , so You can use arduino plotter to see the offsets.
  
 after inputting estimated values into program, You can use KALMAN_AUTOSKEW_FREEZE to test Your values, 
 and checking on serial plotter how kalman filter converges after reset , heating up of sensors, choosing various thermal
 masses etc. etc. 

* main loop is considerably faster, most routines use 10bit ADC reads, and more detailed reads are used only for stats
 and averaging, each 5sec. this way PWM regulation is more responsive. 
 
* example graphing scripts using gnuplot thrown in as bonus. They have some quirks and are not idiotproof. 
 use if You know what they actually do. 

* NTC library implemented as function to save memory. It is not as linear and pretty as other NTC solutons,
and is source of quite large error if ntc series resistors are very off . 
Keeping NTC series resistors in similiar value range is good practice. 

Sketch uses 13832 bytes (42%) of program storage space. Maximum is 32256 bytes.
Global variables use 1712 bytes (83%) of dynamic memory, leaving 336 bytes for local variables. Maximum is 2048 bytes.
 
  It is recommended to match thermal mass of ambient temp. sensor to thermal mass of batteries, f.e. use dummy battery
  glued to sensor. In future versions I will try to implement some better heuristics and additional 
  'outside of the box' sensor to adjust Kalman gain depending on "forecasted" temperature changes. 
  
  Also box which is possible to be closed after inserting batteries (like glass or plexi shield window over top) 
  is recommended , this allows sensing end of charging quicker (smaller dT) 
  
  if someone plans to build ultra fast charger basing on that, it should be easy, 
  in this case small fan is recommended to avoid detecting heating up of batteries due to high current . 
  
  It is still work in progress, there is still loads to do.
  Some TODO :
  * implementing sleep mode and input voltage detection, evaluation, heuristics, some sort of clock or timer, etc.
   this is really vague subject, it depends on application - solar charger, wind charger, sattelite, radio repeater etc, 
   all have different requirements. Some basic hooks could be implemented though , like ability to recieve commands
   like waking on interrupt and initiating start of charge by 'button press' event (or signal from another micro),
   responding to commands over serial port, telemetry. 
  * state of charge heuristics
  * balancing of cells instead of idling after end of charge 
  (measuring open circuit voltage and balancing cells to have equal SOC by mere coloumb counting method)
  * multi-cell version ("battery pack" where all cells are in series and only deltaT is detected on individual cells)
  * code cleanup
  * non linear shunt resistor element, or other way to detect small current - for power saving.
   while knowing accurate current is usefull for estimation of mAh gone into cell, it is not really needed for charging. 
   
  Few hardware design hints :
 * analog inputs do not need any filtering nor capacitors - actually it makes things worse.
 * 5V supply voltage regulation and filtering is most important for clean signals,
  for charge rate over 200mA this does not matter too much. Capacitors, coils, good voltage regulator and care to filter
  noise around switching mosfets make most effect on getting clean temperature, voltage and current readings. 
  
  Have fun. 
  
  
  
