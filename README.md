# arduino-ni-mh-charger
Ni-MH charger in c++, portable to almost any arduino-compatible microcontroller

This is based on project stolen from https://github.com/gameinstance/NiMH_smart_charger

I do it as coding excercise, but it works good enough so it can have some use to others. 

There are many fixes and changes, to sum it up :

-Currently discharger code is removed completely.
-Main good change is using additional NTC thermistor to detect ambient temperature.
This way temperature of each cell is difference inbetween ambient and batt temp.
This, plus removing spurious noise by usage of Kalman filter , allows to detect delta as small as 2C, 
allowing to charge even old, small capacity AAA batteries with high internal resistance. 
-Analog knob allowing setting up charge current. 
changing current changes total timeout value aswell, so slow charging will not trip timeout protection prematurely. 
currently it is set to quite high values.
-ramping up charging current 
 this feature slowly increases charging current to set value over time. 
 starting current as well as current increase is defined by current set by analog knob. 
 It connects both worlds, allowing slow charge of battery, while ensuring solid deltaT or deltaV detection . 
 it seems that also low current charging at early phase ensures forming of good first layers of electroplated material, 
 and proper polarisation of electrolyte. 
 -kalman filter , either just for temp delta, or for all thermal sensors. 
  it provides less random swings than heavy averaging, allowing better detection of dT event 
 -LCD display 
 -few audio notification options up to choice (standard tone, newtone library (-1300k) , or 'dumbtone' - beeping without timers
 -most of extra functions got #ifdef'd , so one can strip most of non-essential stuff, allowing stuffing the charger code
  into attiny8, and still allowing two channels and ambient temp sensor!
  rationale for porting to attiny/atmega8/small micros is that it allows building single-cell BMS cheaply. 
  This allows solutions like wind/solar/nuclear charger for extreme conditions , 
  where other battery chemistries cannot keep up, for use in repeaters, weather stations, cameras, sattelites. 
  
  or other way round, if all bells and whistles get included, it allows porting to high tech micros like ESP32, ST32 series, 
  atmega2650 etc. allowing more channels , better sensitivity and eyecandy on OLED display. 
  on atmega or blue pill , one can use four charging channels easily - amount of analog inputs is the limit.
  
 -it just works. original code had some issues. 
  This version is extensively tested. Sliding window accumulates temperature deltas twice so any positive dT occuring as 
  continous increase is amplified. Sliding window can store up to 120minutes of data on atmega328. 
  On attiny8 one should use LOW_MEMORY_VERSION which cuts some corners, like collects data once per 2 minutes to allow
  120minutes of data collected. This allows charging at very slow rates, with good thermal insulation 100mA is possible. 
  For tabletop with crude battery holder and open box,  200mA is recommended minimum. 
  
  It is recommended to match thermal mass of ambient temp. sensor to thermal mass of batteries, f.e. use dummy battery
  glued to sensor. In future versions I will try to implement some better heuristics and additional 
  'outside of the box' sensor to adjust Kalman gain depending on "forecasted" temperature changes. 
  
  Also box which is possible to be closed after inserting batteries (like glass or plexi shield window over top) 
  is recommended , this allows sensing end of charging quicker (smaller dT) 
  
  if someone plans to build ultra fast charger basing on that, it should be easy, 
  in this case small fan is recommended to avoid detecting heating up of batteries due to high current . 
  
  Have fun. 
  

