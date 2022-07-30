// *********************************************************************************************//
// Blacklfy Camera Sync with Teensy 3.x
// *********************************************************************************************//
//
// Description
// -----------
// This code uses a Teensy 3.2 microcontroller to control an external light source for 
// Team 21050's Rapid Optical Imaging of Physiological Process (ROIPP)
// engineering senior design project for the University of Arizona (ENGR 498) 2021.
//
// Requirements
// ------------
// This sofware requires a Teensy 3.x Microcontroller.
//
// Setup
// -----
// The Camera Line 2 will need to be programmed to be the Frame Sync
// Lin2   Pin7
// All PWM pins and one software PWM pin (7) can be configured
//
// Operation
// ---------
// This code cycles through 13 LEDs (+1 spare), 12 of which are PWM capable with the other 
// being only digitally capable, upon an external source from a Blackfly camera. 
// The camera provides a trigger on the GPIO line, letting the teensy know that a picture is 
// about to be taken. Through this GPIO signal, the camera and microcontroller are able to 
// to synchronize the camera images to the 13 LED light sources.
 
// By pressing a button, the system is able to be turned on and off such that all LEDs are on or off. 
// Moreover, the user is able to change the PWM duty cycle for the various LEDs and even 
// set unique LED sequences.
//
// To LED sequence is stored in EEPROM and can be altered with USB attached terminal.
//
// Shanon McCoy,Urs Utzinger, February 2021, Tucson Arizona, USA
//
// *********************************************************************************************//
// Copyright (c) 2021
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
// and associated documentation files (the "Software"), to deal in the Software without restriction, 
// including without limitation the rights to use, copy, modify, merge, publish, distribute, 
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
//
// The copyright notice and this permission notice shall be included in all copies or substantial 
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// *********************************************************************************************//

// Use software PWM on pin 7 or use constant on
#define USESOFTPWM

#include "SoftPWM.h"
#include <iterator>
#include <algorithm>

// *********************************************************************************************//
// Teensy 3.2 Specifics
// PWM pins: 3,4,5,6,9,10,20,21,22,23,25,32
// ADC pins: 14,15,16,17,18,19,20,21,22,23,26,27,28, 29,30,31,A10,A11,A12,A13 (for SPEC_VIDEO)
// Pin 11 can be used for ADC external trigger
// SERIAL_TX and RX can be: 1,0 3,4 
// PWM pins use differen timers
// FTM1 3, 4,
// FTM0 5, 6, 9, 10, 20, 21, 22, 23,
// FTM2 25, 32
// https://www.pjrc.com/teensy/pinout.html
// *********************************************************************************************//

// *********************************************************************************************//
// Machine States:
// ------------------------------------------------------------------------
// Manual:   Work on single PWM channel through USB terminal
// Auto:     Autoadvance to next channel when external Frame Trigger occurs
// Off:      PWM output is off
// ------------------------------------------------------------------------
enum states {Manual, Auto, Off};
volatile states myState = Off;
volatile states myPreviousState = Off;  // keeping track of previous state
// *********************************************************************************************//

// *********************************************************************************************//
// Pin Names:
// ------------------------------------------------------------------------
//Camera Trigger
#define CAMTRG           8  // Camera frame trigger, needs to go to pin 8 (input)
// Hardware PWM Channels
#define PWM1             3  // Connect to LED driver channel  1 (outut)
#define PWM2             4  // Connect to LED driver channel  2 (outut)
#define PWM3             5  // Connect to LED driver channel  3 (outut)
#define PWM4             6  // Connect to LED driver channel  4 (outut)
#define PWM5             9  // Connect to LED driver channel  5 (outut)
#define PWM6             10 // Connect to LED driver channel 6 (outut)
#define PWM7             20 // Connect to LED driver channel 7 (outut)
#define PWM8             21 // Connect to LED driver channel 8 (outut)
#define PWM9             22 // Connect to LED driver channel 9 (outut)
#define PWM10            23 // Connect to LED driver channel 10 (outut)
#define PWM11            25 // Connect to LED driver channel 11 (outut)
#define PWM12            32 // Connect to LED driver channel 12 (outut)
// Software PWM Hardware Channels
#define PWM13             7 // Connect to LED driver channel 13 (outut)
#define BLANK            99 // Connect to LED driver virtual channel 99 (outut)
// Power Button
#define POWERSWITCH       2 // External Button
#define LEDPIN           13 // buitin LED
// *********************************************************************************************//

// *********************************************************************************************//
// LED channel configuration
// ------------------------------------------------------------------------
#define NUMLEDS 14
volatile int             ch = 2;   // Start LED cycle off at LEDs[0]
volatile int         LEDs[] = {PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9, PWM10, PWM11, PWM12, PWM13, BLANK };    // LEDs
volatile bool  LEDsEnable[] = {true, true, true, true, true, true, true, true, true, true,  true,  true,  true,  true};  // Is channel on or off=false
volatile float  DutyCycle[] = {10.,  10.,  10.,  10.,  10.,  10.,  10.,  10.,  10.,  10.,   10.,   10.,   10.,   0.  };    // Duty cycle of the LEDs (ie 10. = 10% duty cycle)


// *********************************************************************************************//
// Button Debounce
// ------------------------------------------------------------------------
volatile long  lastInterrupt; // Keeping track

// *********************************************************************************************//
// EEPROM configuration
// ------------------------------------------------------------------------
#include <EEPROM.h>
#define EEPROM_SIZE 2048 // Max size
int eepromAddress = 0;   // Where storage starts
struct EEPROMsettings {            // Setting structure
  byte         valid;              // First time use of EEPROM?
  states       myState;            // Whater to be in Autoadvance or Manual or Off Mode
  int          LEDs[NUMLEDS];      // Which pins are the LEDs attached
  bool         LEDsEnable[NUMLEDS]; // Is this LED channel used
  float        DutyCycle[NUMLEDS]; // Duty cycle of the LEDs (ie 10 = 10% duty cycle)
  float        PWM_Frequency;      //
  unsigned int PWM_Resolution;     //
  };
EEPROMsettings mySettings;         // the settings

// *********************************************************************************************//
// Intervals and Timing
// ------------------------------------------------------------------------
#define POLL_INTERVAL          10000    // desired main loop time in microseconds 
#define CHECKINPUT_INTERVAL    50000    // interval in microseconds between polling serial input
#define LEDON_INTERVAL        100000    // interval in microseconds between turning LED on/off for status report
#define LEDOFF_INTERVAL       400000    // interval in microseconds between turning LED on/off for status report
// *********************************************************************************************//
unsigned long pollInterval;                    //
unsigned long lastInAvail;                     //
unsigned long lastPoll;                        //
unsigned long currentTime;                     //
unsigned long nextLEDCheck;                    //

// *********************************************************************************************//
// Controlling Reporting and Input Output
// ------------------------------------------------------------------------
bool SERIAL_REPORTING = true;                 // Boot messages on/off

// *********************************************************************************************//
// Indicator
// ------------------------------------------------------------------------
const int ledPin = LEDPIN;                    // Check on https://www.pjrc.com/teensy/pinout.html; pin should not interfere with I2C and SPI
bool ledStatus = false;                       // Led should be off at start up


// *********************************************************************************************//
// Main System and PWM working variables
// ------------------------------------------------------------------------
unsigned int  PWM_Pin        = PWM3;           //
bool          PWM_Enabled    = false;          //
long          CPU_Frequency  = F_CPU / 1E6;    //
float         PWM_Frequency  = 488.28;         // Default Teensy 3.2
unsigned int  PWM_Resolution = 8;              // 
unsigned int  PWM_MaxValue   = pow(2, PWM_Resolution)-1;
float         PWM_Duty       = 10.0;           //

// *********************************************************************************************//
// Serial
#define SERIAL_PORT_SPEED    2000000    // Serial Baud Rate
char   inBuff[] = "----------------";
int    bytesread;

// *********************************************************************************************//
// *********************************************************************************************//
// SETUP                                                                                        //
// *********************************************************************************************//
// *********************************************************************************************//
void setup(){

  // Output Pins
  pinMode(PWM1,        OUTPUT); digitalWriteFast(PWM1, LOW);
  pinMode(PWM2,        OUTPUT); digitalWriteFast(PWM2, LOW);
  pinMode(PWM3,        OUTPUT); digitalWriteFast(PWM3, LOW);
  pinMode(PWM4,        OUTPUT); digitalWriteFast(PWM4, LOW);
  pinMode(PWM5,        OUTPUT); digitalWriteFast(PWM5, LOW);
  pinMode(PWM6,        OUTPUT); digitalWriteFast(PWM6, LOW);
  pinMode(PWM7,        OUTPUT); digitalWriteFast(PWM7, LOW);
  pinMode(PWM8,        OUTPUT); digitalWriteFast(PWM8, LOW);
  pinMode(PWM9,        OUTPUT); digitalWriteFast(PWM9, LOW);
  pinMode(PWM10,       OUTPUT); digitalWriteFast(PWM10, LOW);
  pinMode(PWM11,       OUTPUT); digitalWriteFast(PWM11, LOW);
  pinMode(PWM12,       OUTPUT); digitalWriteFast(PWM12, LOW);
  pinMode(PWM13,       OUTPUT); digitalWriteFast(PWM13, LOW);
  // Input Pins
  pinMode(CAMTRG,      INPUT_PULLUP);
  pinMode(POWERSWITCH, INPUT_PULLUP);                                 // initialize pin for power switch

  //EEPROM
  EEPROM.get(eepromAddress, mySettings);
  if (mySettings.valid == 0xF0) { // apply settings because EEPROM has valid settings
    PWM_Frequency  = mySettings.PWM_Frequency;
    PWM_Resolution = mySettings.PWM_Resolution;
    PWM_MaxValue   = pow(2,PWM_Resolution)-1;  
    for (int i=0; i<NUMLEDS; i++) {
      LEDs[i]       = mySettings.LEDs[i];      
      LEDsEnable[i] = mySettings.LEDsEnable[i];
      DutyCycle[i]  = mySettings.DutyCycle[i];        
    }
    DutyCycle[NUMLEDS-1] = 0.; // Background image has no LEDs on
    myState        = mySettings.myState;
  } else {
    // create default values
    PWM_Resolution = 8;
    PWM_Frequency  = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution); 
    PWM_MaxValue   = pow(2,PWM_Resolution)-1;
    unsigned int  tmp[NUMLEDS] =  {PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9, PWM10, PWM11, PWM12, PWM13, BLANK };   
    for (int i=0; i<NUMLEDS; i++) {
      LEDs[i]       = tmp[i];      
      LEDsEnable[i] = true; 
      DutyCycle[i]  = 5.0;        
    }
    DutyCycle[NUMLEDS-1] = 0.; // Background image has no LEDs on
    myState        = Auto;
  }

  // PWM
  SoftPWMBegin();

  // Start PWM at 0% output
  // void setupPWM(uint16_t PWM_Pin, float PWM_MaxFreq, float PWM_Duty, unsigned int PWM_Resolution);
  setupPWM(PWM1,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM2,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM3,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM4,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM5,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM6,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM7,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM8,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM9,  PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM10, PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM11, PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM12, PWM_Frequency, 0.0, PWM_Resolution);
  setupPWM(PWM13, PWM_Frequency, 0.0, PWM_Resolution);

  // Serial startup
  Serial.begin(SERIAL_PORT_SPEED);
  if (SERIAL_REPORTING) {
    Serial.println("Starting System");
    printHelp();
  }

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(CAMTRG),   frameISR, RISING); // Frame start
  attachInterrupt(digitalPinToInterrupt(POWERSWITCH),  onOff, CHANGE); // Create interrupt on pin for power switch. Trigger when there is a change detected (button is pressed)

  // House keeping
  pollInterval = POLL_INTERVAL; // 1 m sec
  lastInAvail = lastPoll = lastInterrupt = nextLEDCheck = micros();
  ledStatus = true;
  digitalWrite(ledPin, ledStatus); // initialization completed

} // end setup

// *********************************************************************************************//
// *********************************************************************************************//
// Main LOOP                                                                                    //
// *********************************************************************************************//
// *********************************************************************************************//

void loop(){
  
  // Time Keeper
  //////////////////////////////////////////////////////////////////
  currentTime = micros();                                         // whats the time

  // Main loop delay. Limits how often this loop is running
  //////////////////////////////////////////////////////////////////
  int pollDelay = (pollInterval - (int)(currentTime - lastPoll)); // how long do we need to wait?
  if (pollDelay > 0) {
    delayMicroseconds(pollDelay);  // wait
  }
  lastPoll = currentTime;

  // Input Commands
  //////////////////////////////////////////////////////////////////
  if ((currentTime - lastInAvail) >= CHECKINPUT_INTERVAL) {
    if (Serial.available()) {
      bytesread = Serial.readBytesUntil('\n', inBuff, 16);      // Read from serial until CR is read or timeout exceeded
      inBuff[bytesread] = '\0';
      String instruction = String(inBuff);
      processInstruction(instruction);
    }
    lastInAvail = currentTime;
  }
  
  // Blink LED
  //////////////////////////////////////////////////////////////////
  if (currentTime > nextLEDCheck) {
    if (ledStatus) {
      // LED is ON
      ledStatus = false; 
      digitalWriteFast(ledPin, ledStatus);                        // turn off
      nextLEDCheck = currentTime + LEDOFF_INTERVAL;
    } else {
      // LED is OFF
      ledStatus = true;
      digitalWriteFast(ledPin, ledStatus);                        // turn on
      nextLEDCheck = currentTime + LEDON_INTERVAL;
    }
  }

} 

// *********************************************************************************************//
// Interrupt Service Routines                                                                   //
// *********************************************************************************************//

// Frame Trigger Interrupt Service Routine
//
// Turns off current LED
// Checks if LED is enabled
// Turns on next LED
//////////////////////////////////////////////////////////////////
void frameISR() {
  switch (myState)  {
    case Off:
      // do nothing
      break;
    case Auto:
      if (LEDs[ch] == 7) {  
        SoftPWMSet(7, 0); // Turn off the current LED 
      } else if (ch < NUMLEDS-1) { 
        analogWrite(LEDs[ch], uint16_t(0)); // Turn off the current LED
      } else {
        // do nothing, was background measurement
      }
      ch = (ch + 1)%NUMLEDS;// Increment LED channel to next index
      while (LEDsEnable[ch] == false) {ch = (ch + 1)%NUMLEDS;}  // Get the ch to the next LED that is part of the cycle sequence
      if (ch < NUMLEDS-1) { // keep last channel for background
        if (LEDs[ch] == 7) {  SoftPWMSetPercent(LEDs[ch], (uint8_t)(DutyCycle[ch]));             }  // Turn on the LED
        else               {  analogWrite(LEDs[ch], uint16_t(DutyCycle[ch]*PWM_MaxValue/100.0)); }  
      } else {
        // do nothing for background measurement
      }
      break;
    case Manual:
      // do nothing
      break;
  }
}

// Button ISR, toggles On/Off state of system
//
// turns off all LEDs for Off state
// Debounces 20ms
//////////////////////////////////////////////////////////////////
void onOff() {
  unsigned long currentTime = micros();
  if (currentTime - lastInterrupt > 20000) {                       // debounce condition
    if ((myState == Auto) || (myState == Manual)) {
      myPreviousState = myState;                                   // keep track if state was manual or auto
      myState = Off;
      for (int i=0; i<NUMLEDS-1; i++)  {                           // Turen off all PWM except last channel which is background
        if (LEDs[i] == 7) {  SoftPWMSet(7, 0); } else {  analogWrite(LEDs[ch], uint16_t(0)); }      
      }
    } else {
      myState = myPreviousState; // switch back to auto or manual
    }
    lastInterrupt = currentTime;  // keep track of time for debouncing
  }  
}

// *********************************************************************************************//
// Support Functions                                                                            //
// *********************************************************************************************//

// Teensy 3.2 realtion between PWM frequency and PWM resolution
//////////////////////////////////////////////////////////////////
float GetMaxPWMFreqValue(long FREQ, int PWM_Resolution)
{
  /* for Teensy CPU frequencies are
      24MHZ
      48MHZ
      72MHZ
      96MHZ
      120MHZ
  */
  int FREQ_pointer = -1;
  float PWM_ideal_frequency[5][15]
  {
    {6000000 , 3000000, 1500000, 750000 , 375000, 187500, 93750 , 46875   , 23437.5 , 11718.75 , 5859.375, 2929.687, 1464.843, 732.421, 366.2109},
    {12000000, 6000000, 3000000, 1500000, 750000, 375000, 187500, 93750   , 46875   , 23437.5  , 11718.75 , 5859.375, 2929.687, 1464.843, 732.4218},
    {9000000 , 4500000, 2250000, 1125000, 562500, 281250, 140625, 70312   , 35156.25, 17578.12 , 8789.062, 4394.531, 2197.265, 1098.632, 549.3164},
    {12000000, 6000000, 3000000, 1500000, 750000, 375000, 187500, 93750   , 46875   , 23437.5  , 11718.75 , 5859.375, 2929.687, 1464.843, 732.4218},
    {15000000, 7500000, 3750000, 1875000, 937500, 468750, 234375, 117187.5, 58593.75, 29296.875, 14648.537, 7324.219, 3662.109, 1831.055, 915.527 }
  };

  switch (FREQ) {
    case 24:  FREQ_pointer =  0; break;
    case 48:  FREQ_pointer =  1; break;
    case 72:  FREQ_pointer =  2; break;
    case 96:  FREQ_pointer =  3; break;
    case 120: FREQ_pointer =  4; break;
    default:  FREQ_pointer = -1; break;
  }
  if (FREQ_pointer >= 0) { return (PWM_ideal_frequency[FREQ_pointer][PWM_Resolution - 2]); } 
  else { return (488.28); }
} // end of getMaxPWMFreqValue

void listFrequencies()
{
  Serial.println("-------------------------------------------------");
  for(int i=2; i<=16; i++) {
    Serial.printf("Resolution: %2d bit, Frequency: %f\n", i, GetMaxPWMFreqValue(CPU_Frequency, i));
  }
  Serial.println("-------------------------------------------------");  
} 

boolean isPWM(uint8_t mypin) {
  const uint8_t last = 12;
  const uint8_t pwmpins[] = {3, 4, 5, 6, 8, 9, 10, 20, 21, 22, 23, 25, 32};
  return std::binary_search(pwmpins, &pwmpins[last], mypin);
}

boolean isSoftPWM(uint8_t mypin) {
  const uint8_t last = 20;
  const uint8_t softpwmpins[] = {0, 1, 2, 7, 8, 11, 12, 14, 15, 16, 17, 18, 19, 24, 26, 27, 28, 29, 30, 31, 33};
  return std::binary_search(softpwmpins, &softpwmpins[last], mypin);
}

void listPins() {
  char pinType[] = "Soft";
  Serial.println("-------------------------------------------------");
  for (int i=0; i<33; i++){
    if      (isPWM(i))     {strcpy(pinType, "PWM");}
    else if (isSoftPWM(i)) {strcpy(pinType, "Soft");}
    else                   {strcpy(pinType, "N.A.");}
    Serial.printf("Pin: %2d, %s\n", i, pinType );
  }
  Serial.println("-------------------------------------------------");  
}

// Setup PWM modulation on a pin
// This is slow and should only be used for setup and not to change dutycyle
//////////////////////////////////////////////////////////////////
void setupPWM(uint16_t PWM_Pin, float PWM_Freq, float Duty, unsigned int Resolution) {
  if (isPWM(PWM_Pin)) {
    if ((PWM_Resolution >= 2) && (PWM_Resolution <= 15)) { // check bounds
      analogWriteResolution(PWM_Resolution);    // change resolution
      PWM_MaxValue = pow(2, PWM_Resolution)-1; // update global
    }
    if (PWM_Freq > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
      PWM_Freq = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
    }
    analogWriteFrequency(PWM_Pin, PWM_Freq);
    PWM_Frequency = PWM_Freq;                   // update global
    if ((Duty >= 0.0) && (Duty <= 100.0)) {    
      analogWrite(PWM_Pin, uint16_t(Duty / 100.0 * float(PWM_MaxValue)));
      PWM_Duty = Duty;                          // update global
    }
  } else if (isSoftPWM(PWM_Pin)) {
    if (Duty <= 100.0) {    
      if (Duty > 0.001) {
        SoftPWMSetPercent(PWM_Pin, (uint8_t)(PWM_Duty));
        PWM_Duty = Duty;                          // update global
      } else {
        SoftPWMSet(PWM_Pin, 0);
        PWM_Duty = 0;                            // update global
      }
    }
  } // is hard/soft PWM
}

// What are the software options?
//////////////////////////////////////////////////////////////////
void printHelp() {
  Serial.println("-------------------------------------------------");
  Serial.println("LED Controller");
  Serial.println("-------------------------------------------------");
  Serial.println("i/I  system information");
  Serial.println("-------------------------------------------------");
  Serial.println("s0   load channel 0 to current working settings");
  Serial.println("S0   save channel 0 from current working settings");
  Serial.println("-------------------------------------------------");
  Serial.println("a/A  disable/enable Auto Advance"); 
  Serial.println("m/M  disable/enable PWM pin"); 
  Serial.println("x    print channel settings");
  Serial.println("e/E  read/save settings to EEPROM");
  Serial.println("------- Data Input--------------------------------");
  Serial.println("p5   set PWM pin 5");
  Serial.println("d50  set duty cyle to 50%");
  Serial.println("f512 set frequency to 512Hz");
  Serial.println("r8   set PWM resolution to 8 bits");
  Serial.println("-------------------------------------------------");
  Serial.println("Shannon McCoy, Urs Utzinger, 2020-21");
  Serial.println("-------------------------------------------------");
}

void printSystemInformation() {
  Serial.println("-------------------------------------------------");
  Serial.printf( "Frequency: %f Hz\n", PWM_Frequency);
  Serial.printf( "Duty: %+4.3f percent\n", PWM_Duty);
  Serial.printf( "Resolution: %2d bit\n", PWM_Resolution);
  Serial.printf( "CPU: %2d MHz\n", CPU_Frequency);
  Serial.printf( "PWM Max: %4d\n" , PWM_MaxValue);
  Serial.printf( "Pin: %2d %s\n", PWM_Pin, PWM_Enabled?"Enabled":"Disabled");
  Serial.printf( "Channel: %d\n", ch);
  Serial.print(  "State is: ");
  if (myState == Off)    { Serial.println("Off"); }
  if (myState == Auto)   { Serial.println("Auto"); }
  if (myState == Manual) { Serial.println("Manual"); }
  printChannels();
}

void printPinInformation() {
  Serial.println("-------------------------------------------------");
  Serial.println("Maximum Values:");
  listFrequencies();
  Serial.println("SoftPWM:"); 
  Serial.println("Resolution: 8 bit, Frequency: 60.0");
  listPins();
}

void printChannels() {
  Serial.println("-------------------------------------------------");
  for (int i=0; i<NUMLEDS; i++) {
    Serial.printf( "%2d Pin: %2d", i, LEDs[i]);
    Serial.printf( " Duty: %+4.3f", DutyCycle[i]);
    Serial.printf( " Enabled: %s\n", LEDsEnable[i]?"Yes":"No");    
  }
  Serial.println("-------------------------------------------------");
}

// SerialCommand handlers
//////////////////////////////////////////////////////////////////

void processInstruction(String instruction) {
  String value    = "0.01";
  String command  = "o";
  float  tempFloat;
  long   tempInt;
  int instructionLength = instruction.length();
  if (instructionLength > 0) { command = instruction.substring(0,1); } 
  if (instructionLength > 1) {   value = instruction.substring(1,instructionLength); }
  //mySerial.println(command);
  //mySerial.println(value);

  if        (command == 'a') { // manual mode
    // ENABLE/DISABLE Autodavance based on Frame Trigger////////////////////////////
    myState = Manual;    
    Serial.println("Manual");
  } else if (command == 'A') { // auto advance
    myState = Auto;
    Serial.println("Auto");

  } else if (command == 'x') { // channel settings
    printChannels();

  } else if ( command == 'i') { // system information
    printSystemInformation();

  } else if ( command == 'I') { // system information
    printPinInformation();

  } else if (command == 'e') { // read EEPROM
    EEPROM.get(eepromAddress, mySettings);
    if (mySettings.valid == 0xF0) { // apply settings because EEPROM has valid settings
      PWM_Frequency  = mySettings.PWM_Frequency;
      PWM_Resolution = mySettings.PWM_Resolution;
      PWM_MaxValue   = pow(2,PWM_Resolution)-1;  
      for (int i=0; i<NUMLEDS; i++) {
        LEDs[i]      = mySettings.LEDs[i];      
        LEDsEnable[i] = mySettings.LEDsEnable[i];
        DutyCycle[i] = mySettings.DutyCycle[i];        
      }
      myState        = mySettings.myState;
      Serial.println("EEPROM read.");
    } else { Serial.println("EEPROM settings not valid, not applied."); }
  } else if (command == 'E') { // saveEEPROM
    mySettings.valid           = 0xF0;  // make EEPROM settings valid
    mySettings.PWM_Frequency   = PWM_Frequency;
    mySettings.PWM_Resolution  = PWM_Resolution;
    for (int i=0; i<NUMLEDS; i++) {
      mySettings.LEDs[i]       = LEDs[i];      
      mySettings.LEDsEnable[i] = LEDsEnable[i];
      mySettings.DutyCycle[i]  = DutyCycle[i];        
    }
    mySettings.DutyCycle[NUMLEDS-1] = 0;
    mySettings.myState        = myState;
    EEPROM.put(eepromAddress, mySettings);
    Serial.println("Settings saved to EEPROM");

  } else if (command == 'm') { // turn off PWM
    // ENABLE/DISABLE PWM //////////////////////////////////////////////////////////
    if (isPWM(PWM_Pin)) {
      analogWrite(PWM_Pin, 0);
      PWM_Enabled = false;
      Serial.println("off");
    } else if (isSoftPWM(PWM_Pin)) {
      SoftPWMSet(PWM_Pin,  0);
      PWM_Enabled = false;
      Serial.println("off");
    } else {
      Serial.print(PWM_Pin);
      Serial.println(" is not a PWM pin.");
    }
  } else if (command == 'M') { // turn on PWM
    if (isPWM(PWM_Pin)) {
      analogWrite(PWM_Pin, (unsigned int)(PWM_Duty / 100.0 * float(PWM_MaxValue)));
      PWM_Enabled = true;
      Serial.println("on");
    } else if (isSoftPWM(PWM_Pin)) {
      SoftPWMSetPercent(PWM_Pin, (uint8_t)(PWM_Duty));
      PWM_Enabled = true;
      Serial.println("on");
    } else {
      Serial.print(PWM_Pin);
      Serial.println(" is not a PWM pin.");
    }

  } else if (command == 's') { // load duty and pin for channel
    // Load & Save Channel Settings////////////////////////////////////////////////
    tempInt = value.toInt();      
    if ((tempInt >=0) || (tempInt < NUMLEDS)) { // cehck boundaries      
      ch          = tempInt;
      PWM_Duty    = DutyCycle[ch];
      PWM_Pin     = LEDs[ch];
      PWM_Enabled = LEDsEnable[ch];
      Serial.printf("Current Channel: %2d\n", ch);
      Serial.printf("PWM pin:         %2d\n", PWM_Pin);
      Serial.printf("PWM Duty:        %4.3f\n", PWM_Duty);
      Serial.printf("PWM Frequency:   %f\n", PWM_Frequency);
      Serial.printf("Channel:         %s\n", PWM_Enabled?"Enabled":"Disabled");
      if (PWM_Enabled) {
        setupPWM(PWM_Pin, PWM_Frequency, PWM_Duty, PWM_Resolution);
      } else {
        if      (isPWM(PWM_Pin))     { analogWrite(PWM_Pin, 0); Serial.println("off"); }
        else if (isSoftPWM(PWM_Pin)) { SoftPWMSet(PWM_Pin,  0); Serial.println("off"); }
      }
    } else { Serial.println("Channel out of valid Range.");   }
  } else if (command == 'S') { // save duty cycle and enable/disable and pin into selected channel
      tempInt = value.toInt();      
      if ((tempInt >=0) || (tempInt < NUMLEDS)) {      
        ch = tempInt;
        DutyCycle[ch] = PWM_Duty;
        LEDs[ch]      = PWM_Pin;
        LEDsEnable[ch] = PWM_Enabled;       
      } else { Serial.println("Channel out of valid Range.");   }
      
  } else if (command == 'd') { // duty cycle
    // SET DUTY CYCLE /////////////////////////////////////////////////////////////
    tempFloat = value.toFloat();
    if ((tempFloat < 0.0) || (tempFloat > 100.0)) { // check boundaries
      Serial.println("Duty cyle out of valid Range.");
    } else {
      PWM_Duty = tempFloat;
      setupPWM(PWM_Pin, PWM_Frequency, PWM_Duty, PWM_Resolution);
      Serial.printf("Duty Cycle set to: %+4.3f\n", PWM_Duty);
    }

  } else if (command == 'f') { // frequency
    // SET Frequency //////////////////////////////////////////////////////////////
    float tempFloat = value.toFloat();
    if (isPWM(PWM_Pin)) {
      Serial.printf("Desired Frequency: %f\n", tempFloat);
      if (tempFloat <= GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
        PWM_Frequency = tempFloat;
        setupPWM(PWM_Pin, PWM_Frequency, PWM_Duty, PWM_Resolution);
        Serial.printf("Frequency set to: %f\n", PWM_Frequency);
      } else {
        Serial.println("Frequency to high. Not changed. Change Resolution first.");
      }
    } else if (isSoftPWM(PWM_Pin)) {
      Serial.print("Can not change frequency on soft PWM pin: ");
      Serial.println(PWM_Pin);
    }

  } else if (command == 'r') { // resolution of pulse width
    // SET Resolution ////////////////////////////////////////////////////////////
    tempInt = value.toInt();
    if (isPWM(PWM_Pin)) {
      if ((tempInt < 2) || (tempInt > 15)) { // check boundary
        Serial.println("PWM Resolution out of valid Range.");
      } else {
        PWM_Resolution = tempInt;
        PWM_MaxValue = pow(2, PWM_Resolution)-1;
        if (PWM_Frequency > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
          PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
        }
        setupPWM(PWM_Pin, PWM_Frequency, PWM_Duty, PWM_Resolution);
        Serial.printf("PWM Resolution set to: %2d\n", PWM_Resolution);
        Serial.printf("PWM Max Value: %5d\n", PWM_MaxValue);
        Serial.printf("PWM Frequency adjusted to: %f\n", PWM_Frequency);
      }
    } else if (isSoftPWM(PWM_Pin)) {
      Serial.printf("Can not change resolution on soft PWM pin: %2d\n", PWM_Pin);
    }

  } else if (command == 'p') { // choose pin
    // Choose the pin //////////////////////////////////////////////////////////////
    tempInt = (uint8_t)(value.toInt());
    if (isPWM(tempInt)) {
      PWM_Pin = tempInt;
      Serial.println("Setting the pin to current frequency, duty cycle and resolution");
      pinMode(PWM_Pin, OUTPUT);      
      setupPWM(PWM_Pin, PWM_Frequency, PWM_Duty, PWM_Resolution);
      Serial.printf("Changed PWM pin: %2d\n", PWM_Pin);
      Serial.printf("PWM Frequency: %f\n", PWM_Frequency);
      Serial.printf("PWM Duty: %4.3f\n", PWM_Duty);
      Serial.printf("PWM Resolution set to: %2d\n", PWM_Resolution);
      Serial.printf("PWM Max Value: %5d\n", PWM_MaxValue);
    } else if (isSoftPWM(tempInt)) {
      PWM_Pin = tempInt;
      pinMode(PWM_Pin, OUTPUT);
      SoftPWMSet(PWM_Pin, 0);
      SoftPWMSetPercent(PWM_Pin, PWM_Duty);
      Serial.printf("Changed Soft PWM pin: %2d\n", PWM_Pin);
      Serial.printf("PWM Duty: %4.3f\n", PWM_Duty);
    } else {
      Serial.println("Pin not available for PWM.");
    }

  } else if (command == '\n') { // ignore
    // Ignore Carriage Return //////////////////////////////////////////////////////////////

  } else if ((command == '?') || (command == 'h')) { // send HELP information
    // HELP //////////////////////////////////////////////////////////////
    printHelp();
  }
} // end process instruction
