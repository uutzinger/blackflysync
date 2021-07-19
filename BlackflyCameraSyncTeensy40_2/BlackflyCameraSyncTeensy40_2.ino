// *********************************************************************************************//
// Blacklfy Camera Sync with Teensy 4.x
// *********************************************************************************************//
//
// Description
// -----------
// This code uses a Teensy 4.x microcontroller to control an external light source for 
// Team 21050's Rapid Optical Imaging of Physiological Process (ROIPP)
// engineering senior design project for the University of Arizona (ENGR 498) 2021.
//
// Requirements
// ------------
// This sofware requires a Teensy 4.x Microcontroller.
//
// Setup
// -----
// The Camera Line 2 will need to be programmed to be the Frame Sync
//
// Operation
// ---------
// This code cycles through 13 LEDs (+1 spare), upon an external sync source from a Blackfly camera. 
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

#define VERSION "1.1.0"
#define EEPROM_VALID 0xF1
#include <iterator>
#include <algorithm>

// *********************************************************************************************//
// Teensy 4.0 Specifics
// PWM pins: 0,1,2,3,4,5,6,7,8,9,10,11,12,,13,14,15,18,19,22,23,24,25,28,29,33,34,35,36,37,38,39
// ADC pins: 14,15,16,17,18,19,20,21,22,23, 24,26,27
// SERIAL_TX and RX can be: 1,0  7,8  14,15  16,17  20,21  24,25  28,29  38,39 
// https://www.pjrc.com/teensy/pinout.html
// *********************************************************************************************//

// FlexPWM 4.482kHz
// QuadTimer 3.611kHz

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
#define LEDPIN           13 // buitin LED
// Hardware PWM Channels
#define PWM1             2  // Connect to LED driver channel 1 (output)
#define PWM2             3  // Connect to LED driver channel 2 (output)
#define PWM3             4  // Connect to LED driver channel 3 (output)
#define PWM4             5  // Connect to LED driver channel 4 (output)
#define PWM5             6  // Connect to LED driver channel 5 (output)
#define PWM6             7  // Connect to LED driver channel 6 (output)
#define PWM7             8  // Connect to LED driver channel 7 (output)
#define PWM8             9  // Connect to LED driver channel 8 (output)
#define PWM9             10 // Connect to LED driver channel 9 (output)
#define PWM10            11 // Connect to LED driver channel 10 (output)
#define PWM11            12 // Connect to LED driver channel 11 (output)
#define PWM12            14 // Connect to LED driver channel 12 (output)
#define PWM13            15 // Connect to LED driver channel 13 (output)
#define CLK              22 // Connect to LED driver CLOK (output)
#define BLANK            99 // Connect to LED driver virtual channel 99 (none)
#define CAMTRG           21 // Camera frame trigger, needs to go to pin 8 (input)
#define POWERSWITCH      20 // External Button (input)
// *********************************************************************************************//
// This depends on the hardware logic of the FET driver and FET of your circuit
#define TURN_ON  HIGH      // 
#define TURN_OFF LOW       // 
#define PWM_INV  true      // If true, high represents OFF

// *********************************************************************************************//
// LED channel configuration
// ------------------------------------------------------------------------
#define NUM_CHANNELS 14
volatile int             ch = 2;   // Start LED cycle off at LEDs[0]
volatile int         LEDs[] = {PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9, PWM10, PWM11, PWM12, PWM13, BLANK };// LEDs
volatile bool  LEDsEnable[] = {false, false, false, false, false, false, false, false, false, false,  false,  false,  false,  false};  // Is channel on or off=false
volatile float  LEDsInten[] = {5.,   5.,   5.,   5.,   5.,   5.,   5.,   5.,   5.,   5.,    5.,    5.,    5.,    5.};    // PWM in %

// *********************************************************************************************//
// Button Debounce
// ------------------------------------------------------------------------
volatile long  lastInterrupt; 

// *********************************************************************************************//
// EEPROM configuration
// ------------------------------------------------------------------------
#include <EEPROM.h>
#define EEPROM_SIZE 2048 // Max size
int eepromAddress = 0;   // Where storage starts
struct EEPROMsettings {                  // Setting structure
  byte         valid;                    // First time use of EEPROM?
  states       myState;                  // Whater to be in Autoadvance or Manual or Off Mode
  int          LEDs[NUM_CHANNELS];       // Which pins are the LEDs attached
  bool         LEDsEnable[NUM_CHANNELS]; // Is this LED channel used
  float        LEDsInten[NUM_CHANNELS];  // Is this LED brightness in %
  float        DutyCycle;                // Duty cycle (ie 10 = 10% duty cycle)
  float        PWM_Frequency;            // Lower frequency, higher resolution
  unsigned int PWM_Resolution;           // Check teensy specs, smaller resolution, higher PWM clock
  int          CameraTrigger;            // Input pin for trigger
  int          PowerSwitch;              // Input pin for on/off optional
  int          CLK_Pin;                  // Output pin for PWM signal
  };

EEPROMsettings mySettings; // the settings

// *********************************************************************************************//
// Intervals and Timing
// ------------------------------------------------------------------------
#define POLL_INTERVAL          10000     // desired main loop time in microseconds 
#define CHECKINPUT_INTERVAL    50000     // interval in microseconds between polling serial input
#define LEDON_INTERVAL        100000     // internal LED interval on in microseconds 
#define LEDOFF_INTERVAL       400000     // internal LED interval off in microseconds 
// *********************************************************************************************//
unsigned long pollInterval;              //
unsigned long lastInAvail;               //
unsigned long lastPoll;                  //
unsigned long currentTime;               //
unsigned long nextLEDCheck;              //

// *********************************************************************************************//
// Controlling Reporting and Input Output
// ------------------------------------------------------------------------
bool SERIAL_REPORTING = true;            // Boot messages on/off

// *********************************************************************************************//
// Indicator
// ------------------------------------------------------------------------
const int ledPin = LEDPIN;               // Check on https://www.pjrc.com/teensy/pinout.html; pin should not interfere with I2C and SPI
bool ledStatus = false;                  // Led should be off at start up

// *********************************************************************************************//
// Main System and PWM working variables
// ------------------------------------------------------------------------
unsigned int  Pin            = PWM1;          //
unsigned int  CLK_Pin        = CLK;           //
bool          PWM_Enabled    = false;         //
long          CPU_Frequency  = F_CPU / 1E6;   //
float         PWM_Frequency  = 4577.64;       // Ideal 15 bit and 600MHz Teensy 4.0
unsigned int  PWM_Resolution = 8;             // 
unsigned int  PWM_MaxValue   = pow(2, PWM_Resolution)-1;
float         DutyCycle      = 5.0;           //
int           CameraTrigger  = CAMTRG;        //
int           PowerSwitch    = POWERSWITCH;   //

// *********************************************************************************************//
// Serial
#define SERIAL_PORT_SPEED    2000000     // Serial Baud Rate
char   inBuff[] = "----------------";    // needs to match maximum length of a command
int    bytesread;

// *********************************************************************************************//
// *********************************************************************************************//
// SETUP                                                                                        //
// *********************************************************************************************//
// *********************************************************************************************//
void setup(){

  // Configure output pins, set them all to off/low
  pinMode(PWM1,        OUTPUT); digitalWriteFast(PWM1,  TURN_OFF);
  pinMode(PWM2,        OUTPUT); digitalWriteFast(PWM2,  TURN_OFF);
  pinMode(PWM3,        OUTPUT); digitalWriteFast(PWM3,  TURN_OFF);
  pinMode(PWM4,        OUTPUT); digitalWriteFast(PWM4,  TURN_OFF);
  pinMode(PWM5,        OUTPUT); digitalWriteFast(PWM5,  TURN_OFF);
  pinMode(PWM6,        OUTPUT); digitalWriteFast(PWM6,  TURN_OFF);
  pinMode(PWM7,        OUTPUT); digitalWriteFast(PWM7,  TURN_OFF);
  pinMode(PWM8,        OUTPUT); digitalWriteFast(PWM8,  TURN_OFF);
  pinMode(PWM9,        OUTPUT); digitalWriteFast(PWM9,  TURN_OFF);
  pinMode(PWM10,       OUTPUT); digitalWriteFast(PWM10, TURN_OFF);
  pinMode(PWM11,       OUTPUT); digitalWriteFast(PWM11, TURN_OFF);
  pinMode(PWM12,       OUTPUT); digitalWriteFast(PWM12, TURN_OFF);
  pinMode(PWM13,       OUTPUT); digitalWriteFast(PWM13, TURN_OFF);
  pinMode(CLK,         OUTPUT); if (PWM_INV) {digitalWriteFast(PWM13, HIGH);} else {digitalWriteFast(PWM13, LOW);}
  pinMode(ledPin,      OUTPUT); digitalWriteFast(ledPin,LOW ); ledStatus = false;

  //EEPROM
  EEPROM.get(eepromAddress, mySettings);
  if (mySettings.valid == EEPROM_VALID) { // apply settings because EEPROM has valid settings
    CameraTrigger  = mySettings.CameraTrigger;
    PowerSwitch    = mySettings.PowerSwitch;
    PWM_Frequency  = mySettings.PWM_Frequency;
    PWM_Resolution = mySettings.PWM_Resolution;
    PWM_MaxValue   = pow(2,PWM_Resolution)-1;  
    DutyCycle      = mySettings.DutyCycle;
    CLK_Pin        = mySettings.CLK_Pin;   
    for (int i=0; i<NUM_CHANNELS; i++) {
      LEDs[i]       = mySettings.LEDs[i];      
      LEDsEnable[i] = mySettings.LEDsEnable[i];
      LEDsInten[i]  = mySettings.LEDsInten[i];
    }
    myState        = mySettings.myState;
  } else {
    // create default values for settings
    PWM_Resolution = 8;
    PWM_Frequency  = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution); 
    PWM_MaxValue   = pow(2,PWM_Resolution)-1;
    unsigned int  tmp[NUM_CHANNELS] =  {PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9, PWM10, PWM11, PWM12, PWM13, BLANK };   
    DutyCycle      = 5.0;        
    for (int i=0; i<NUM_CHANNELS; i++) {
      LEDs[i]       = tmp[i];      
      LEDsEnable[i] = false; 
      LEDsInten[i]  = 5.;
    }
    myState        = Manual;
  }

  // Input Pins
  pinMode(CameraTrigger, INPUT_PULLUP);  // Frame trigger
  pinMode(PowerSwitch,   INPUT_PULLUP);  // initialize pin for power switch

  // Set PWM source
  setupPWM(CLK_Pin,  PWM_Frequency, DutyCycle, PWM_Resolution);

  // Serial startup
  Serial.begin(SERIAL_PORT_SPEED);
  if (SERIAL_REPORTING) {
    Serial.println("Starting System");
    printHelp();
  }

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(CameraTrigger),   frameISR, RISING); // Frame start
  attachInterrupt(digitalPinToInterrupt(PowerSwitch),        onOff, CHANGE); // Create interrupt on pin for power switch. Trigger when there is a change detected (button is pressed)

  // House keeping
  pollInterval = POLL_INTERVAL; // 1 m sec
  lastInAvail = lastPoll = lastInterrupt = nextLEDCheck = micros();

  printSystemInformation();

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
//  analgoWrite(pin,0) takes 2.5-3 microseconds on Teensy 3.2
//  analgoWrite(pin,some value > 0) takes 5-6 microseconds on Teensy 3.2
//
// Turns off current LED
// Checks if LED is enabled, otherwise skip
// Set intensity
// Turns on next LED
// If current LED is highest channel, do not turn on/off anything
//////////////////////////////////////////////////////////////////
void frameISR() {
  switch (myState)  {
    case Off:
      // do nothing
      break;
    case Auto:
      // Turn OFF current LED
      if (ch < NUM_CHANNELS-1) { digitalWriteFast(LEDs[ch], TURN_OFF); } 
      // increment channel
      ch = (ch + 1)%NUM_CHANNELS;
      while (LEDsEnable[ch] == false) {ch = (ch + 1)%NUM_CHANNELS;}  // search for the next active LED channel
      // Turn ON next LED
      if (ch < NUM_CHANNELS-1) { 
        if (PWM_INV) {
          // PWM signal high turns the LED off
          analogWrite(CLK_Pin, uint16_t((100.0-LEDsInten[ch]) / 100.0 * float(PWM_MaxValue)));
        } else {
          // PWM signal high turns the LED on
          analogWrite(CLK_Pin, uint16_t(LEDsInten[ch] / 100.0 * float(PWM_MaxValue)));
        digitalWriteFast(LEDs[ch],  TURN_ON); 
        }
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
      for (int i=0; i<NUM_CHANNELS-1; i++)  {                           // Turen off all PWM except last channel which is background
        digitalWriteFast(LEDs[i],  TURN_OFF);      
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

// Teensy 4.0 realtion between PWM frequency and PWM resolution
//////////////////////////////////////////////////////////////////
float GetMaxPWMFreqValue(long FREQ, int PWM_Resolution)
{
  /* for Teensy CPU frequencies are
      24MHZ
      396MHZ & 450MHZ
      528MHZ & 600MHZ
  */
  // FlexPWM 4.482kHz
  // QuadTimer 3.611kHz

  int FREQ_pointer = -1;
  float PWM_ideal_frequency[3][14]
  {
    { 6000000,  3000000, 1500000,  750000,  375000,  187500,  93750,    46875,     23437.5,  11718.75,  5859.375, 2929.687, 1464.843, 732.421},
    {33000000, 16500000, 8250000, 4125000, 2062500, 1031250, 515625,   257812.5,  128906.25, 64453.13, 32226.56, 16113.28,  8056.64, 4028.32 },
    {37500000, 18750000, 9375000, 4687500, 2343750, 1171875, 585937.5, 292968.75, 146484.38, 73242.19, 36621.09, 18310.55,  9155.27, 4577.64 }
  };

  switch (FREQ) {
    case 24:  FREQ_pointer =  0; break;
    case 396: FREQ_pointer =  1; break;
    case 450: FREQ_pointer =  2; break;
    case 528: FREQ_pointer =  1; break;
    case 600: FREQ_pointer =  2; break;
    default:  FREQ_pointer = -1; break;
  }
  if (FREQ_pointer >= 0) { return (PWM_ideal_frequency[FREQ_pointer][PWM_Resolution - 2]); } 
  else { return (4482); }
} // end of getMaxPWMFreqValue

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
      if (PWM_INV) {
        // PWM signal high turns the LED off
        analogWrite(PWM_Pin, uint16_t((100.0-Duty) / 100.0 * float(PWM_MaxValue)));
      } else {
        // PWM signal high turns the LED on
        analogWrite(PWM_Pin, uint16_t(Duty / 100.0 * float(PWM_MaxValue)));
      }
      DutyCycle = Duty;                          // update global
    }
  } 
}

// What are the software options?
//////////////////////////////////////////////////////////////////
void printHelp() {
  Serial.println("-------------------------------------------------");
  Serial.println("   BlackFly Camera to LED Illumination Sync");
  Serial.println("-------------------------------------------------");
  Serial.println("https://github.com/uutzinger/blackflysync  ");
  Serial.println("0 Disable Auto Advance: a");
  Serial.println("  f: set the PWM frequency");
  Serial.println("  d: set the duty cycle");
  Serial.println("  r: set resolution in bits");
  Serial.println("1 Load working channel from EEPROM: e.g. s0");
  Serial.println("2 Adjust working channel");
  Serial.println("  p: set the pin associated to working channel");
  Serial.println("  d: set intensity of working channel 0..100");
  Serial.println("  m/M: disable/enable the working channel");  
  Serial.println("3 Save working channel: S0");
  Serial.println("4 Enable Auto Advance: A");
  Serial.println("5 Save channel configurations to EEPROM: E");
  Serial.println("------- Data Input--------------------------------");
  Serial.println("p5   set current working pin to 5");
  Serial.println("d50  set duty cyle to 50%");
  Serial.println("m/M  disable/enable current working pin"); 
  Serial.println("f512 set frequency to 512Hz");
  Serial.println("r8   set PWM resolution to 8 bits");
  Serial.println("-------------------------------------------------");
  Serial.println("i/I  system information");
  Serial.println("x    print channel settings");
  Serial.println("-------------------------------------------------");
  Serial.println("s0   load channel 0 to current working settings");
  Serial.println("S0   save channel 0 from current working settings");
  Serial.println("save to EEPROM to make presistant");
  Serial.println("-------------------------------------------------");
  Serial.println("c21  set camera trigger to pin 21");
  Serial.println("C22  set PWM clock to pin 22");
  Serial.println("o20  set on/off button to pin 20, -1 = disabled");
  Serial.println("-------------------------------------------------");
  Serial.println("a/A  disable/enable Auto Advance"); 
  Serial.println("e/E  read/save settings to EEPROM");
  Serial.println("-------------------------------------------------");
  Serial.println("Shannon McCoy, Urs Utzinger, 2020-21");
  Serial.println("-------------------------------------------------");
}

void printSystemInformation() {
  Serial.println("-------------------------------------------------");
  Serial.printf( "Software version:     %s\n", VERSION);
  Serial.printf( "CPU:                  %2d MHz\n", CPU_Frequency);
  Serial.printf( "PWM pin:              %d\n", CLK_Pin);
  Serial.printf( "Frequency:            %f.1 Hz\n", PWM_Frequency);
  Serial.printf( "Current Duty:         %+4.3f percent\n", DutyCycle);
  Serial.printf( "Current Resolution:   %2d bit\n", PWM_Resolution);
  Serial.printf( "PWM Max:              %4d\n" , PWM_MaxValue);
  Serial.printf( "Camera trigger is on: %d\n", CameraTrigger);
  Serial.printf( "Power switch:         %d\n", PowerSwitch);
  Serial.println("-------------------------------------------------");
  Serial.print(  "State is:             ");
  if (myState == Off)    { Serial.println("Off"); }
  if (myState == Auto)   { Serial.println("Auto"); }
  if (myState == Manual) { Serial.println("Manual"); }
  Serial.println("-------------------------------------------------");
  Serial.printf( "Working on pin: %2d which is %s\n", Pin, PWM_Enabled?"on":"off");
  printChannels();
}

void printChannels() {
  Serial.println("-------------------------------------------------");
  for (int i=0; i<NUM_CHANNELS; i++) {
    Serial.printf( "Channel: %2d pin: %2d", i, LEDs[i]);
    Serial.printf( " Enabled: %s", LEDsEnable[i]?"Yes":"No"); 
    Serial.printf( " Duty: %f\n",  LEDsInten[i]); 
  }
}

void printPinInformation() {
  Serial.println("-------------------------------------------------");
  Serial.println("Maximum Values:");
  Serial.println("-------------------------------------------------");
  listFrequencies();
  listPins();
}

void listFrequencies()
{
  Serial.println("-------------------------------------------------");
  for(int i=2; i<=15; i++) {
    Serial.printf("Resolution: %2d bit, Frequency: %f\n", i, GetMaxPWMFreqValue(CPU_Frequency, i));
  }
  Serial.println("Resolution: 16 bit, Frequency: N.A.");
} 

boolean isPWM(uint8_t mypin) {
  const uint8_t last = 31; // number of pins -1
  const uint8_t pwmpins[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,18,19,22,23,24,25,28,29,33,34,35,36,37,38,39};
  return std::binary_search(pwmpins, &pwmpins[last], mypin);
}

boolean isIO(uint8_t mypin) {
  const uint8_t last = 38; // number of pins -1
  const uint8_t iopins[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39};
  return std::binary_search(iopins, &iopins[last], mypin);
}

void listPins() {
  char pinType[] = "Hard";
  Serial.println("Available Pins for PWM");
  Serial.println("-------------------------------------------------");
  for (int i=0; i<40; i++){
    if      (isPWM(i))     {strcpy(pinType, "PWM");}
    else if (isIO(i))      {strcpy(pinType, "DIO");}
    else                   {strcpy(pinType, "N.A.");}
    Serial.printf("Pin: %2d, %s\n", i, pinType );
  }
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
    
  } else if (command == 'c') { // canera trigger pin
    // Camera Trigger /////////////////////////////////////////////////////////////
    tempInt = value.toInt();
    if ((tempInt < 0) || (tempInt > 39)) { // check boundaries
      Serial.println("Trigger out of valid Range.");
    } else {
      detachInterrupt(digitalPinToInterrupt(CameraTrigger));
      CameraTrigger = tempInt;
      pinMode(CameraTrigger, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(CameraTrigger), frameISR, RISING); // Frame start
      Serial.printf("Camera Trigger attached to: %d\n", CameraTrigger);
    }

  } else if (command == 'C') { // pwm clock pin
    // PWM Clock /////////////////////////////////////////////////////////////
    tempInt = value.toInt();
    if ((tempInt < 0) || (tempInt > 39)) { // check boundaries
      Serial.println("PWM pin not valid.");
    } else {
      digitalWriteFast(CLK_Pin, TURN_OFF);
      CLK_Pin = tempInt;
      pinMode(CLK_Pin, OUTPUT);
      setupPWM(CLK_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);      
      Serial.printf("PWM Clock attached to: %d\n", CLK_Pin);
    }

  } else if (command == 'o') { // power button
    // Power Button /////////////////////////////////////////////////////////////
    tempInt = value.toInt();
    if ((tempInt < -1) || (tempInt > 39)) { // check boundaries
      Serial.println("Pin for Power Button is not valid.");
    } else {
      if (PowerSwitch >= 0) { detachInterrupt(digitalPinToInterrupt(PowerSwitch)); } // remove powerswitch interrupt only if not disabled
      PowerSwitch = tempInt;
      if (PowerSwitch >= 0) { // attach interrupt if powerswitch is not -1
        pinMode(PowerSwitch, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PowerSwitch), onOff, CHANGE); // Power Switch
      }
      Serial.printf("Power Switch attached to: %d\n", PowerSwitch);
    }
  
  } else if (command == 'x') { // channel settings
    printChannels();

  } else if ( command == 'i') { // system information
    printSystemInformation();

  } else if ( command == 'I') { // system information
    printPinInformation();

  } else if ( command == 'e') { // read EEPROM
    EEPROM.get(eepromAddress, mySettings);
    if (mySettings.valid == EEPROM_VALID) { // apply settings because EEPROM has valid settings
      CameraTrigger  = mySettings.CameraTrigger;
      PowerSwitch    = mySettings.PowerSwitch;
      PWM_Frequency  = mySettings.PWM_Frequency;
      PWM_Resolution = mySettings.PWM_Resolution;
      PWM_MaxValue   = pow(2,PWM_Resolution)-1;  
      DutyCycle      = mySettings.DutyCycle;
      CLK_Pin        = mySettings.CLK_Pin;   
      for (int i=0; i<NUM_CHANNELS; i++) {
        LEDs[i]      = mySettings.LEDs[i];      
        LEDsEnable[i]= mySettings.LEDsEnable[i];
        LEDsInten[i] = mySettings.LEDsInten[i];
      }
      myState        = mySettings.myState;
      Serial.println("EEPROM read.");
    } else { Serial.println("EEPROM settings not valid, not applied."); }

  } else if (command == 'E') { // saveEEPROM
    mySettings.CameraTrigger   = CameraTrigger;
    mySettings.PowerSwitch     = PowerSwitch;
    mySettings.valid           = EEPROM_VALID;  // make EEPROM settings valid
    mySettings.PWM_Frequency   = PWM_Frequency;
    mySettings.PWM_Resolution  = PWM_Resolution;
    mySettings.DutyCycle       = DutyCycle;   
    mySettings.CLK_Pin         = CLK_Pin;     
    for (int i=0; i<NUM_CHANNELS; i++) {
      mySettings.LEDs[i]       = LEDs[i];      
      mySettings.LEDsEnable[i] = LEDsEnable[i];
      mySettings.LEDsInten[i]  = LEDsInten[i];
    }
    mySettings.myState        = myState;
    EEPROM.put(eepromAddress, mySettings);
    Serial.println("Settings saved to EEPROM");

  } else if (command == 'm') { // turn on/off
    // ENABLE/DISABLE  //////////////////////////////////////////////////////////
    if (isIO(Pin) || (Pin == 99)) {
      digitalWriteFast(Pin,  TURN_OFF);
      PWM_Enabled = false;
      Serial.printf("Pin %d is off\n", Pin);
    } else {
      Serial.printf("Pin %d is not a DIO pin.\n", Pin);
    }

  } else if (command == 'M') { // turn on PWM
    if (isIO(Pin) || (Pin==99)) {
      digitalWriteFast(Pin,  TURN_ON);
      PWM_Enabled = true;
      Serial.printf("Pin %d is on\n", Pin);
    } else {
      Serial.printf("Pin %d is not a DIO pin.\n", Pin);
    }

  } else if (command == 's') { // load duty and pin for channel
    // Load & Save Channel Settings////////////////////////////////////////////////
    tempInt = value.toInt();      
    if ((tempInt >=0) || (tempInt < NUM_CHANNELS)) { // cehck boundaries      
      ch          = tempInt;
      Pin         = LEDs[ch];
      PWM_Enabled = LEDsEnable[ch];
      DutyCycle   = LEDsInten[ch];
      Serial.printf("Current Channel: %2d\n",   ch);
      Serial.printf("Pin:             %2d\n",   Pin);
      Serial.printf("PWM Duty:        %4.3f\n", DutyCycle);
      Serial.printf("PWM Frequency:   %f\n",    PWM_Frequency);
      Serial.printf("Channel:         %s\n",    PWM_Enabled?"Enabled":"Disabled");
      if (PWM_Enabled) {
        if      (isIO(Pin))     { digitalWriteFast(LEDs[ch],  TURN_ON);}
      } else {
        if      (isIO(Pin))     { digitalWriteFast(LEDs[ch],  TURN_OFF);}
      }
    } else { Serial.println("Channel out of valid Range.");   }
  } else if (command == 'S') { // save duty cycle and enable/disable and pin into selected channel
      tempInt = value.toInt();      
      if ((tempInt >=0) || (tempInt < NUM_CHANNELS)) {      
        ch = tempInt;
        LEDs[ch]       = Pin;
        LEDsEnable[ch] = PWM_Enabled;       
        LEDsInten[ch]  = DutyCycle;       
      } else { Serial.println("Channel out of valid Range.");   }
      
  } else if (command == 'd') { // duty cycle
    // SET DUTY CYCLE /////////////////////////////////////////////////////////////
    tempFloat = value.toFloat();
    if ((tempFloat < 0.0) || (tempFloat > 100.0)) { // check boundaries
      Serial.println("Duty cyle out of valid Range.");
    } else {
      DutyCycle = tempFloat;
      setupPWM(CLK_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
      Serial.printf("Duty Cycle set to: %+4.3f\n", DutyCycle);
    }

  } else if (command == 'f') { // frequency
    // SET Frequency //////////////////////////////////////////////////////////////
    float tempFloat = value.toFloat();
    if (isPWM(CLK_Pin)) {
      Serial.printf("Desired Frequency: %f\n", tempFloat);
      if (tempFloat <= GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
        PWM_Frequency = tempFloat;
        setupPWM(CLK_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
        Serial.printf("Frequency set to: %f\n", PWM_Frequency);
      } else {
        Serial.println("Frequency to high. Not changed. Change Resolution first.");
      }
    }

  } else if (command == 'r') { // resolution of pulse width modulation
    // SET Resolution ////////////////////////////////////////////////////////////
    tempInt = value.toInt();
    if (isPWM(CLK_Pin)) {
      if ((tempInt < 2) || (tempInt > 15)) { // check boundary
        Serial.println("PWM Resolution out of valid Range.");
      } else {
        PWM_Resolution = tempInt;
        PWM_MaxValue = pow(2, PWM_Resolution)-1;
        if (PWM_Frequency > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
          PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
        }
        setupPWM(CLK_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
        Serial.printf("PWM Resolution set to: %2d\n", PWM_Resolution);
        Serial.printf("PWM Max Value: %5d\n", PWM_MaxValue);
        Serial.printf("PWM Frequency adjusted to: %f\n", PWM_Frequency);
      }
    } else {
      Serial.printf("Pin %d is not capable of PWM\n", CLK_Pin);
    }

  } else if (command == 'p') { // choose pin
    // Choose the pin //////////////////////////////////////////////////////////////
    tempInt = (uint8_t)(value.toInt());
    if (isIO(tempInt)) {
      Pin = tempInt;
      pinMode(Pin, OUTPUT);
      if (PWM_Enabled) { digitalWriteFast(Pin,  TURN_ON); } else { digitalWriteFast(Pin, TURN_OFF); }
      Serial.printf("Changeing pin: %2d\n", Pin);
      Serial.printf("Pin is: %s\n", PWM_Enabled ? "on" : "off");
    } else {
      Serial.println("Pin not available.");
    }

  } else if (command == '\n') { // ignore
    // Ignore Carriage Return //////////////////////////////////////////////////////////////
    printHelp();

  } else if ((command == '?') || (command == 'h')) { // send HELP information
    // HELP //////////////////////////////////////////////////////////////
    printHelp();
  }
} // end process instruction
