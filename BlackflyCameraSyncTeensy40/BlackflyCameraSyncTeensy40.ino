// *********************************************************************************************//
// Blacklfy Camera Sync with Teensy 4.x
// *********************************************************************************************//
//
// Description
// -----------
// This program controls a custom LED light source. It uses the camera frame sync to advance the active
// LED in a sequence of LED lights. It uses PWM to adjust the LED intensity. 
// It is designed to work at high frame rates at the limits of the microcontroller.
//
// Requirements
// ------------
// - Teensy 4.x Microcontroller.
// - Camera with digital frame sync.
// - Custom LED driver (power FET & FET driver)
//
// Setup
// -----
// To setup with a Blackfly camera, the Output Line (Line 2, red) will need to be programmed 
// to represent the Frame Sync.  The best configuration is with inverted logic, meaning signal high
// indicates no light exposure and low that the senor is colleccting light.
// Check https://github.com/uutzinger/camera for python programs to setup the blackfly camera.
//
// Operation
// ---------
// This code cycles through up to 13 LEDs (+ background). You can modify the code for more LEDs.
// The camera provides a trigger on its GPIO line, letting the teensy know that the sensor is 
// exposed to light. 
// The intensity of the lightsource is adjusted  using PWM  modulation. This digital modulation
// can be used to drive the gate of LED power electronics.
//
// Shanon McCoy, Urs Utzinger, February, 2021-2022, Tucson Arizona, USA
//
// *********************************************************************************************//
// Copyright (c) 2021, 2022 Urs Utzinger, Shannon McCoy
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

#define VERSION "1.9.2"
#define EEPROM_VALID 0xF2
#include <iterator>
#include <algorithm>

// *********************************************************************************************//
// Teensy 4.0 Specifications from https://www.pjrc.com/teensy/pinout.html:
// PWM pins: 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,18,19,22,23,24,25,28,29,33,34,35,36,37,38,39
// ADC pins: 14,15,16,17,18,19,20,21,22,23,24,26,27
// SERIAL_TX and RX can be: 1,0  7,8  14,15  16,17  20,21  24,25  28,29  38,39 
// FlexPWM   4.482kHz default frequency
// QuadTimer 3.611kHz default frequency
// *********************************************************************************************//

// *********************************************************************************************//
// ISR States:
// ------------------------------------------------------------------------
// Autoadvance: Autoadvance to next channel when external Frame Trigger occurs
// Manual:      User is adjusting a single channel
// ------------------------------------------------------------------------
volatile bool AutoAdvance = false;            // Auto (true) or Manual
// 

// ------------------------------------------------------------------------
// Controlling Reporting and Input Output
// ------------------------------------------------------------------------
bool SERIAL_REPORTING = false;                // Boot messages and interrupt reporting on/off

// ------------------------------------------------------------------------
// Pin Names:
// ------------------------------------------------------------------------
// The pin and channel associations can be changed during runtime and stored to EEPROM.
// These are default values.
#define LEDPIN          13                    // Teensy buit-in LED
// Hardware PWM Channels
#define CH1              2                    // Connect to LED driver channel 1             (output)
#define CH2              3                    // Connect to LED driver channel 2             (output)
#define CH3              4                    // Connect to LED driver channel 3             (output)
#define CH4              5                    // Connect to LED driver channel 4             (output)
#define CH5              6                    // Connect to LED driver channel 5             (output)
#define CH6              7                    // Connect to LED driver channel 6             (output)
#define CH7              8                    // Connect to LED driver channel 7             (output)
#define CH8              9                    // Connect to LED driver channel 8             (output)
#define CH9             10                    // Connect to LED driver channel 9             (output)
#define CH10            11                    // Connect to LED driver channel 10            (output)
#define CH11            12                    // Connect to LED driver channel 11            (output)
#define CH12            14                    // Connect to LED driver channel 12            (output)
#define CH13            15                    // Connect to LED driver channel 13            (output)
#define PWM             22                    // Connect to LED driver CLOCK                 (output)
#define BACKGND         99                    // Connect to LED driver virtual channel 99    (does not exist)
#define CAMTRG          21                    // Camera frame trigger, needs to go to pin 21 (input)
#define POWERSWITCH     -1                    // External Button (input), not conencted = -1 (input)

// *********************************************************************************************//
// The follwoing adjusts for the hardware logic of the LED power driver:
// Some driver FETs have a gate and enable/disable logic. 
// We use common PWM on all gates to adjust for LED intensity. 
// The logic on enable/disable or on the gate and be high or low for enable.
#define TURN_ON  HIGH                         // Adjust if you have inverted logic for ON/OFF
#define TURN_OFF LOW                          // Adjust if you have inverted logic for ON/OFF
#define PWM_INV  true                         // If true, high on PWM pin represents OFF

// You can advance to the next LED right after exposure is completed or within first few miroseconds
// after camera starts exposing. 
// Ideal is to advance after current exposure, so that correct LED is illuminating sensor during frame trigger.
// With an inverted FRAME_EXPOSURE signal that would be the rising edge of the trigger signal
#define INTERRUPTTRIGGER RISING               // Can be RISING or FALLING
// Oscillations on the frame trigger input can occur when channel output changes large currents.
// These oscillations erroneously advance the LED seqeunce. 
// The oscillations last a few micro seconds and are in the 10MHz range.
// Debouncing prevents triggers to be recognized for the amount of microseconds specified;
// With a frame rate of 500 Hz we have a frame trigger every 2000 micro seconds,
// therore the debounce block should be less than 2 ms long. 
// Adjust this value according your frame rate.
#define DEBOUNCEBLOCK 1800                    // in microseconds

// ------------------------------------------------------------------------
// LED channel configuration
// for fewer or more channels, adjust this section
// ------------------------------------------------------------------------
#define NUM_CHANNELS 14
#define BACKGROUND_CHANNEL 13
// volatile declaration is needed since variables are accessed or changed during interrupt service routine
unsigned int                     chWorking = 0;   // current working channel for manual changes
volatile int                     chCurrent = 0;   // Start LED cycke at LED[0]
volatile int            LEDs[NUM_CHANNELS] = {CH1,  CH2,  CH3,  CH4,  CH5,  CH6,  CH7,  CH8,  CH9,  CH10,  CH11,  CH12,  CH13,  BACKGND }; // LEDs
volatile bool     LEDsEnable[NUM_CHANNELS] = {false, false, false, false, false, false, false, false, false, false,  false,  false,  false,  false};  // Is channel on or off=false
float              LEDsInten[NUM_CHANNELS] = {5.,    5.,    5.,    5.,    5.,    5.,    5.,    5.,    5.,    5.,     5.,     5.,     5.,     5.};     // PWM in %
volatile int      LEDsIntenI[NUM_CHANNELS] = {12,    12,    12,    12,    12,    12,    12,    12,    12,    12,     12,     12,     12,     12};     // PWM in raw numbers, max depends on PWM resolution
char             LEDsName[NUM_CHANNELS][6] = {"365  ","460  ","525  ","590  ","623  ","660  ","740  ","850  ","950  ","1050 ","WHITE","280  ","420  ","BGND "};

//////////////////////////////////////////////////////////////////////////////////////////////////
// You should not need to change values below                                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////

// ------------------------------------------------------------------------
// Button and Frame Trigger ISR
// ------------------------------------------------------------------------
volatile long lastInterrupt;                 // keep time when interrup occured 
volatile bool myPreviousAdvance = false;
volatile unsigned int frameTriggerOccurred = 0;
volatile unsigned long lastTriggerTime = 0;

// ------------------------------------------------------------------------
// EEPROM configuration
// ------------------------------------------------------------------------
#include <EEPROM.h>
#define EEPROM_SIZE 2048                      // Max size
int eepromAddress = 0;                        // Where storage starts
struct EEPROMsettings {                       // Setting structure
  byte         valid;                         // First time use of EEPROM?
  int          LEDs[NUM_CHANNELS];            // Which pins are the LEDs attached
  bool         LEDsEnable[NUM_CHANNELS];      // Is this LED channel used
  float        LEDsInten[NUM_CHANNELS];       // Is this LED brightness in %
  float        DutyCycle;                     // Duty cycle (ie 10 = 10% duty cycle)
  float        PWM_Frequency;                 // Lower frequency, higher resolution
  unsigned int PWM_Resolution;                // Check teensy specs, smaller resolution, higher PWM clock
  int          CameraTrigger;                 // Input pin for trigger
  int          PowerSwitch;                   // Input pin for on/off optional
  int          PWM_Pin;                       // Output pin for PWM signal
  bool         AutoAdvance;                   // Frame trigger turns on next LED
  char         LEDsName[NUM_CHANNELS][6];
  };

EEPROMsettings mySettings; // the settings

// ------------------------------------------------------------------------
// Intervals and Timing
// ------------------------------------------------------------------------
//#define POLL_INTERVAL           1000          // desired main loop time in microseconds 
#define CHECKINPUT_INTERVAL     1000          // interval in microseconds between polling serial input
#define LEDON_INTERVAL        100000          // internal LED on time in microseconds 
#define LEDOFF_INTERVAL       900000          // internal LED off time in microseconds 
unsigned long lastInAvail;                    //
unsigned long loopStartTime;                  //
unsigned long nextLEDCheck;                   //
unsigned long lastFrameTrigger;               // detect timeout for auto shut off
bool Stopped = false;                         //

// ------------------------------------------------------------------------
// Indicator
// ------------------------------------------------------------------------
bool ledStatus = false;                       // Built-in Led should be off at start up

// ------------------------------------------------------------------------
// Main working variables
// ------------------------------------------------------------------------
int           Pin            = CH1;           // The pin we manually change, we can change at runtime
volatile int  PWM_Pin        = PWM;           // The pin we use for the FET Gate, we can change at runtime
bool          PWM_Enabled    = false;         // Turn on/off PWM, we can change at runtime
long          CPU_Frequency  = F_CPU / 1E6;   // Set at run time, depends on Teensy.
float         PWM_Frequency  = 4577.64;       // Ideal 15 bit and 600MHz Teensy 4.0, we can change at runtime
unsigned int  PWM_Resolution = 8;             // The current resolution, we can change at runtime.
unsigned int  PWM_MaxValue   = pow(2, PWM_Resolution)-1; // Computed internally
float         DutyCycle      = 5.0;           // The current dutycyle, we can change at runtime
int           CameraTrigger  = CAMTRG;        // The current input trigger, we can change at runtime
int           PowerSwitch    = POWERSWITCH;   // The current on/off switch, we can change at runtime

// ------------------------------------------------------------------------
// Serial
#define SERIAL_PORT_SPEED    2000000          // Serial Baud Rate, teensy ignores baudrate and uses max speed
char   inBuff[] = "------------------------"; // needs to match maximum length of a command
int    bytesread;
unsigned long laserSerialInputTimeout;        // last time we received serial input command

// *********************************************************************************************//
// *********************************************************************************************//
// SETUP                                                                                        //
// *********************************************************************************************//
// *********************************************************************************************//
void setup(){

  // EEPROM
  if (loadSettings() == false) { applyDefaultSettings(); }

  // Configure output pins, set them all to off/low
  for (int i=0; i<NUM_CHANNELS; i++) {
    if ( isIO(LEDs[i]) ) {
		  pinMode(LEDs[i],   OUTPUT); 														
		  digitalWrite(LEDs[i], TURN_OFF);
    }
  }
  pinMode(PWM,           OUTPUT); if (PWM_INV) {digitalWrite(PWM, HIGH);} else {digitalWrite(PWM, LOW);}
  pinMode(LEDPIN,        OUTPUT); digitalWrite(LEDPIN,LOW ); ledStatus = false;

  // Configure input pins
  pinMode(CameraTrigger, INPUT_PULLUP);  // Frame trigger
  if ( isIO(PowerSwitch) ) pinMode(PowerSwitch,   INPUT_PULLUP); // initialize pin for power switch
 
  // Setup PWM on PWM pin
  setupPWM(PWM_Pin,  PWM_Frequency, DutyCycle, PWM_Resolution);

  // Serial startup
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.setTimeout(20000); // give up to 20secs to complete serial input
  if (SERIAL_REPORTING) {
    Serial.println("Starting System.");
    printHelp();
  }

  // Interrupts
  //
  // Frame Trigger, required
  attachInterrupt(digitalPinToInterrupt(CameraTrigger), frameISR, INTERRUPTTRIGGER); // Frame start
  // Power Button, if button is not an existing pin, dont enable this, e.g. pin=-1
  if ( isIO(PowerSwitch) ) { attachInterrupt(digitalPinToInterrupt(PowerSwitch), onOff, CHANGE); } // Create interrupt on pin for power switch. Trigger when there is a change detected (button is pressed)

  // House keeping
  lastInAvail = lastInterrupt = nextLEDCheck = micros();
  // Display system information and help
  if (SERIAL_REPORTING) {
    printSystemInformation();
    printHelp();
  }

} // end setup

// *********************************************************************************************//
// *********************************************************************************************//
// Main LOOP                                                                                    //
// *********************************************************************************************//
// *********************************************************************************************//

void loop(){
  
  // Time Keeper
  //////////////////////////////////////////////////////////////////
  loopStartTime = micros();                                         // whats the time
 
  // Input Commands
  //////////////////////////////////////////////////////////////////
  if ((loopStartTime - lastInAvail) >= CHECKINPUT_INTERVAL) {
    lastInAvail = loopStartTime;
    if ( Serial.available() ) {
      bytesread = Serial.readBytesUntil('\n', inBuff, sizeof(inBuff)); // Read from serial until CR is read or timeout exceeded
      // while (Serial.available() > 0) { Serial.read(); }             // clear remainig characters
      inBuff[bytesread] = '\0';
      processInstruction();
    }
  }
  
  // Blink built in LED for status report
  //////////////////////////////////////////////////////////////////
  if ( frameTriggerOccurred > 0 ) {                                 // blink with trigger
    ledStatus = !ledStatus;
    digitalWriteFast(LEDPIN, ledStatus ? HIGH : LOW);               // change LED on/off
    nextLEDCheck = loopStartTime + LEDON_INTERVAL;
    if (SERIAL_REPORTING) {Serial.printf("Triggered! %u times, Channel: %i, Intensity %d, %s.\r\n",frameTriggerOccurred, chCurrent, LEDsIntenI[chCurrent], LEDsEnable[chCurrent]?"on":"off");}
    frameTriggerOccurred = 0;                                       // reset frametrigger
    lastFrameTrigger = loopStartTime;
    Stopped = false;
  } else {                                                          // regular blinking with LEDON and LEDOFF timing
    if ( loopStartTime > nextLEDCheck ) {
      if ( ledStatus ) {
        // LED is ON
        ledStatus = false; 
        digitalWriteFast(LEDPIN, ledStatus ? HIGH : LOW);           // turn off
        nextLEDCheck = loopStartTime + LEDOFF_INTERVAL;
      } else {
        // LED is OFF
        ledStatus = true;
        digitalWriteFast(LEDPIN, ledStatus ? HIGH : LOW);           // turn on
        nextLEDCheck = loopStartTime + LEDON_INTERVAL;
      }
    }
  }

  // Auto Turn OFF 
  //////////////////////////////////////////////////////////////////
  // If autoadvance enabled and if no frame trigger detected 
  // for 5 seconds turn off all LEDs
  if ( AutoAdvance && (Stopped==false) ) {
    if ( (loopStartTime - lastFrameTrigger) > 5000000 ) {           // 5 seconds frametrigger timeout
      for (int i=0; i<NUM_CHANNELS; i++) {
        if ( (LEDsEnable[i]==true) && isIO(LEDs[i]) ) { digitalWrite(LEDs[i],TURN_OFF); }     
      }
      Stopped = true;
      if ( SERIAL_REPORTING ) { Serial.println("Turned off all LEDs."); }
    }
  }
  
  // Main loop delay. Limits how often this loop is running
  //////////////////////////////////////////////////////////////////
  // Not sure if this is helpful: 
  // delayMicroseconds on other platforms does not call yield function.
  // yield is needed by some platforms to service the operating system.
  // delay usually includes a call to yield. 
  // on some platforms one should not delay for more than 100ms.
  // this theoretically limits mainloop access to CPU
  //long myDelay = (POLL_INTERVAL - (long)micros() - (long)currentTime ); // how long do we need to wait?
  //if ( myDelay > 0 ) {
  //  delayMicroseconds((unsigned int)myDelay);  // wait
  //}
} 

// *********************************************************************************************//
// Interrupt Service Routines                                                                   //
// *********************************************************************************************//

// Frame Trigger Interrupt Service Routine
// 
//  analgoWrite(pin,0)              takes 2.5-3 microseconds on Teensy 3.2
//  analgoWrite(pin,some value > 0) takes   5-6 microseconds on Teensy 3.2
//
// 1. Turns off current LED
// 2. Increment to next LED
// 3. Check if LED is enabled, otherwise skip
// 4. Set intensity
// 5. Turns on LED, if current LED is background channel, do not turn on/off

//////////////////////////////////////////////////////////////////
void frameISR() {
  if ( AutoAdvance ) {
    unsigned long triggerTime = micros(); // Teensy provides timing information in ISR
    if ( (triggerTime - lastTriggerTime) > DEBOUNCEBLOCK ) { // Debounce
      // 1) Turn OFF previous LED, skip the background channel
      if (LEDs[chCurrent] != BACKGND) { digitalWriteFast(LEDs[chCurrent], TURN_OFF); } 
      // 2) Increment channel
      chCurrent += 1; 
      if ( chCurrent >= NUM_CHANNELS ) {chCurrent = 0;}
      // Continue incrementing if channel is disabled
      while ( LEDsEnable[chCurrent] == false ) { 
        chCurrent += 1; 
        if ( chCurrent >= NUM_CHANNELS ) { chCurrent = 0; }
      }
      // 3) Turn ON next LED, skip background channel
      if ( LEDs[chCurrent] != BACKGND ) { 
        analogWrite(PWM_Pin, LEDsIntenI[chCurrent]); // set intensity with pwm pin 
        digitalWrite(LEDs[chCurrent], TURN_ON); // turn on enable pin
      }
      frameTriggerOccurred += 1;  // signal to main loop
      lastTriggerTime = triggerTime;
    }
  }
}

// Button ISR, toggles On/Off state of system
//
// turns off all LEDs for Off state
// Debounces 20ms
//////////////////////////////////////////////////////////////////
void onOff() {
  unsigned long onOffTime = micros();
  if ( onOffTime - lastInterrupt > 20000 ) {                      // debounce condition, 20ms
    if (AutoAdvance) {
      myPreviousAdvance = AutoAdvance;                            // keep track if state was manual or auto
      AutoAdvance = false;
      for (int i=0; i<NUM_CHANNELS; i++)  {                       // Turen off all PWM except last channel which is background
        if ( LEDs[i] != BACKGND ) { digitalWriteFast(LEDs[i],  TURN_OFF); }
      }
    } else {
      AutoAdvance = myPreviousAdvance;                            // switch back to auto or manual
    }
    lastInterrupt = onOffTime;                                    // keep track of time for debouncing
  }  
}

// *********************************************************************************************//
// Support Functions                                                                            //
// *********************************************************************************************//

// Teensy 4.0 relation between PWM frequency and PWM resolution
///////////////////////////////////////////////////////////////
//https://www.pjrc.com/teensy/td_pulse.html
float GetMaxPWMFreqValue(long FREQ, unsigned int PWM_Resolution)
{
  /* for Teensy CPU frequencies are
      24MHZ
      396MHZ & 450MHZ
      528MHZ & 600MHZ
      FlexPWM default frequency is 4.482kHz https://www.pjrc.com/teensy/td_pulse.html
      QuadTimer defaulf frequency is 3.611kHz https://www.pjrc.com/teensy/td_pulse.html
  */

  int FREQ_pointer = -1;
  float PWM_ideal_frequency[3][14]
  {   // 2 bit, 3 bit ... 15 bit
    { 6000000,  3000000, 1500000,  750000,  375000,  187500,  93750,    46875,     23437.5,  11718.75,  5859.375, 2929.687, 1464.843, 732.421}, //      24 MHz
    {33000000, 16500000, 8250000, 4125000, 2062500, 1031250, 515625,   257812.5,  128906.25, 64453.13, 32226.56, 16113.28,  8056.64, 4028.32 }, // 528,396 MHz
    {37500000, 18750000, 9375000, 4687500, 2343750, 1171875, 585937.5, 292968.75, 146484.38, 73242.19, 36621.09, 18310.55,  9155.27, 4577.64 }  // 600,450 MHz 
  };

  switch (FREQ) {
    case 24:  FREQ_pointer =  0; break;
    case 396: FREQ_pointer =  1; break;
    case 450: FREQ_pointer =  2; break;
    case 528: FREQ_pointer =  1; break;
    case 600: FREQ_pointer =  2; break;
    default:  FREQ_pointer = -1; break;
  }
  if (FREQ_pointer >= 0) { return (PWM_ideal_frequency[FREQ_pointer][(int)PWM_Resolution - 2]); } 
  else { return (4482); } // 4.482kHz
} // end of getMaxPWMFreqValue

// Setup PWM modulation on a pin
// This might be slow and should only be used for setup and not to change dutycyle
//////////////////////////////////////////////////////////////////
void setupPWM(int PWM_Pin, float PWM_Freq, float Duty, unsigned int PWM_Resolution) {
  if (isPWM(PWM_Pin)) {
    if ( (PWM_Resolution >= 2) && (PWM_Resolution <= 15) ) {  // check bounds
      analogWriteResolution(PWM_Resolution);  // change resolution
      PWM_MaxValue = pow(2, PWM_Resolution)-1;  // update global
    }
    if (PWM_Freq > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
      PWM_Freq = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
    }
    analogWriteFrequency(PWM_Pin, PWM_Freq);
    PWM_Frequency = PWM_Freq; // update global
    if ( (Duty > ((1./float(PWM_MaxValue))*100.)) && (Duty < 100.0) ) {    
      if ( PWM_INV ) {
        // PWM signal high turns the LED off
        analogWrite(PWM_Pin, int((100.0-Duty) / 100.0 * float(PWM_MaxValue)));
      } else {
        // PWM signal high turns the LED on
        analogWrite(PWM_Pin, int(Duty / 100.0 * float(PWM_MaxValue)));
      }
      DutyCycle = Duty;                          // update global
    } else if ( Duty <= (1./float(PWM_MaxValue))*100. ) {
        pinMode(PWM_Pin, OUTPUT);
        if (PWM_INV) { digitalWrite(PWM_Pin,HIGH); } else { digitalWrite(PWM_Pin,LOW); }
        DutyCycle = Duty;                          // update global
    } else if ( Duty >= 100.0 ) {
        pinMode(PWM_Pin, OUTPUT);
      if ( PWM_INV ) { digitalWrite(PWM_Pin,LOW); } else { digitalWrite(PWM_Pin,HIGH); }
      DutyCycle = Duty;                          // update global
    }
  } 
}

// What are the software options?
//////////////////////////////////////////////////////////////////
void printHelp() {
  Serial.println("---------------------------------------------------------");
  Serial.println("BlackFly Camera to LED Sync: Help");
  Serial.println("https://github.com/uutzinger/blackflysync  ");
  Serial.println("---------------------------------------------------------");
  Serial.println("Work Flow:");
  Serial.println("0 Disable Auto Advance: a");
  Serial.println("  f: set the PWM frequency");
  Serial.println("  d: set the duty cycle");
  Serial.println("  r: set resolution in bits");
  Serial.println("1 Load working channel from EEPROM: e.g. s0");
  Serial.println("2 Adjust working channel:");
  Serial.println("  p: set the pin associated to working channel");
  Serial.println("  d: set intensity of working channel 0..100");
  Serial.println("3 Save working channel: e.g. S0");
  Serial.println("4 Enable channel in the autoadvance array: e.g. M1");  
  Serial.println("5 Enable Auto Advance: A (pls verify all settings first)");
  Serial.println("6 Save channel configurations to EEPROM: E");
  Serial.println("---------------------------------------------------------");
  Serial.println("------- Data Input --------------------------------------");
  Serial.println("p5    set current working pin to 5");
  Serial.println("d50   set duty cyle to 50%");
  Serial.println("m/M   disable/enable current working channel"); 
  Serial.println("      (all channels except current working channel");
  Serial.println("       will be turned off)"); 
  Serial.println("m2/M2 disable/enable channel 2"); 
  Serial.println("t2    enter name for channel 2");
  Serial.println("f512  set PWM frequency to 512Hz");
  Serial.println("r8    set PWM resolution to 8 bits");
  Serial.println("------- Information -------------------------------------");
  Serial.println("i/I  system information");
  Serial.println("x    print all channel settings");
  Serial.println("x1   print channel 1 settings");
  Serial.println("------- Load & Save -------------------------------------");
  Serial.println("s0   load channel 0 to current working settings");
  Serial.println("S0   save channel 0 from current working settings");
  Serial.println("save to EEPROM with E to make presistent");
  Serial.println("------- Pins --------------------------------------------");
  Serial.println("c21  set camera trigger to pin 21");
  Serial.println("C22  set PWM clock to pin 22");
  Serial.println("o20  set on/off button to pin 20, -1 = disabled");
  Serial.println("------- Utils -------------------------------------------");
  Serial.println("a/A  disable/enable Auto Advance"); 
  Serial.println("e/E  read/save settings to EEPROM");
  Serial.println("Z    turn off all channels");
  Serial.println("---------------------------------------------------------");
  Serial.println("Shannon McCoy, Devesh Khosla, Urs Utzinger, 2020-22");
  Serial.println("---------------------------------------------------------");
}

// What are we working on right now?
void printSystemInformation() {
  Serial.println("---------------------------------------------------------");
  Serial.println("System Status");
  Serial.println("---------------------------------------------------------");
  Serial.printf( "Software version:     %s\r\n",        VERSION);
  Serial.printf( "CPU:                  %d MHz\r\n",    CPU_Frequency);
  Serial.printf( "PWM pin:              %d\r\n",        PWM_Pin);
  Serial.printf( "Frequency:            %.2f Hz\r\n",   PWM_Frequency);
  Serial.printf( "Current Duty:         %.2f [%%]\r\n", DutyCycle);
  Serial.printf( "Current Resolution:   %d bit\r\n",    PWM_Resolution);
  Serial.printf( "PWM Max:              %d\r\n",        PWM_MaxValue);
  Serial.printf( "Camera trigger is on: %d\r\n",        CameraTrigger);
  Serial.printf( "Power switch:         %d\r\n",        PowerSwitch);
  Serial.println("---------------------------------------------------------");
  Serial.printf( "State is:             %s\r\n",        AutoAdvance?"Auto":"Manual");
  Serial.println("---------------------------------------------------------");
  Serial.printf( "Working on LED %s:\r\nChannel: %2d pin: %2d %s\r\n", \
                  LEDsName[chWorking], chWorking, Pin, PWM_Enabled?"On ":"Off" );
  printChannels();
}

// What are the settings for all channels?
void printChannels() {
  Serial.println("---------------------------------------------------------");
  Serial.println("Auto Advance Sequence:");
  for (int i=0; i<NUM_CHANNELS; i++) {
    printChannel(i);
  }
}

void printChannel(int i) {
  Serial.printf( "Channel: %2d pin: %2d", i, LEDs[i]);
  Serial.printf( " %s",                      LEDsEnable[i]?"On ":"Off"); 
  Serial.printf( " %6.2f[%%] duty",          LEDsInten[i]); 
  Serial.printf( " [%4d]",                   LEDsIntenI[i]); 
  Serial.printf( " Name: %s\r\n",            LEDsName[i]); 
}

// What can we choose from?
void printPinInformation() {
  Serial.println("---------------------------------------------------------");
  Serial.println("Maximum Values:");
  listFrequencies();
  listPins();
}

// What are the max frequencies for each resolution?
void listFrequencies()
{
  for(int i=2; i<=15; i++) {
    Serial.printf("Resolution: %2d bit, Frequency: %11.2f\r\n", \
                   i, GetMaxPWMFreqValue(CPU_Frequency, (unsigned int) i));
  }
  Serial.println("Resolution: 16 bit, Frequency:        N.A.");
} 

boolean isPWM(int mypin) {
  //https://www.pjrc.com/teensy/td_pulse.html
  const int last = 31; // number of pins -1
  const int pwmpins[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,18,19,\
                         22,23,24,25,28,29,33,34,35,36,37,38,39};
  return std::binary_search(pwmpins, &pwmpins[last], mypin);
}

boolean isIO(int mypin) {
  const int last = 38; // number of pins -1
  const int iopins[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,\
                        19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,\
                        35,36,37,38,39};
  return std::binary_search(iopins, &iopins[last], mypin);
}

void listPins() {
  char pinType[] = "Hard";
  Serial.println("---------------------------------------------------------");
  Serial.println("Available Pins for PWM and DIO:");
  for (int i=0; i<40; i++){
    if      ( isPWM(i) )     {strcpy(pinType, "PWM");}
    else if ( isIO(i) )      {strcpy(pinType, "DIO");}
    else                     {strcpy(pinType, "N.A.");}
    Serial.printf("Pin: %2d, %s\n", i, pinType );
  }
}

// SerialCommand handler
//////////////////////////////////////////////////////////////////

void processInstruction() {
  float  tmpF;
  int    tmpI;
  char   instruction[2];
  char   value[64];
  char   *pEnd;
  bool   tempAutoAdvance;

  pEnd = strchr (inBuff, '\r');       // search for carriage return
  if (pEnd) { *pEnd = '\0'; }         // if found truncate

  if (strlen(inBuff) > 0) {           // process input if input buffer is not empty
    strncpy(instruction, inBuff, 1); instruction[1]='\0';  // no null char appended when using strncpy
    if (strlen(inBuff) > 1) { 
      strncpy(value, inBuff+1, sizeof(value));
    } else {
      value[0] = '\0'; 
    }

    //Serial.print("Instruction: "); Serial.println(instruction);
    //Serial.print("Value:   "); Serial.println(value);        

    if        ( instruction[0] == 'a' ) { // manual mode
      // ENABLE/DISABLE Autodavance based on Frame Trigger////////////////////////////
      AutoAdvance = false;    
      Serial.println("Mode: Manual.");

    } else if ( instruction[0] == 'A' ) { // auto advance
      AutoAdvance = true;
      Serial.println("Mode: Auto Advance.");
      
    } else if ( instruction[0] == 'c' ) { // canera trigger pin
      // Camera Trigger /////////////////////////////////////////////////////////////
      if (strlen(value)>0) {
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI < 0) || (tmpI > 39)) { // check boundaries
          Serial.println("Trigger out of valid range.");
        } else {
          detachInterrupt(digitalPinToInterrupt(CameraTrigger));
          CameraTrigger = tmpI;
          pinMode(CameraTrigger, INPUT_PULLUP);
          attachInterrupt(digitalPinToInterrupt(CameraTrigger), frameISR, RISING); // Frame start
          Serial.printf("Camera Trigger attached to: %d.\r\n", CameraTrigger);
        }
      }

    } else if ( instruction[0] == 'C' ) { // pwm clock pin
      // PWM Clock /////////////////////////////////////////////////////////////
      if (strlen(value)>0) {
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI < 0) || (tmpI > 39)) { // check boundaries
          Serial.println("PWM pin not valid.");
        } else {
          digitalWrite(PWM_Pin, TURN_OFF);
          PWM_Pin = tmpI;
          pinMode(PWM_Pin, OUTPUT);
          setupPWM(PWM_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);      
          Serial.printf("PWM Clock attached to: %d.\r\n", PWM_Pin);
        }
      }

    } else if ( instruction[0] == 'o' ) { // power button
      // Power Button /////////////////////////////////////////////////////////////
      if (strlen(value)>0) { 
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI < -1) || (tmpI > 39)) { // check boundaries
          Serial.println("Pin for Power Button is not valid.");
        } else {
          if ( isIO(PowerSwitch) ) { detachInterrupt(digitalPinToInterrupt(PowerSwitch)); }
          PowerSwitch = tmpI;
          if ( isIO(PowerSwitch) ) { // attach interrupt if powerswitch is not -1
            pinMode(PowerSwitch, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(PowerSwitch), onOff, CHANGE); // Power Switch
          }
          Serial.printf("Power Switch attached to: %d.\r\n", PowerSwitch);
        }
      }
    
    } else if ( instruction[0] == 'x' ) { // channel settings
      if (strlen(value)>0) { 
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI >=0) || (tmpI < NUM_CHANNELS)) { // check boundaries
           printChannel(tmpI);
        }
      } else { 
        printChannels();
      }

    } else if ( instruction[0] == 'i' ) { // system information
      printSystemInformation();

    } else if ( instruction[0] == 'I' ) { // system information
      printPinInformation();

    } else if ( instruction[0] == 'e' ) { // read EEPROM
      if (loadSettings()) { 
        Serial.println("EEPROM read.");
      } else { 
        Serial.println("EEPROM settings not valid, not applied."); 
      }

    } else if ( instruction[0] == 'E' ) { // saveEEPROM
      saveSettings();
      Serial.println("Settings saved to EEPROM.");

    } else if ( instruction[0] == 'm' ) { // turn off
      // ENABLE/DISABLE  //////////////////////////////////////////////////////////
      if (strlen(value)>0) {
        tmpI = int(strtol(value, NULL, 10));
        LEDsEnable[tmpI]  = false;
        Serial.printf("Channel %d, is off.\r\n", tmpI);
      } else {                        // adjust only current working channle/pin
        if (isIO(Pin)) {
          digitalWrite(Pin, TURN_OFF); // turn off current working channel
          Serial.printf("Channel %d, is off.\r\n", chWorking);
        } else {
          Serial.printf("Enable flag set to off. No changes to hardware as pin %d is not a DIO pin.\r\n", Pin);
        }
        PWM_Enabled = false;
      }

    } else if ( instruction[0] == 'M' ) { // turn on
      if (strlen(value)>0) {          // adjust LED seetings
        tmpI = int(strtol(value, NULL, 10));
        LEDsEnable[tmpI] = true;
        Serial.printf("Channel %d, is on.\r\n", tmpI);
      } else {                        // adjust only current working channel/pin
        setupPWM(PWM_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
        if ( isIO(Pin) ) {
          for (int i=0; i<NUM_CHANNELS-1; i++) { if (isIO(LEDs[i])) { digitalWriteFast(LEDs[i],TURN_OFF); } } // turn off all channels, can not have two LEDs on at same time
          digitalWrite(Pin,  TURN_ON);
          Serial.printf("Pin %d is on.\n", Pin);
        } else {
          Serial.printf("Enable flag set to on. No changes to hardware as pin %d is not a DIO pin.\r\n", Pin);
        }
        PWM_Enabled = true;
      }

    } else if ( instruction[0] == 's' ) {
      // Load & Save Channel Settings////////////////////////////////////////////////
      if (strlen(value)>0) {          // adjust LED seetings
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI >=0) && (tmpI < NUM_CHANNELS)) { // check boundaries      
          chWorking       = tmpI;
          Pin             = LEDs[chWorking];
          PWM_Enabled     = LEDsEnable[chWorking];
          DutyCycle       = LEDsInten[chWorking];
          Serial.printf("Current Channel:     %2d\r\n",   chWorking);
          Serial.printf("Pin:                 %2d\r\n",   Pin);
          Serial.printf("PWM Duty:            %5.2f\r\n", DutyCycle);
          Serial.printf("PWM Frequency: %11.2f\r\n",      PWM_Frequency);
          Serial.printf("Channel:       %s\r\n",          PWM_Enabled?" Enabled":"Disabled");
        } else { 
          Serial.println("Channel out of valid range.");   
        }
      }

    } else if ( instruction[0] == 'S' ) { // save duty cycle and pin into selected channel
      if (strlen(value)>0) {          // adjust LED seetings
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI >=0) && (tmpI < NUM_CHANNELS)) {
          chWorking             = tmpI;
          LEDs[chWorking]       = Pin;
          LEDsEnable[chWorking] = PWM_Enabled;
          LEDsInten[chWorking]  = DutyCycle;
          if (isIO(LEDs[chWorking])) {
            if (PWM_INV) { LEDsIntenI[chWorking] = int(( 100.0 - DutyCycle) / 100. * float(PWM_MaxValue) ); }
            else         { LEDsIntenI[chWorking] = int(          DutyCycle  / 100. * float(PWM_MaxValue) ); }
          } else         { LEDsIntenI[chWorking] = int(0); }
          Serial.printf("Channel %d saved.\r\n",chWorking);
        } else { 
          Serial.printf("Channel %d out of valid range.\r\n",chWorking);
        }
      }
        
    } else if ( instruction[0] == 't' ) {
      // Set Channel Name ///////////////////////////////////////////////////////////
      if (strlen(value)>0) {          // adjust LED seetings
        tmpI = int(strtol(value, NULL, 10));
        if ((tmpI >=0) && (tmpI < NUM_CHANNELS)) { // check boundaries      
          // Obtain new name
          while (Serial.available() > 0) { Serial.read(); } // clear remainig characters
          Serial.printf("Enter new Name for Channel %d (up to 5 characters):\r\n",  tmpI);
          // Serial.setTimeout(20000); // give up to 20secs, already set at startup
          bytesread = Serial.readBytesUntil('\n', inBuff, sizeof(inBuff));  // Read from serial until CR is read or timeout exceeded
          inBuff[bytesread] = '\0';
          if (strlen(inBuff) > 0) {
            strncpy(LEDsName[tmpI],inBuff,6);
            Serial.printf("Channel %d Name: %s.\r\n", tmpI, LEDsName[tmpI] );
          } else {
            Serial.println("Name too short.");
          }
        } else { Serial.println("Channel # out of valid range.");   }
      }

    } else if ( instruction[0] == 'd' ) { // duty cycle
      // SET DUTY CYCLE /////////////////////////////////////////////////////////////
      if (strlen(value)>0) {          // adjust LED seetings
        tmpF = strtof(value, NULL);
        if ((tmpF < 0.0) || (tmpF > 100.0)) { // check boundaries
          Serial.println("Duty cyle out of valid range.");
        } else {
          DutyCycle = tmpF;
          setupPWM(PWM_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
          Serial.printf("Duty Cycle set to: %+4.3f.\r\n", DutyCycle);
        }
      }

    } else if ( instruction[0] == 'f' ) { // frequency
      // SET Frequency //////////////////////////////////////////////////////////////
      if (strlen(value)>0) {          // adjust LED seetings
        tmpF = strtof(value, NULL);
        if (isPWM(PWM_Pin)) {
          Serial.printf("Desired frequency: %10.2f.\r\n", tmpF);
          if (tmpF <= GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
            PWM_Frequency = tmpF;
            setupPWM(PWM_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
            Serial.printf("Frequency set to: %10.2f.\r\n", PWM_Frequency);
          } else {
            Serial.println("Frequency too high. Not changed. Change resolution first.");
          }
        }
      }

    } else if ( instruction[0] == 'r' ) { // resolution of pulse width modulation
      // SET Resolution ////////////////////////////////////////////////////////////
      if (strlen(value)>0) {          // adjust LED seetings
        tmpI = int(strtol(value, NULL, 10));
        if ( isPWM(PWM_Pin) ) {
          if ((tmpI < 2) || (tmpI > 15)) { // check boundary
            Serial.println("PWM Resolution out of valid range.");
          } else {
            PWM_Resolution = (unsigned int)(tmpI);
            PWM_MaxValue = pow(2, PWM_Resolution)-1;
            if (PWM_Frequency > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
              PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
            }
            setupPWM(PWM_Pin, PWM_Frequency, DutyCycle, PWM_Resolution);
            Serial.printf("PWM Resolution set to: %2d.\r\n", PWM_Resolution);
            Serial.printf("PWM max value: %5d.\r\n", PWM_MaxValue);
            Serial.printf("PWM Frequency adjusted to: %10.2f.\r\n", PWM_Frequency);
          }
        } else {
          Serial.printf("Pin %d is not capable of PWM.\r\n", PWM_Pin);
        }
      }

    } else if ( instruction[0] == 'p' ) { // choose pin
      // Choose the pin //////////////////////////////////////////////////////////////
      if (strlen(value)>0) {          // adjust LED seetings
        tmpI = int(strtol(value, NULL, 10));
        if ( isIO(tmpI) ) {
          Pin = tmpI;
          pinMode(Pin, OUTPUT);
          if (PWM_Enabled) { 
            for (int i=0; i<NUM_CHANNELS-1; i++) { if (isIO(LEDs[i])) { digitalWriteFast(LEDs[i],TURN_OFF); } } // turn all pins off
            digitalWrite(Pin,  TURN_ON); } // turn new pin on
          else { 
            digitalWrite(Pin, TURN_OFF);  // turn new pin off
          }
          Serial.printf("Changeing pin: %2d.\r\n", Pin);
          Serial.printf("Pin is: %s.\r\n", PWM_Enabled ? "on" : "off");
        } else {
          Serial.println("Pin not available.");
        }
      }

    } else if ( instruction[0] == 'Z' ) { 
      for (int i=0; i<NUM_CHANNELS-1; i++) { if (isIO(LEDs[i])) { digitalWriteFast(LEDs[i],TURN_OFF); } } 
      Serial.println("Turned all channels off.");

    } else if ( (instruction[0] == '?') || (instruction[0] == 'h') ) { // send HELP information
      // HELP //////////////////////////////////////////////////////////////
      printHelp();
    } else { 
      Serial.println("Unknown instruction.");
    }
    
    value[0] = '\0'; 
    instruction[0] = '\0'; 

  } // inputbuffer is not empty
} // end process instruction

boolean loadSettings() {
  EEPROM.get(eepromAddress, mySettings);
  // apply settings t variabales
  if (mySettings.valid == EEPROM_VALID) { // apply settings because EEPROM has valid settings
    CameraTrigger  = mySettings.CameraTrigger;
    PowerSwitch    = mySettings.PowerSwitch;
    PWM_Frequency  = mySettings.PWM_Frequency;
    PWM_Resolution = mySettings.PWM_Resolution;
    PWM_MaxValue   = pow(2,PWM_Resolution)-1;  
    DutyCycle      = mySettings.DutyCycle;
    PWM_Pin        = mySettings.PWM_Pin;   
    for (int i=0; i<NUM_CHANNELS; i++) {
      LEDs[i]       = mySettings.LEDs[i];      
      LEDsEnable[i] = mySettings.LEDsEnable[i];
      LEDsInten[i]  = mySettings.LEDsInten[i];
      if (PWM_INV) { LEDsIntenI[i] = int(( 100.0 - LEDsInten[i]) / 100. * float(PWM_MaxValue) ); }
      else         { LEDsIntenI[i] = int(          LEDsInten[i]  / 100. * float(PWM_MaxValue) ); }
      strncpy(LEDsName[i], mySettings.LEDsName[i], 6);
    }
    AutoAdvance     = mySettings.AutoAdvance;
    return true;
  } else {
    return false;
  }
}

void applyDefaultSettings() {
    // create default values for settings
    PWM_Resolution = 8;
    PWM_Frequency  = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution); 
    PWM_MaxValue   = pow(2,PWM_Resolution)-1;
    const int tmp[NUM_CHANNELS] =  {CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, BACKGND };   
    DutyCycle      = 5.0;        
    for (int i=0; i<NUM_CHANNELS; i++) {
      LEDs[i]       = tmp[i];      
      LEDsEnable[i] = false; 
      LEDsInten[i]  = 5.;
      LEDsIntenI[i] = (int) (LEDsInten[i]  / 100. * float(PWM_MaxValue));
      strcpy(LEDsName[i], "UNKN");
    }
    AutoAdvance     = false;
}


void saveSettings() {
    mySettings.CameraTrigger   = CameraTrigger;
    mySettings.PowerSwitch     = PowerSwitch;
    mySettings.valid           = EEPROM_VALID;  // make EEPROM settings valid
    mySettings.PWM_Frequency   = PWM_Frequency;
    mySettings.PWM_Resolution  = PWM_Resolution;
    mySettings.DutyCycle       = DutyCycle;   
    mySettings.PWM_Pin         = PWM_Pin;
    bool valid=false;
    for (int i=0; i<NUM_CHANNELS; i++) {
      valid = valid || LEDsEnable[i];
    }
    if ( !valid ) {
      LEDsEnable[NUM_CHANNELS-1] = true;
      Serial.println("Turned on background channel to enable at least one channel.");
    } // There needs to be at least one channel enabled, otherwise ISR will get stuck
    for (int i=0; i<NUM_CHANNELS; i++) {
      mySettings.LEDs[i]       = LEDs[i];      
      mySettings.LEDsEnable[i] = LEDsEnable[i];
      mySettings.LEDsInten[i]  = LEDsInten[i];
      strncpy(mySettings.LEDsName[i], LEDsName[i], 6);
    }
    mySettings.AutoAdvance     = AutoAdvance;
    EEPROM.put(eepromAddress, mySettings);
}
