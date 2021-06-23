// *********************************************************************************************//
// Teensy Analog Write Test
// *********************************************************************************************//
//
// Description
// -----------
//
// Author
// ------
// Urs Utzinger, February 2021, Tucson Arizona, USA
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

#include <iterator>
#include <algorithm>

// *********************************************************************************************//
// Pin Names:
// ------------------------------------------------------------------------
#define LEDPIN           13 // buitin LED
// Hardware PWM Channels
#define PWM1             3  // Connect to LED driver channel 1 (outut)
#define PWM2             4  // Connect to LED driver channel 2 (outut)
#define PWM3             5  // Connect to LED driver channel 3 (outut)
#define PWM4             6  // Connect to LED driver channel 4 (outut)
// *********************************************************************************************//


// *********************************************************************************************//
// Intervals and Timing
// ------------------------------------------------------------------------
#define POLL_INTERVAL         100000    // desired main loop time in microseconds 
#define LEDON_INTERVAL        300000    // interval in microseconds between turning LED on/off for status report
#define LEDOFF_INTERVAL       700000    // interval in microseconds between turning LED on/off for status report
// *********************************************************************************************//
unsigned long currentTime;                     //
unsigned long pollInterval;
unsigned long lastPoll;
unsigned long nextLEDCheck;                    //

// *********************************************************************************************//
// Indicator
// ------------------------------------------------------------------------
const int ledPin = LEDPIN;                    // Check on https://www.pjrc.com/teensy/pinout.html; pin should not interfere with I2C and SPI
bool ledStatus = false;                       // Led should be off at start up


// *********************************************************************************************//
// Main System and PWM working variables
// ------------------------------------------------------------------------
unsigned int  PWM_Pin        = PWM1;           //
bool          PWM_Enabled    = false;          //
long          CPU_Frequency  = F_CPU / 1E6;    //
float         PWM_Frequency  = 4577.64;        // Ideal 15 bit and 600MHz Teensy 4.0
unsigned int  PWM_Resolution = 8;              // 
unsigned int  PWM_MaxValue   = pow(2, PWM_Resolution)-1;
float         PWM_Duty       = 0.0;           //

float analogWriteTime10 = 0.;
float analogWriteTime20 = 0.;
float analogWriteTime30 = 0.;
float analogWriteTime40 = 0.;
float analogWriteTime11 = 0.;
float analogWriteTime21 = 0.;
float analogWriteTime31 = 0.;
float analogWriteTime41 = 0.;


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

  // Configure output pins, set them all to off/low
  pinMode(PWM1,        OUTPUT); digitalWriteFast(PWM1,  LOW);
  pinMode(PWM2,        OUTPUT); digitalWriteFast(PWM2,  LOW);
  pinMode(PWM3,        OUTPUT); digitalWriteFast(PWM3,  LOW);
  pinMode(PWM4,        OUTPUT); digitalWriteFast(PWM4,  LOW);
  pinMode(ledPin,      OUTPUT); digitalWriteFast(ledPin,LOW ); 
  ledStatus = false;

  // create default values for settings
  PWM_Resolution = 8;
  //PWM_Frequency  = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution); 
  PWM_Frequency  = 50000.0; 
  PWM_MaxValue   = pow(2,PWM_Resolution)-1;

  // PWM
  // Start PWM at 0% output
  // void setupPWM(uint16_t PWM_Pin, float PWM_MaxFreq, float PWM_Duty, unsigned int PWM_Resolution);
  setupPWM(PWM1,  PWM_Frequency, 5.0, PWM_Resolution);
  setupPWM(PWM2,  PWM_Frequency, 5.0, PWM_Resolution);
  setupPWM(PWM3,  PWM_Frequency, 5.0, PWM_Resolution);
  setupPWM(PWM4,  PWM_Frequency, 5.0, PWM_Resolution);

  // Serial startup
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println("Starting System");

  // House keeping
  pollInterval = POLL_INTERVAL; // 1 m sec
  lastPoll = nextLEDCheck = micros();

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


  PWM_Duty = PWM_Duty + 1.0;
  if (PWM_Duty > 100.0) {PWM_Duty =0.0;}
  
  currentTime = micros();
  analogWrite(PWM1, uint16_t(0)); // Turn off the current LED takes 2.5-3 micro seconds
  analogWriteTime10 = 0.9* analogWriteTime10 + 0.1*float(micros()-currentTime);
  currentTime = micros();
  analogWrite(PWM2, uint16_t(0)); // Turn off the current LED
  analogWriteTime20 = 0.9* analogWriteTime20 + 0.1*float(micros()-currentTime);
  currentTime = micros();
  analogWrite(PWM3, uint16_t(0)); // Turn off the current LED
  analogWriteTime30 = 0.9* analogWriteTime30 + 0.1*float(micros()-currentTime);
  currentTime = micros();
  analogWrite(PWM4, uint16_t(0)); // Turn off the current LED
  analogWriteTime40 = 0.9* analogWriteTime40 + 0.1*float(micros()-currentTime);

  currentTime = micros();
  analogWrite(PWM1, uint16_t(PWM_Duty*PWM_MaxValue/100.0)); // Turn off the current LED takes 5-6 micro seconds
  analogWriteTime11 = 0.9* analogWriteTime11 + 0.1*float(micros()-currentTime);
  currentTime = micros();
  analogWrite(PWM2, uint16_t(PWM_Duty*PWM_MaxValue/100.0)); // Turn off the current LED
  analogWriteTime21 = 0.9* analogWriteTime21 + 0.1*float(micros()-currentTime);
  currentTime = micros();
  analogWrite(PWM3, uint16_t(PWM_Duty*PWM_MaxValue/100.0)); // Turn off the current LED
  analogWriteTime31 = 0.9* analogWriteTime31 + 0.1*float(micros()-currentTime);
  currentTime = micros();
  analogWrite(PWM4, uint16_t(PWM_Duty*PWM_MaxValue/100.0)); // Turn off the current LED
  analogWriteTime41 = 0.9* analogWriteTime41 + 0.1*float(micros()-currentTime);
  
  // Blink LED
  //////////////////////////////////////////////////////////////////
  if (currentTime > nextLEDCheck) {
    Serial.printf("1: %f %f\n", analogWriteTime10, PWM_Duty);
    Serial.printf("2: %f\n", analogWriteTime20);
    Serial.printf("3: %f\n", analogWriteTime30);
    Serial.printf("4: %f\n", analogWriteTime40);
    Serial.printf("1: %f %f\n", analogWriteTime11, PWM_Duty);
    Serial.printf("2: %f\n", analogWriteTime21);
    Serial.printf("3: %f\n", analogWriteTime31);
    Serial.printf("4: %f\n", analogWriteTime41);
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

// Setup PWM modulation on a pin
// This is slow and should only be used for setup and not to change dutycyle
//////////////////////////////////////////////////////////////////
void setupPWM(uint16_t PWM_Pin, float PWM_Freq, float Duty, unsigned int Resolution) {
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
}
