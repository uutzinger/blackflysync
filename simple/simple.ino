// *********************************************************************************************//
// Blacklfy Camera Sync with Teensy 4.x
// *********************************************************************************************//

// *********************************************************************************************//
// Pin Names:
// ------------------------------------------------------------------------
#define NUM_CHANNELS      3 //
#define CH1               2 // Connect to LED driver channel 1 (output)
#define CH2               3 // Connect to LED driver channel 2 (output)
#define CH_BG            -1 // all LEDs off
#define CLK              22 // Connect to LED driver PWM CLK (output)
#define CAMTRG           21 // Camera frame trigger, needs to go to pin 8 (input)
#define LEDPIN           13 // Built in LED
#define TURN_ON  HIGH       // 
#define TURN_OFF LOW        // 

// *********************************************************************************************//
// LED channel configuration
// ------------------------------------------------------------------------

volatile int LEDs[NUM_CHANNELS] = {CH1,  CH2, CH_BG }; // LEDs
volatile int     currentChannel = 0;

#define       PWM_Frequency  50000
#define       PWM_Resolution     8   
#define       PWM_MaxValue     255
#define       DutyCycle        5.0  

volatile bool frameTriggerOccured = false;
volatile bool ledStatus = false;

// *********************************************************************************************//
// SETUP                                                                                        //
// *********************************************************************************************//
void setup(){

  // Configure output pins, set them all to off/low
  pinMode(CH1,        OUTPUT); digitalWriteFast(CH1,   TURN_OFF);
  pinMode(CH2,        OUTPUT); digitalWriteFast(CH2,   TURN_OFF);
  pinMode(CLK,        OUTPUT); digitalWriteFast(CLK,   HIGH); 
  pinMode(LEDPIN,     OUTPUT); digitalWriteFast(LEDPIN,LOW ); 

  // Input Pins
  pinMode(CAMTRG, INPUT_PULLUP);  // Frame trigger

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(CAMTRG),   frameISR, RISING); // Frame start
 
  // set PWM clk oputput
  analogWriteResolution(PWM_Resolution);    // change resolution
  analogWriteFrequency(CLK, PWM_Frequency);
  analogWrite(CLK, uint16_t((100.0-DutyCycle) / 100.0 * float(PWM_MaxValue)));
} // end setup

// *********************************************************************************************//
// *********************************************************************************************//
// Main LOOP                                                                                    //
// *********************************************************************************************//
// *********************************************************************************************//

void loop(){
    
  // Blink LED
  //////////////////////////////////////////////////////////////////
  if (frameTriggerOccured) {
    ledStatus = bool(~ledStatus);
    digitalWriteFast(LEDPIN, ledStatus); // blink
    frameTriggerOccured = false; // reset signal
  }

} 

//////////////////////////////////////////////////////////////////
void frameISR() {
  // Turn OFF previous LED, skip the background channel
  if (LEDs[currentChannel] != CH_BG) { digitalWriteFast(LEDs[currentChannel], TURN_OFF); } 
  // Increment channel
  currentChannel++; if (currentChannel >= NUM_CHANNELS) {currentChannel = 0;}
  // Continue incrementing if channel is disabled
  // Turn ON next LED, skip background channel
  if (LEDs[currentChannel] != CH_BG) { 
    digitalWriteFast(LEDs[currentChannel], TURN_ON); // turn on enable pin
  }
  frameTriggerOccured = true;  // signal to main loop
}
