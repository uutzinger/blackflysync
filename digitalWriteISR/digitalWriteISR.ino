#define BAUDRATE 115200      // Serial communicaiton speed

// *********************************************************************************************//
// Pin Settings
// ------------------------------------------------------------------------
#define NUM_CHANNELS       3 // Number of channels
#define CH1                2 // Connect pin 2 to channel 1 (output)
#define CH2                3 // Connect pin 3 to channel 2 (output)
#define CH_BG             -1 // Connect no pin to channel 3  
#define CLK               22 // Connect pin 22 to CLK (output)
#define TRG               21 // Connect pin 21 to trigger (input)
#define LEDPIN            13 // Built in LED
#define TURN_ON  HIGH        // Define Enable Channel
#define TURN_OFF LOW         // Define Disable Channel

// PWM Settings
// ------------------------------------------------------------------------
#define PWM_Frequency      5
#define PWM_Resolution     8   
#define PWM_MaxValue     255
#define DutyCycle        5.0  

// Variables
// ------------------------------------------------------------------------
volatile int LEDs[NUM_CHANNELS] = {CH1, CH2, CH_BG }; // LED channel array
volatile int currentChannel = 0;                      // Index to current LED in LED array
volatile bool triggerOccurred = false;                 // Signal to main loop
bool ledStatus = false;                               // Blinking of the built in LED

// *********************************************************************************************//
// SETUP                                                                                        //
// *********************************************************************************************//
void setup(){

  // Configure output pins, set them all to off/low
  pinMode(CH1,    OUTPUT); digitalWrite(CH1,   TURN_OFF);
  pinMode(CH2,    OUTPUT); digitalWrite(CH2,   TURN_OFF);
  pinMode(CLK,    OUTPUT); digitalWrite(CLK,   HIGH); 
  pinMode(LEDPIN, OUTPUT); digitalWrite(LEDPIN,LOW ); 

  // Configure input pins
  pinMode(TRG,    INPUT_PULLUP);  // trigger

  // Configure Interrupts
  attachInterrupt(digitalPinToInterrupt(TRG), myISR, RISING);
 
  // Configure PWM output
  analogWriteResolution(PWM_Resolution);                               // change resolution
  analogWriteFrequency(CLK, PWM_Frequency);                            // set PWM frequency on CLK pin
  analogWrite(CLK, uint16_t(DutyCycle / 100.0 * float(PWM_MaxValue))); // set Duty Cyle and enable PWM on CLK pin (i have inverted PWM logic)

  // Start Serial IO
  Serial.begin(BAUDRATE);
  Serial.println("System started");
} // end setup

// *********************************************************************************************//
// Main LOOP                                                                                    //
// *********************************************************************************************//

void loop(){
    
  // Blink LED if ISR was called
  // ------------------------------------------------------------------------
  if (triggerOccurred) {
    ledStatus = !ledStatus;
    digitalWriteFast(LEDPIN, ledStatus); // blink
    triggerOccurred = false;             // reset signal
    Serial.println(currentChannel);      // debug
  }

} 

// *********************************************************************************************//
// Support Functions                                                                            //
// *********************************************************************************************//

void myISR() {
  // Turn OFF previous channel, skip the background channel
  if (LEDs[currentChannel] != CH_BG) { digitalWriteFast(LEDs[currentChannel], TURN_OFF); } 
  // Increment channel index
  currentChannel = currentChannel + 1;
  if ( currentChannel == NUM_CHANNELS ) { currentChannel = 0; }
  // Turn ON next LED, skip the background channel
  if (LEDs[currentChannel] != CH_BG) { digitalWriteFast(LEDs[currentChannel], TURN_ON); } 
  triggerOccurred = true;  // signal to main loop
}
