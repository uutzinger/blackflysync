# Blackfly Camera to LED sync
This software sets power of LEDs.  
It has 13 channels and autoadvances to next channel after each frame trigger.  
Each channel is composed of dutycycle and the hardware pin it is associate with.
Autoadvancing can be disabled.
Channels can be disabled/enabled.
13th Channel is used to take image with all LEDs off.

## Example Work Flow
  - Disable Auto Advance: a
  - Load working channel from EEPROM: e.g. s0
  - Adjust working channel:
    - p: set the pin associated to current channel
    - f: set the PWM frequency
    - d: set the duty cycle
    - r: resolution in bits
  - Save working channel: s
  - Enable Auto Advance: A
  - Save channel configurations to EEPROM: E

## Help/Configuration
```
  -------------------------------------------------
  i/I  system information
  -------------------------------------------------
  s0   load channel 0 to current working settings
  S0   save channel 0 from current working settings
  -------------------------------------------------
  c21  set camera trigger to pin 21
  o0   set on/off button to pin 20
  -------------------------------------------------
  a/A  disable/enable Auto Advance 
  m/M  disable/enable PWM pin 
  x    print channel settings
  e/E  read/save settings to EEPROM
  ------- Data Input-------------------------------
  p5   set PWM pin 5
  d50  set duty cyle to 50%
  f512 set frequency to 512Hz
  r8   set PWM resolution to 8 bits
  -------------------------------------------------
  Shannon McCoy, Urs Utzinger, 2020-21");
  -------------------------------------------------
```

## Example Channel Configuration

## Example System Settings
