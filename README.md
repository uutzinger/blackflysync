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
```
 0 Pin:  2 Duty: +5.000 Enabled: Yes
 1 Pin:  3 Duty: +5.000 Enabled: Yes
 2 Pin:  4 Duty: +5.000 Enabled: Yes
 3 Pin:  5 Duty: +5.000 Enabled: Yes
 4 Pin:  6 Duty: +5.000 Enabled: Yes
 5 Pin:  7 Duty: +5.000 Enabled: Yes
 6 Pin:  8 Duty: +5.000 Enabled: Yes
 7 Pin:  9 Duty: +5.000 Enabled: Yes
 8 Pin: 10 Duty: +5.000 Enabled: Yes
 9 Pin: 11 Duty: +5.000 Enabled: Yes
10 Pin: 12 Duty: +5.000 Enabled: Yes
11 Pin: 14 Duty: +5.000 Enabled: Yes
12 Pin: 15 Duty: +5.000 Enabled: Yes
13 Pin: 99 Duty: +0.000 Enabled: Yes
```

## Example System Settings
```
Frequency: 585937.500000 Hz
Duty: +0.000 percent
Resolution:  8 bit
CPU: 600 MHz
PWM Max:  255
Pin:  2 Disabled
Channel: 2
Camera Trigger: 21
Power Switch: 20
State is: Auto
```
