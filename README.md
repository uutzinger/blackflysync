# Blackfly Camera to LED sync
This software is developed for Teensy 4.x or 3.2.  
It controls an external light source for a camera.  
The frame trigger of the camera is used to advance to the next illumination color.  
The lightsource intensity is adjusted using PWM modulation.  
For a 500 frames/second camera and using PWM frequency of 50kHz will result in 100 light pulses per frame.  

This software is desgined for a FET driver and FET power switch. A FET driver usually has an enable/disable input. The gate of the driver is conencted to a PWM clock and the individual channel is enabled through the enable/disable input. Without such hardware this software is likely not useful.

Up to 13 channels are configured but more can be added.  
When auto advances is enabled each frame trigger first turns off current LED channel and activates next one.
Each channel has a pin associated on the microcontroller.  
There is one common PWM signal for all LEDs on one pin of the microcontroller.    
Individual channels can be disabled/enabled in software.
If the 14th Channel is enabled an image will be taken with all LEDs off.
A button can be attached to disable all lights. The button will need to create a conenction to ground when pressed. If the button pin is set to -1, this feature is disabled.

## Example Work Flow
  - Disable Auto Advance: a
    - f: set the PWM frequency
    - d: set the duty cycle
    - r: resolution in bits
    - C: set PWM pin
  - Load working channel from EEPROM: e.g. s0
  - Adjust working channel:
    - enable/disable channel
    - p: set the pin associated to current channel
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
Software version:     1.0
CPU:                  600 MHz
PWM pin:              22
Frequency:            50000.000000.1 Hz
Duty:                 +5.000 percent
Resolution:            8 bit
PWM Max:               255
Camera trigger is on: 21
Power switch:         0
-------------------------------------------------
State is:             Manual
-------------------------------------------------
Working on channel: 2 using pin:  2 which is off
-------------------------------------------------
Channel:  0 pin:  2 Enabled: No
Channel:  1 pin:  3 Enabled: No
Channel:  2 pin:  4 Enabled: No
Channel:  3 pin:  5 Enabled: No
Channel:  4 pin:  6 Enabled: No
Channel:  5 pin:  7 Enabled: No
Channel:  6 pin:  8 Enabled: No
Channel:  7 pin:  9 Enabled: No
Channel:  8 pin: 10 Enabled: No
Channel:  9 pin: 11 Enabled: No
Channel: 10 pin: 12 Enabled: No
Channel: 11 pin: 14 Enabled: No
Channel: 12 pin: 15 Enabled: No
Channel: 13 pin: 99 Enabled: Yes
```

Currently working on channel 2 which is attached to pin 2.  
The PWM frequency is 50kHz (same for all pins), the Duty Cycle on pin 22 is 5%.

## PWM Maximum Values
```
Resolution:  2 bit, Frequency: 37500000.000000
Resolution:  3 bit, Frequency: 18750000.000000
Resolution:  4 bit, Frequency: 9375000.000000
Resolution:  5 bit, Frequency: 4687500.000000
Resolution:  6 bit, Frequency: 2343750.000000
Resolution:  7 bit, Frequency: 1171875.000000
Resolution:  8 bit, Frequency: 585937.500000
Resolution:  9 bit, Frequency: 292968.750000
Resolution: 10 bit, Frequency: 146484.375000
Resolution: 11 bit, Frequency: 73242.187500
Resolution: 12 bit, Frequency: 36621.089844
Resolution: 13 bit, Frequency: 18310.550781
Resolution: 14 bit, Frequency: 9155.269531
Resolution: 15 bit, Frequency: 4577.640137
Resolution: 16 bit, Frequency: N.A.
```

## Available pins
```
Pin:  0, PWM
Pin:  1, PWM
Pin:  2, PWM
Pin:  3, PWM
Pin:  4, PWM
Pin:  5, PWM
Pin:  6, PWM
Pin:  7, PWM
Pin:  8, PWM
Pin:  9, PWM
Pin: 10, PWM
Pin: 11, PWM
Pin: 12, PWM
Pin: 13, PWM
Pin: 14, PWM
Pin: 15, PWM
Pin: 16, DIO
Pin: 17, DIO
Pin: 18, PWM
Pin: 19, PWM
Pin: 20, DIO
Pin: 21, DIO
Pin: 22, PWM
Pin: 23, PWM
Pin: 24, PWM
Pin: 25, PWM
Pin: 26, DIO
Pin: 27, DIO
Pin: 28, PWM
Pin: 29, PWM
Pin: 30, DIO
Pin: 31, DIO
Pin: 32, DIO
Pin: 33, PWM
Pin: 34, PWM
Pin: 35, PWM
Pin: 36, PWM
Pin: 37, PWM
Pin: 38, PWM
Pin: 39, PWM
```
