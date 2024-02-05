# Renduinix
This is the Open Source code for the Arduino based Renix Engine Monitor scan tool for Renix Equipped Jeeps from 1987 - 1990
This software is meant to stay Open Source.

  This sketch will read a 1986 to 1990 Jeep Renix ECU's TX output, convert the hex string into useable measurements,
  and display them on a 16x2 LCD display allowing you to cylce through with buttons.

## The circuit:
  
An Adafruit Pro Trinket 5V 16MHz is connected to a 16x2 RGB LCD Screen and 5 buttons to provide the base scanner. 

    Components needed for communication:
     just about any NPN Transisitor (I'm using a 2N3904)
     a 1k and 3.3k Resistor
     some wire and maybe a breadboard

The Renix Diagnostic Connector has 2 ports, D2 is the larger one. D2 Pins are numbered
   Bottom to Top, Left to Right if D1 is to the Left of D2

     D1      D2
    |4 1|   <3 6 9 12 15|
    |5 2)   <2 5 8 11 14|
    |6 3|   <1 4 7 10 13|

    Pin D2-1 is the ECU TX output, it is open collector to ground
    Pin D2-4 is Ignition Postive (+)
    Pin D2-7 is Ignition Ground  (-)

We will use an NPN transistor to physically NOT (invert) the data and use some resistors to get the switching done

    Arduino VIN pin ->  Pin D2-4 Ign Pos(+) (Pro Trinket's voltage regulator is good to 16v)
    Arduino 5V pin  ->  3.3k resistor -> NPN Base      <- Pin D2-1 TX         (This is the ECU Input)
    Arduino 5V pin  ->  1k resistor   -> NPN Collector <- Arduino RX Pin 0    (This is the Output to Arduino)
    Arduino Gnd pin ->                   NPN Emitter   <- Pin D2-7 Ign Gnd(-) (Negitive Rail)

  Core Code
- RenixComm V 0.3
- First incarnation January 7th, 2012
- This revision Jan 17th, 2012 (Subrevision 24)
- Phil Andrews 2011, 2012
- Thanks to Kes for help with the code

  Additional Code
- Renduinix v0.72
- Created July 13th, 2016 (v0.1)
- by Nick Risley
- Modified September 27th, 2016

  Update Log:
- v0.1 Modified core code to display predetermined outputs on a 16x2 display
- v0.2 Interfaced buttons to cycle through a sensor array
- v0.3 Updated calculations to show proper, added more data points, and debounced button presses
- v0.4 Cleaned up formating, custom character, updated text data points, added permanent memory function
- v0.5 Update pins for Trinket Pro, fixed RPM calc, added custom backlight options, fixed freeze if no data was present
- v0.6 Migrated code to Switch Case, Revamped Menu System, improved stream acquisition, added settings and debug, unified gauge spacing, added custom Baud and PROG# Option
- v0.7 Added EEPROM presets and reset, revamped vehicle selection, released Alpha Models
    - 0.71 Fixed Color and Software Info Menus
    - 0.72 Fixed Vehicle Selection Menu

