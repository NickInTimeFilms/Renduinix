/*
  Renduinix v0.7rev1

  This sketch will read a 1986 to 1990 Jeep Renix ECU's TX output, convert the hex string into useable measurements,
  and display them on a 16x2 LCD display allowing you to cylce through with buttons.

  The circuit:
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
  RenixComm V 0.3
  First incarnation January 7th, 2012
  This revision Jan 17th, 2012 (Subrevision 24)
  Phil Andrews 2011, 2012
  Thanks to Kes for help with the code

  Additional Code
  Renduinix v0.71
  Created July 13th, 2016 (v0.1)
  by Nick Risley
  Modified September 23th, 2016

  Update Log:
  v0.1 Modified core code to display predetermined outputs on a 16x2 display
  v0.2 Interfaced buttons to cycle through a sensor array
  v0.3 Updated calculations to show proper, added more data points, and debounced button presses
  v0.4 Cleaned up formating, custom character, updated text data points, added permanent memory function
  v0.5 Update pins for Trinket Pro, fixed RPM calc, added custom backlight options, fixed freeze if no data was present
  v0.6 Migrated code to Switch Case, Revamped Menu System, improved stream acquisition, added settings and debug, unified gauge spacing, added custom Baud and PROG# Option
  v0.7 Added EEPROM presets and reset, revamped vehicle selection, released Alpha Models
     -0.71 Fixed Color and Soft Info Menus,
*/
float softwareVersion = 0.71;

#include <EEPROM.h> //included for EEPROM memory functions
#include <LiquidCrystal.h> //included to interface with the LCD

//RS, E, 4, 5, 6, 7
LiquidCrystal lcd(12, A0, 8, 6, 5, 4);
//LiquidCrystal lcd(8, 9, 4, 5, 6, 7); //Old LCD Keypad Sheild Pins

//LCD RGB Backlight Pins, uses PWM
byte redPin = 9;
byte greenPin = 10;
byte bluePin = 11;

//Custom character setup for LCD
byte fahrenheitSymbol[8] =
{
  B11000,
  B11000,
  B01111,
  B01000,
  B01110,
  B01000,
  B01000
};
byte leftArrow[8] =
{
  B00010,
  B00100,
  B01000,
  B11111,
  B01000,
  B00100,
  B00010
};
byte rightArrow[8] =
{
  B01000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B01000
};

// defining values for buttons
byte lcdKey    = 0;
int adcKeyIn  = 0; //Analog read, keep int
const byte btnBOTTOMRIGHT = 0;
const byte btnTOPRIGHT    = 1;
const byte btnBOTTOMLEFT  = 2;
const byte btnTOPLEFT  = 3;
const byte btnSELECT = 4;
const byte btnNONE  = 5;

//Delays next button press in ms, poor mans debounce fucntion
byte buttonDelayFast = (10);  //used for fast selection
byte buttonDelay     = (100);  //general purpose like changing gauge selection
byte buttonDelaySlow = (200); //used for menu changes, keep under 255 or change to int

// read the buttons
int readLcdButtons()
{
  adcKeyIn = analogRead(1);      // read the value from the button resistors
  // my buttons when read are centered at these values: 0, 578, 649, 799, 933, 1013
  // we add approx 50 to those values to allow variation
  if (adcKeyIn < 100) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result

  // custom thresholds used
  if (adcKeyIn < 625)  return btnSELECT;
  if (adcKeyIn < 700)  return btnBOTTOMRIGHT;
  if (adcKeyIn < 850)  return btnBOTTOMLEFT;
  if (adcKeyIn < 975)  return btnTOPRIGHT;
  if (adcKeyIn < 1024) return btnTOPLEFT;

  return btnNONE;  // when all others fail, return this...
}

// Serial registers etc
uint8_t prev = 0xFF;
uint8_t current = 0xFF;
uint8_t next = 0xFF;
uint8_t Position = 0;
uint8_t dataStream[39];  // 32 bytes. we only have 1024 to play with. 31 for data, 1 for sign apart from it doesn't use the sign. bah.
uint8_t averageStream[39];
uint8_t averageLoop = 0;

// Serial data variables
byte ProgramVersion;
byte PROMversion;
byte CalibrationCode;
unsigned long PROM;
float MAP = 0;
int CTS = 0;
int IAT = 0;
float Volts = 0;
float Lambda = 0;
unsigned int RPM = 0;
byte PROM9_10;
byte PROM7_8;
byte TPS = 0;
int SparkADV = 0;
int Unknown14;
int Unknown15;
float Baro = 0;
int Unknown17;
int LoopStatus;
int Exhaust;
float InjPulse = 0;
int Unknown20;
int ThrottleSwitch;
int FuelSync;
int Unknown22;
int Unknown23;
byte STFuelTrim = 0;
int Unknown25;
byte LTFuelTrim = 0;
byte Knock = 0;
int Unknown28;
int ACSwitch;
int ACRequest;
byte PROM11_12;

//callback pointer stuff for displaying each sensor string
typedef void (*GeneralFunction) ();

//each callback is named and given commands
void displayPROM ()
{
  lcd.print(ProgramVersion, HEX);
  lcd.print(PROMversion, HEX);
  lcd.print(CalibrationCode, HEX);
  lcd.print(PROM7_8, HEX);
  lcd.print(PROM9_10, HEX);
  lcd.print(PROM11_12, HEX);
}
void displayMAP ()
{
  if (MAP < 10)
    lcd.print(" ");//adds extra space to keep formatting
  lcd.print(" "); //1 spacer
  lcd.print(MAP, 1); //1 decimal point
  lcd.print("MAP");
}
void displayCTS ()
{
  if (CTS >= -9 && CTS <= -1)
    lcd.print(" ");
  if (CTS >= 0 && CTS <= 9)
    lcd.print(" ");
  if (CTS >= 10 && CTS <= 99)
    lcd.print(" ");
  lcd.print("  "); //2 spacers
  lcd.print(CTS);
  lcd.write(3);    //writes custom character 3 (fahrenheit)
  lcd.print("WT");
}
void displayIAT ()
{
  if (IAT >= -9 && IAT <= -1)
    lcd.print(" ");
  if (IAT >= 0 && IAT <= 9)
    lcd.print(" ");
  if (IAT >= 10 && IAT <= 99)
    lcd.print(" ");
  lcd.print("  "); //2 spacers
  lcd.print(IAT);
  lcd.write(3);    //writes custom character 3 (fahrenheit)
  lcd.print("IA");
}
void displayVolts ()
{
  if (Volts < 10)
    lcd.print(" ");
  lcd.print(" "); //1 spacer
  lcd.print(Volts, 1); //1 decimal point
  lcd.print("VLT");
}
void displayLambda ()
{
  lcd.print(" "); //1 spacer
  lcd.print(Lambda);
  lcd.print("vO2");
}
void displayRPM ()
{
  if (RPM < 10)
    lcd.print(" ");
  if (RPM < 100)
    lcd.print(" ");
  if (RPM < 1000)
    lcd.print(" ");
  lcd.print(" "); //1 spacer
  lcd.print(RPM);
  lcd.print("RPM");
}
void displayTPS ()
{
  if (TPS < 10)
    lcd.print(" ");
  lcd.print("   "); //3 spacers
  lcd.print(TPS);
  lcd.print("TPS");
}
void displaySparkADV ()
{
  if (SparkADV < 10)
    lcd.print(" ");
  lcd.print("   "); //3 spacers
  lcd.print(SparkADV);
  lcd.print("IGN");
}
void displayUnknown14 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown14);
  lcd.print("?14");
}
void displayUnknown15 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown15);
  lcd.print("?15");
}
void displayBaro ()
{
  if (Baro < 10)
    lcd.print(" ");
  lcd.print(" "); //1 spacer
  lcd.print(Baro, 1); //1 decimal
  lcd.print("BAR");
}
void displayUnknown17 ()
{
  lcd.print("  "); //2 spacer
  lcd.print(Unknown17);
  lcd.print("?17");
}
void displayLoopStatus ()
{
  if ((LoopStatus / 2) % 2) { //bitwise math checks even or odd, 0 indexed every 2 #'s
    lcd.print("Decel"); //if odd do, Decel is always #/2 odd
  }
  else if (LoopStatus / 64 % 2) {
    lcd.print(" Clsd"); //if #/64 odd do, 64-127, 192-255
  }
  else lcd.print(" Open"); //Else even do, 0-63, 128-191
  lcd.print(" LP");
}
void displayExhaust ()
{
  if (Exhaust < 128) {
    lcd.print(" Lean"); //if less than do
  }
  else {
    lcd.print(" Rich"); // else do greater than
  }
  lcd.print(" EX");
}
void displayInjPulse ()
{
  if (InjPulse < 10)
    lcd.print(" ");
  lcd.print("  "); //2 spacers
  lcd.print(InjPulse, 1); // 1 decimal
  lcd.print("mS");
}
void displayUnknown20 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown20);
  lcd.print("?20");
}
void displayThrottleSwitch ()
{
  lcd.print(ThrottleSwitch);
  lcd.print(" THR");
}
void displayFuelSync ()
{
  lcd.print("   "); // 3 spacers
  if ((FuelSync / 4) % 2) { //bitwise magic that compares even or odd, 0 indexed every 4 #'s
    lcd.print("-"); // if odd do
  }
  else {
    lcd.print("+"); // if even do
  }
  lcd.print(" SYN");
}
void displayUnknown22 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown22);
  lcd.print("?22");
}
void displayUnknown23 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown23);
  lcd.print("?23");
}
void displaySTFuelTrim ()
{
  if (STFuelTrim < 10)
    lcd.print(" ");
  if (STFuelTrim < 100)
    lcd.print(" ");
  lcd.print("  "); //2 spacers
  lcd.print(STFuelTrim);
  lcd.print(" ST");
}
void displayUnknown25 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown25);
  lcd.print("?25");
}
void displayLTFuelTrim ()
{
  if (LTFuelTrim < 10)
    lcd.print(" ");
  if (LTFuelTrim < 100)
    lcd.print(" ");
  lcd.print("  "); //2 spacers
  lcd.print(LTFuelTrim);
  lcd.print(" LT");
}
void displayKnock ()
{
  if (Knock < 10)
    lcd.print(" ");
  if (Knock < 100)
    lcd.print(" ");
  lcd.print("  "); //2 spacers
  lcd.print(Knock);
  lcd.print("KNO");
}
void displayUnknown28 ()
{
  lcd.print("  "); //2 spacers
  lcd.print(Unknown28);
  lcd.print("?28");
}
void displayACSwitch ()
{
  lcd.print(" "); //1 spacer
  if ((ACSwitch / 32) % 2) {
    lcd.print(" On"); //if odd
  }
  else lcd.print("Off"); //else even
  lcd.print(" A/C");
}
void displayACRequest ()
{
  lcd.print(" "); //1 spacer
  if ((ACRequest / 64) % 2) {
    lcd.print("Yes"); //if odd
  }
  else lcd.print(" No"); //else even
  lcd.print(" REQ");
}

//Data Array connected to callback pointer
GeneralFunction dataArray[] = { displayMAP, displayCTS, displayIAT, displayRPM, displayVolts, displayLambda, displayExhaust, displayLoopStatus, displayTPS, displaySparkADV, displayKnock, displayInjPulse, displayFuelSync, displaySTFuelTrim, displayLTFuelTrim, displayACSwitch, displayACRequest};
int dataTotal = 17; //number of data points in above array
byte dataOrder1 = 6; //Top Left counter, 0 indexed, used byte so that EEPROM can use it
byte dataOrder2 = 5; //Top Right counter, 0 indexed
byte dataOrder3 = 11;//Bottom Left counter, 0 indexed
byte dataOrder4 = 4; //Bottom Right counter, 0 indexed

//EEPROM Memory Variables for storing information between power cylces (Notice 0 was skipped for continuity)
byte EEdataOrder1 = 1; //EEPROM Address to store first counter
byte EEdataOrder2 = 2; //EEPROM Address to store second counter
byte EEdataOrder3 = 3; //EEPROM Address to store third counter
byte EEdataOrder4 = 4; //EEPROM Address to store forth counter

//Menu Counter used to cycle through menues
byte menuOrder = 1; //Sets to Waiting screen initially
byte menuDelay = 40; //delay used on menus to reduce screen flicker

//Assign variables to menus for easy reference
const byte Home = 0;
const byte Waiting = 1; 
const byte Vehicle = 2;
const byte Gauges = 3;
const byte Diagnose = 4;
const byte Options = 5;
const byte Units = 6;
const byte PickColor = 7;
const byte Backlight = 8;
const byte CustomColor = 9;
const byte RGBCustomSave = 10;
const byte Debug = 11;
const byte Settings = 12;
const byte VehicleCustom = 13;
const byte BaudRateMenu = 14;
const byte ProgramNumberMenu = 15;

byte vehicleYear = 87;
long baudRate = 62500;
int programNumber = 177;
int customProgramNumber = 177;
byte baudRateOrder = 0;
byte programNumberOrder = 2;//starts in 1's place

//current LED values to be multiplied by dim value
int redValue = 0; 
int greenValue = 128;
int blueValue = 0;

//diming variable to affect final output of RGB Variables
byte RGBDim = 100; //100 = Max and 0 = Off
int RGBDimOrder = 3; //0 = Off, 1 = Low, 2 = Med, 3 = High, int to prevent overflow
byte RGBDimTotal = 4; //total number of options

//final value that is outputed to the LEDs
int redDisplay = 0; 
int greenDisplay = 0;
int blueDisplay = 0;

//counter variables for RGB Preset Color Selection Screen
int RGBOrder = 0; //0 = Red, Orange, Yellow, Green, Cyan, Blue, Purple, Pink, White, 9 = Custom; int to allow looping
byte RGBTotal = 10;

//counter variables for RGB Custom Color Selection Screen
byte RGBCustomOrder = 0; //0 = red, 1 = green, 2= blue
byte RGBCustomTotal = 3;

//stores custom RGB color values
int redCustom = 0;//int to keep from looping
int greenCustom = 0;
int blueCustom = 0;

//EEPROM Addresses to save options between power cycles
byte EEreset = 0;        //used to determin EEPROM State for first boot
byte EERGBOrder = 5;     //preset color setting
byte EERGBDimOrder = 6;  //dim setting
byte EEredValue = 7;     //selected color values
byte EEgreenValue = 8;
byte EEblueValue = 9;
byte EEredCustom = 10;   //custom color values
byte EEgreenCustom = 11;
byte EEblueCustom = 12;
byte EERGBDim = 13;      //selected dim value
byte EEsplashScreen = 14;
byte EEupdateSpeedMenu = 15;
byte EEupdateSpeed = 16;
byte EEprogramNumber = 17;
byte EEcustomProgramNumber = 18;
byte EEvehicleYear = 19;
byte EEModelNumber = 20;

//Settings Variables
int settingsOrder = 0;//int for looping
byte settingsTotal = 4; //Splash, Speed, Reset, info
int splashScreen = 1; //0 off, 1 on
int updateSpeedMenu = 1;
int updateSpeed = 2;
byte slow = 200; //Gauge Update Speeds in milliseconds/10 (must be less than 256)
byte med = 90;
byte fast = 20;
byte ludi = 8;
byte debugStream = 0; //Sets Number Base, 0 = Dec, 1 = Hex
byte debugStreamOrder = 0;//sets debug stream "page" #

//********************************************************************************************************************************************************************************************************
void setup()
{
  //Check to see if EEPROM is blank and write useable values 
  if (EEPROM.read(EEreset) == 255){
  EEPROM.update(EEreset, 0);
  EEPROM.update(EEdataOrder1, 1);
  EEPROM.update(EEdataOrder2, 3);
  EEPROM.update(EEdataOrder3, 4);
  EEPROM.update(EEdataOrder4, 5);
  EEPROM.update(EERGBOrder, 3);     //preset color setting
  EEPROM.update(EERGBDimOrder, 3);  //dim setting
  EEPROM.update(EEredValue, 0);     //selected color values
  EEPROM.update(EEgreenValue, 128);
  EEPROM.update(EEblueValue, 0);
  EEPROM.update(EEredCustom, 128);   //custom color values
  EEPROM.update(EEgreenCustom, 128);
  EEPROM.update(EEblueCustom, 128);
  EEPROM.update(EERGBDim, 100);      //selected dim value
  EEPROM.update(EEsplashScreen, 1);
  EEPROM.update(EEupdateSpeedMenu, 1);
  EEPROM.update(EEupdateSpeed, fast);
  EEPROM.update(EEprogramNumber, 177);
  EEPROM.update(EEcustomProgramNumber, 177);
  EEPROM.update(EEvehicleYear, 87);
  }
  
// Sets the Active Varables equal to the Saved Variables
  redValue = EEPROM.read(EEredValue);
  greenValue = EEPROM.read(EEgreenValue);
  blueValue = EEPROM.read(EEblueValue);
  
  RGBDim = EEPROM.read(EERGBDim);
  
  redCustom = EEPROM.read(EEredCustom);
  greenCustom = EEPROM.read(EEgreenCustom);
  blueCustom = EEPROM.read(EEblueCustom);
  
  RGBOrder = EEPROM.read(EERGBOrder);
  RGBDimOrder = EEPROM.read(EERGBDimOrder);
    
  redDisplay = (redValue * RGBDim / 100);       //adjusts RGB values to selected dim setting
  greenDisplay = (greenValue * RGBDim / 100);
  blueDisplay = (blueValue * RGBDim / 100);

  updateSpeed = EEPROM.read(EEupdateSpeed);

  programNumber = EEPROM.read(EEprogramNumber);
  customProgramNumber = EEPROM.read(EEcustomProgramNumber);
  vehicleYear = EEPROM.read(EEvehicleYear);
  
  analogWrite(redPin, redDisplay);        //turns on RGB backlight to set amount
  analogWrite(greenPin, greenDisplay);    //PWM is used to control the brightness
  analogWrite(bluePin, blueDisplay);
  
  Serial.begin(baudRate);      // Begin serial comms at 62500.
  pinMode(0, INPUT);         // Pin zero, RX on the UART
  pinMode(13, OUTPUT);       // Pin 13, connected to the LED L on the board, useful to show activity
  digitalWrite(13, HIGH);    // Turn L led on
  pinMode(3, OUTPUT);
  analogWrite(3, 10);
  prev = 0;                  // Set the PREV to 0
  Serial.flush();           // Dump the incoming buffer because it will probably be full of junk
  digitalWrite(0, HIGH);     // Renix II is open collector serial-out. Saves on external component count. Requires 1 NPN in open emitter to physically NOT the data

  lcd.begin(16, 2);                     //set up the LCD's number of columns and rows
  lcd.createChar(1, leftArrow);  //sends custom character to lcd memory
  lcd.createChar(2, rightArrow);
  lcd.createChar(3, fahrenheitSymbol);
  
  if (EEPROM.read(EEsplashScreen) == 1)
    {
    lcd.setCursor(1, 0);
    lcd.print("Renduinix v");          //display Splash Screen Text
    lcd.print(softwareVersion);
    lcd.setCursor(0, 1);
    lcd.print("Nick Risley 2016");
    delay(3000);                          //wait
    }
  digitalWrite(13, LOW);                //turns off LED
  analogWrite(3, 0);
}

void loop()
{
  lcd.noCursor();                       //removes underline cursor
  lcd.noBlink();
 
  redDisplay = (redValue * RGBDim / 100);     //adjusts RGB values to selected dim setting
  greenDisplay = (greenValue * RGBDim / 100);
  blueDisplay = (blueValue * RGBDim / 100);
  
  analogWrite(redPin, redDisplay);        //adjusts RGB backlight to set amount
  analogWrite(greenPin, greenDisplay);    //PWM is used to control the brightness
  analogWrite(bluePin, blueDisplay);
  
  lcdKey = readLcdButtons();      //reads variable input
  if (lcdKey == btnSELECT)        //checks if Select button press is true
  {
    menuOrder = Home;             //sets order back to home screen
    delay(buttonDelaySlow);       //delays next cycle to debounce button presses
  }
    
//Main Switch Case to control displayed Menu
switch (menuOrder)
 {
  case Gauges://Gauge Screen *********************************************************************************************
    if (Serial.available() > 0){ //prevents code locking up when no data is present
      do
      {
        if (Serial.available() > 32 )  // Make sure the buffer has read in 33 bytes.
        {
          prev = current;  // Drop the current byte and read in what's first off the buffer
          current = Serial.read();  // Shift the first byte in off the buffer
          next = Serial.peek();  // Read the next byte off the buffer without modifying anything. Saves on a register.
    
          /* This sequence loops three bytes over and over, in sequence through the three memory locations. Useful 3-byte search. We are looking for
            a particular sequence, specifically FF 00 xx but if the ignition is off FF 00 is present part the way through the stream so that needs to be determined.
            Phil relied on the next number being a non-zero but this doesn't always work. I have updated it to look for the Program Number 3rd so it always works*/
    
          if (prev == 0xFF && current == 0x00)
          {
            if (next == 176 || next == 177) //  Looks for the Program Number to verify the beginning of the stream, mine was B1 or 177
            {
            digitalWrite(13, HIGH); // Turn the L LED on to show we are reading in good data.
              for (Position = 0; Position < 31; Position++) {
                dataStream[Position] = Serial.read();  //  Loop 31 bytes into the array straight off the buffer. This is a quick operation.
              }
              for (Position = 0; Position < 31; Position++) {
                averageStream[Position] = dataStream[Position];  // Copy the data into a second array.
              }
              for (Position = 0; Position < 31; Position++) {
                averageStream[Position] = ((dataStream[Position] + averageStream[Position]) / 2); // Add the two together and divide by two to average. This loops.
              }
              averageLoop++;  // AverageLoop seems best at about 3 but it can be a bit glitchy, 4 seems more stable. Over 5 seems to miss the nuances and becomes a bit sluggish.
            }
          }
        }
      } while (averageLoop < 3);  //  See the loop of the above code
      digitalWrite(13, LOW);      // Now we have finished reading in data and averaging it, turn the LED off
      averageLoop = 0;            // Reset the loop
      digitalWrite(0, LOW);       // Drop the input. In theory if the transistor isn't biased wrong the buffer won't fill up. Pointless but eh.
    }
  /* Read the various bytes and words of the array into variables, so calculations can be easily performed
    without screwing up the average. There is a delay which this smooths out some */
  
    ProgramVersion = dataStream[0];          // Not useful as an average, as it should always be the same
    PROMversion = dataStream[1];             // as above
    CalibrationCode = dataStream[2];         // and again, we want the instantaneous
    PROM = (dataStream[1], dataStream[2], dataStream[3], dataStream[11], dataStream[10], dataStream[30]);
    MAP = (averageStream[3] / 9.13 + 3.1);   //converts to "Hg (3.1 - 31)
    CTS = (averageStream[4] / .888 - 40);    //converts to Fahrenheit (-40 - 247)
    IAT = (averageStream[5] / .888 - 40);    //and again
    Volts = (averageStream[6] / 16.17);      //converts to Volts (0 - 15.8)
    Lambda = (averageStream[7] / 51.2);
    if (word(averageStream[9], averageStream[8]) != 0) {
      RPM = 19850000 / word(averageStream[9], averageStream[8]); //9 & 8 are combined into a 16 bit "word" that can then be dec converted and divided.
    }                                         //ex: 9 = 128 & 8 = 128. 128128 in binary is 1000000010000000 or 32896 when converted to dec.
    if (RPM >= 7000) {
      RPM = 7000; // burpy data can make the rpm spike so that's the self imposed limit. Redline is 5200
    }
    PROM9_10 = dataStream[10];
    PROM7_8 = dataStream[11];
    TPS = (averageStream[12] / 2.55);        // Throttle position sensor in Percentage. (0 - 100)
    SparkADV = averageStream[13];            // Ignition Timing in degrees (0 - 255)
    Unknown14 = averageStream[14];           //always 0
    Unknown15 = averageStream[15];
    Baro = (averageStream[16] / 9.13 + 3.1); //Initial MAP reading before Startup in "Hg
    Unknown17 = averageStream[17];
    LoopStatus = dataStream[18];
    Exhaust = dataStream[18];
    InjPulse = (averageStream[19] / 7.81);   //Pulse Width in mS (0 - 32.7)
    Unknown20 = averageStream[20];           //always 0
    ThrottleSwitch = dataStream[21];
    FuelSync = dataStream[21];
    Unknown22 = averageStream[22];
    Unknown23 = averageStream[23];
    STFuelTrim = averageStream[24];          //short term fuel trim (0 - 255)
    Unknown25 = averageStream[25];
    LTFuelTrim = averageStream[26];          //long term fuel trim (0 - 255)
    Knock = averageStream[27];               //knock sensor (0 - 255)
    Unknown28 = averageStream[28];
    ACSwitch = dataStream[29];               //text from specific intervals
    ACRequest = dataStream[29];              //same as above
    PROM11_12 = dataStream[30];
    
    dataOrder1 = EEPROM.read(EEdataOrder1);  //sets the counter variable equal to the saved EEPROM value
    dataOrder2 = EEPROM.read(EEdataOrder2);
    dataOrder3 = EEPROM.read(EEdataOrder3);
    dataOrder4 = EEPROM.read(EEdataOrder4);
  
    lcd.clear();                          //clears screen for new commands
  
    lcdKey = readLcdButtons();//*******reads variable input********
    switch (lcdKey)
    {
      case btnTOPLEFT:        
        ++dataOrder1;                     //increases order by 1 if above is true
        delay(buttonDelay);               //delays next cycle to debounce button presses
        if (dataOrder1 > dataTotal - 1)   //checks if order is out of bounds
          dataOrder1 = 0;                 //sets it back to 0 if above is true
        break;
      case btnTOPRIGHT:                   //Same as above for Top Right
        ++dataOrder2;
        delay(buttonDelay);
        if (dataOrder2 > dataTotal - 1)
          dataOrder2 = 0;
        break;
      case btnBOTTOMLEFT:                 //Same as above for Botttom Left
        ++dataOrder3;
        delay(buttonDelay);
        if (dataOrder3 > dataTotal - 1)
          dataOrder3 = 0;
        break;
      case btnBOTTOMRIGHT:                //Same as above for Bottom Right
        ++dataOrder4;
        delay(buttonDelay);
        if (dataOrder4 > dataTotal - 1)
          dataOrder4 = 0;
        break;
    }
  
    lcd.setCursor(0, 0);              //set top left
    dataArray [dataOrder1] ();        //prints selected data from the Array
    lcd.setCursor(8, 0);
    dataArray [dataOrder2] ();
    lcd.setCursor(0, 1);
    dataArray [dataOrder3] ();
    lcd.setCursor(8, 1);
    dataArray [dataOrder4] ();
      
    EEPROM.update(EEdataOrder1, dataOrder1); //Updates EEPROM value to counter value if it has changed
    EEPROM.update(EEdataOrder2, dataOrder2); //EEPROM has limited lifecycle so (update) is more efficent then (write)
    EEPROM.update(EEdataOrder3, dataOrder3);
    EEPROM.update(EEdataOrder4, dataOrder4);
    
    delay(updateSpeed*10);// used to slow down the sketch so you can read the outputs
    
    Serial.flush();  // We done, flush any junk out of the buffer, so we can sieve for the header and begin again
    digitalWrite(0, HIGH); // push the read line high to begin reading in again
    break;
  case Diagnose://Diagnoses Screen***************************************************************************************
    lcd.clear();
    lcd.print("It MIGHT b broke");
    delay(menuDelay);
  break;
  case Home://Home Selection Screen *******************************************************************************
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Vehicle");
    lcd.setCursor(8,0);
    lcd.print("  Gauges");
    lcd.setCursor(0,1);
    lcd.print("Diagnose");
    lcd.setCursor(8,1);
    lcd.print(" Options");

    lcdKey = readLcdButtons();//*******reads variable input********

    switch (lcdKey)
    {
      case btnTOPLEFT://Vehicle
        menuOrder = Vehicle;
        delay(buttonDelaySlow);
      break;
      case btnTOPRIGHT://Gauges
        menuOrder = Gauges;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMLEFT://Diagnose
        menuOrder = Diagnose;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMRIGHT://Options
        menuOrder = Options;
        delay(buttonDelaySlow);
      break;
    }
    delay(menuDelay);
  break;
  case Vehicle://Vehicle Selection Screen************************************************************************************
    lcd.clear();
    lcd.setCursor(0,0);
    if (vehicleYear < 91)
      lcd.print("^");
    else lcd.print("E");
    lcd.setCursor(2,0);
    lcd.print("Select Year:");
    lcd.setCursor(0,1);
    lcd.print("v ");
    if (vehicleYear < 91)
      lcd.print(vehicleYear+1900);
    else lcd.print("User");
    lcd.print(" 4.0L Auto");
    lcdKey = readLcdButtons();

    switch (lcdKey)
    {
      case btnTOPLEFT://Increase
        vehicleYear++;
        if (vehicleYear > 91)
          menuOrder = ProgramNumberMenu;
        delay(buttonDelaySlow);
      break;
      case btnTOPRIGHT:
        menuOrder = ProgramNumberMenu;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMLEFT://Decrease
        vehicleYear--;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMRIGHT:
        delay(buttonDelaySlow);
      break;
    }
    vehicleYear = constrain(vehicleYear, 87, 91);
    
    if (vehicleYear > 90)
      programNumber = customProgramNumber;
    else if (vehicleYear > 88)
      programNumber = 177;
    else programNumber = 176;
    lcd.cursor();
    lcd.setCursor(5,1);
    EEPROM.update(EEvehicleYear, vehicleYear);
    EEPROM.update(EEprogramNumber, programNumber);
    delay(menuDelay);
  break;
  case VehicleCustom://Custom Vehicle Selection Screen ******************************************************************************
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Baud Rate:");
    lcd.print(baudRate);
    lcd.setCursor(0,1);
    lcd.print("PROG Numb:");
    lcd.print(programNumber);
    
    lcdKey = readLcdButtons();//read bottom buttons to change value               
        switch(lcdKey)                          
        {
        case btnTOPLEFT:
          menuOrder = BaudRateMenu;
          delay(buttonDelaySlow);  
        break;
        case btnBOTTOMLEFT:
          menuOrder = ProgramNumberMenu;
          delay(buttonDelaySlow);  
        break;
        }
    delay(menuDelay);
  break;
  case BaudRateMenu://**************************************************************************************************
    lcd.clear();
    lcd.cursor(); //turns on underline cursor
    lcd.setCursor(0,0);
    lcd.print("^ Baud  Rate:  R");
    lcd.setCursor(0,1);
    lcd.print("v   ");
    if (baudRate < 10)
      lcd.print(" ");
    if (baudRate < 100)
      lcd.print(" ");
    if (baudRate < 1000)
      lcd.print(" ");
    if (baudRate < 10000)
      lcd.print(" ");
    if (baudRate < 100000)
      lcd.print(" ");
    lcd.print(baudRate);
    lcd.setCursor(15,1);
    lcd.print(">");
  
    lcdKey = readLcdButtons();//*******reads variable input********
    switch (lcdKey)
    {
      case btnTOPLEFT://Increase
          switch (baudRateOrder)
          {
          case 0:
            baudRate = baudRate + 10000;  
          break;
          case 1:
            baudRate = baudRate + 1000;  
          break;
          case 2:
            baudRate = baudRate + 100;  
          break;
          case 3:
            baudRate = baudRate + 10;  
          break;
          case 4:
            baudRate = baudRate + 1;  
          break;
          }
      delay(buttonDelaySlow);    
      break;
      case btnTOPRIGHT://Reset
        baudRate = 62500;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMLEFT://Decrease
         switch (baudRateOrder)
          {
          case 0:
            baudRate = baudRate - 10000;  
          break;
          case 1:
            baudRate = baudRate - 1000;  
          break;
          case 2:
            baudRate = baudRate - 100;  
          break;
          case 3:
            baudRate = baudRate - 10;  
          break;
          case 4:
            baudRate = baudRate - 1;  
          break;
          }
      delay(buttonDelaySlow);
      break;
      case btnBOTTOMRIGHT://Options
        baudRateOrder++;
        if (baudRateOrder > 4)
          baudRateOrder = 0;
        delay(buttonDelaySlow);
      break;
    }
    switch (baudRateOrder)
          {
          case 0:
            lcd.setCursor(5,1);  
          break;
          case 1:
            lcd.setCursor(6,1); 
          break;
          case 2:
            lcd.setCursor(7,1);  
          break;
          case 3:
            lcd.setCursor(8,1);  
          break;
          case 4:
            lcd.setCursor(9,1);  
          break;
          }
  delay(menuDelay);
  break;
  case ProgramNumberMenu://**********************************************************************************************************
  lcd.clear();
    lcd.cursor(); //turns on underline cursor
    lcd.setCursor(0,0);
    lcd.print("^ PROG Number: R");
    lcd.setCursor(0,1);
    lcd.print("v    ");
    if (customProgramNumber < 10)
      lcd.print(" ");
    if (customProgramNumber < 100)
      lcd.print(" ");
    lcd.print(customProgramNumber);
    lcd.setCursor(15,1);
    lcd.print(">");
  
    lcdKey = readLcdButtons();//*******reads variable input********
    switch (lcdKey)
    {
      case btnTOPLEFT://Increase
          switch (programNumberOrder)
          {
          case 0:
            customProgramNumber = customProgramNumber + 100;  
          break;
          case 1:
            customProgramNumber = customProgramNumber + 10;  
          break;
          case 2:
            customProgramNumber = customProgramNumber + 1;  
          break;
          }
        if (customProgramNumber > 255)
          customProgramNumber = 255;
        delay(buttonDelaySlow);    
      break;
      case btnTOPRIGHT://Reset
        customProgramNumber = 177;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMLEFT://Decrease
         switch (programNumberOrder)
          {
          case 0:
            customProgramNumber = customProgramNumber - 100;  
          break;
          case 1:
            customProgramNumber = customProgramNumber - 10;  
          break;
          case 2:
            customProgramNumber = customProgramNumber - 1;  
          break;
          }
        if (customProgramNumber < 0)
          customProgramNumber = 0;  
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMRIGHT://Options
        programNumberOrder++;
        if (programNumberOrder > 2)
          programNumberOrder = 0;
        delay(buttonDelaySlow);
      break;
    }
    switch (programNumberOrder)
          {
          case 0:
            lcd.setCursor(5,1);  
          break;
          case 1:
            lcd.setCursor(6,1); 
          break;
          case 2:
            lcd.setCursor(7,1);  
          break;
          }
  EEPROM.update(EEcustomProgramNumber, customProgramNumber);
  programNumber = customProgramNumber;
  delay(menuDelay);
  break;
  case Waiting://Waiting Screen *****************************************************************************************
    if (Serial.available() > 0){
      menuOrder = Gauges;             //sets menu to gauge screen if data is available
      }
    else{  
    lcd.clear();                          
    lcd.setCursor(0, 0);                 
    lcd.print("Waiting for data");    //idle screen
    lcd.setCursor(2, 1);                 
    lcd.print("or Press Home");
    delay(menuDelay);
    }
    break;
  
  case Options://Main Options Screen*************************************************************************************
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Units");
    lcd.setCursor(8,0);
    lcd.print("Display");
    lcd.setCursor(0,1);
    lcd.print("Debug");
    lcd.setCursor(8,1);
    lcd.print("Settings");
    lcdKey = readLcdButtons();         
    switch(lcdKey)                      
    {
      case btnTOPLEFT://Units
        menuOrder = Units;
        delay(buttonDelaySlow);
      break;
      case btnTOPRIGHT://Display
        menuOrder = PickColor;
        delay(buttonDelaySlow);
        break;
      case btnBOTTOMLEFT://Debug
         menuOrder = Debug;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMRIGHT://Settings
        menuOrder = Settings;
        delay(buttonDelaySlow);
      break;    
    }
    delay(menuDelay);  
  break;
  case Units: //Unit Selection Screen *****************************************************************************************
    lcd.clear();
    lcd.print("G-Unit");
    delay(menuDelay);
  break;
  case PickColor: //Preset Color Screen ***************************************************************************************
    lcd.clear();
    lcd.print("<");
    lcd.setCursor(1,0);
    lcd.print("Color:");
    lcd.setCursor(15,0);
    lcd.print(">");
    lcd.setCursor(0,1);
    lcd.write(1);//left arrow
    lcd.print("R");
    lcd.print(redValue);
    lcd.setCursor(6,1);
    lcd.print("G");
    lcd.print(greenValue);
    lcd.setCursor(11,1);
    lcd.print("B");
    lcd.print(blueValue);
    lcd.setCursor(15,1);
    lcd.write(2);//right arrow
  
    //case statment for RGBOrder ******************************
    lcdKey = readLcdButtons();          //reads variable input
    switch(lcdKey)                      //sets variable to switch
    {
      case btnTOPLEFT :                  //back option
        menuOrder = Backlight;
        delay(buttonDelaySlow);
        break;
      case btnTOPRIGHT:
        menuOrder = CustomColor;
        delay(buttonDelaySlow);
        break;
      case btnBOTTOMLEFT:                  //Decrease case*****
        --RGBOrder;
        delay(buttonDelaySlow);
        if (RGBOrder < 0)               //checks if order is out of bounds
           {
           RGBOrder = (RGBTotal -1);    //sets it back to max if above is true
           }
        break;
      case btnBOTTOMRIGHT:                 //Increase case*****       
        ++RGBOrder;
        delay(buttonDelaySlow);
        if (RGBOrder > (RGBTotal - 1))  
          {
          RGBOrder = 0;                 //sets it back to 0 if above is true
          }
        break;
    } 
    //case statment to display selected color ******************
    switch(RGBOrder) 
    {
      case 0://Red
        redValue   = 255;
        greenValue = 0; 
        blueValue  = 0;
        lcd.setCursor(8,0);
        lcd.print("Red");
        break;
      case 1://Orange
        redValue   = 255;
        greenValue = 24; 
        blueValue  = 0;
        lcd.setCursor(8,0);
        lcd.print("Orange");   
        break;
      case 2://Yellow
        redValue   = 255;
        greenValue = 50; 
        blueValue  = 0;
        lcd.setCursor(8,0);
        lcd.print("Yellow");   
        break;     
      case 3://Green
        redValue   = 0;
        greenValue = 128; 
        blueValue  = 0;
        lcd.setCursor(8,0);
        lcd.print("Green");   
        break;
      case 4://Cyan
        redValue   = 0;
        greenValue = 128; 
        blueValue  = 128;
        lcd.setCursor(8,0);
        lcd.print("Cyan");   
        break;  
      case 5://Blue
        redValue   = 0;
        greenValue = 0; 
        blueValue  = 255;
        lcd.setCursor(8,0);
        lcd.print("Blue");   
        break;
      case 6://Purple
        redValue   = 255;
        greenValue = 0; 
        blueValue  = 180;
        lcd.setCursor(8,0);
        lcd.print("Purple");   
        break;  
      case 7://Pink
        redValue   = 255;
        greenValue = 0; 
        blueValue  = 50;
        lcd.setCursor(8,0);
        lcd.print("Pink");   
        break;  
      case 8://White
        redValue   = 255;
        greenValue = 60; 
        blueValue  = 70;
        lcd.setCursor(8,0);
        lcd.print("White");   
        break;  
      case 9://Custom
        redValue   = redCustom;
        greenValue = greenCustom; 
        blueValue  = blueCustom;
        lcd.setCursor(8,0);
        lcd.print("Custom");   
        break;       
     }     
    EEPROM.update(EEredValue, redValue); //Saves selection to memory
    EEPROM.update(EEgreenValue, greenValue);
    EEPROM.update(EEblueValue, blueValue);
    EEPROM.update(EERGBOrder, RGBOrder);
    delay(menuDelay);
    break;
  case CustomColor: //Custom RGB Backlight Menu *****************************************************************************
    //Set custom variables to current variables to show previously saved color 
    redValue = redCustom;
    greenValue = greenCustom;
    blueValue = blueCustom;
     
    lcd.clear();
    lcd.cursor(); //turns on underline cursor
    lcd.setCursor(0,0);
    lcd.print("^ Custom Color S");
    lcd.setCursor(0,1);
    lcd.print("vR");
    lcd.print(redValue);
    lcd.setCursor(6,1);
    lcd.print("G");
    lcd.print(greenValue);
    lcd.setCursor(11,1);
    lcd.print("B");
    lcd.print(blueValue);
    lcd.setCursor(15,1);
    lcd.write(2);//right arrow
  
    //case statment for RGBCustomOrder ******************************
    lcdKey = readLcdButtons();                   
    switch(lcdKey)                                
    {
      case btnTOPLEFT://increase selected value     
        switch (RGBCustomOrder)
        {
          case 0:                 //Increase Red
            ++redValue;
            if (redValue > 255)
              redValue = 255;
            break;
          case 1:                 //Increase Green
            ++greenValue;
            if (greenValue > 255)
              greenValue = 255;
            break;
          case 2:                 //Increase Blue
            ++blueValue;
            if (blueValue > 255)
              blueValue = 255;
            break;    
        }
        delay(buttonDelayFast);
        break;
      case btnTOPRIGHT://switch to next menu
        menuOrder = RGBCustomSave;
        delay(buttonDelaySlow);
        break;  
      case btnBOTTOMLEFT://decrease selected order      
        switch (RGBCustomOrder)
        {
          case 0:                 //Decrease Red
            --redValue;
            delay(buttonDelayFast);
            if (redValue < 1)
              redValue = 0;
            break;
          case 1:                 //Decrease Green
            --greenValue;
            delay(buttonDelayFast);
            if (greenValue < 1)
              greenValue = 0;
            break;
          case 2:                 //Decrease Blue
            --blueValue;
            delay(buttonDelayFast);
            if (blueValue < 1)
              blueValue = 0;
            break;    
        }
        break;
      case btnBOTTOMRIGHT://Move to next selection      
        ++RGBCustomOrder;
        delay(buttonDelaySlow);
        if (RGBCustomOrder > (RGBCustomTotal - 1))  
          RGBCustomOrder = 0;                     //sets it back to 0 if above is true
        break;
    }
    //Set Final Underline Cursor Position**********************************
    switch (RGBCustomOrder)
    {
      case 0:                 //Red
        lcd.setCursor(1,1);
        break;  
      case 1:                 //Green
        lcd.setCursor(6,1);  
        break;
      case 2:                 //Blue
        lcd.setCursor(11,1);
        break;  
    }
    //Save selected value to custom variable  
    redCustom = redValue;
    greenCustom = greenValue;
    blueCustom = blueValue;
    delay(menuDelay);
    break;    
  case RGBCustomSave://quick menu that sets RGB color back to RGBOrder **********************************************************
    EEPROM.update(EEredCustom, redCustom);
    EEPROM.update(EEgreenCustom, greenCustom);
    EEPROM.update(EEblueCustom,blueCustom);

    if(RGBOrder == 9){
      redValue = redCustom;
      greenValue = greenCustom;
      blueValue = blueCustom;
      EEPROM.update(EEredValue, redValue);
      EEPROM.update(EEgreenValue, greenValue);
      EEPROM.update(EEblueValue,blueValue);
    } 
    else{
      redValue = EEPROM.read(EEredValue);
      greenValue = EEPROM.read(EEgreenValue);
      blueValue = EEPROM.read(EEblueValue);
    }
    menuOrder = Backlight;
    break; 
  case Backlight://RGB Dimmer Screen *********************************************************************************************
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("<");
    lcd.setCursor(3,0);
    lcd.print("Backlight");
    lcd.setCursor(15,0);
    lcd.print(">");
    lcd.setCursor(0,1);
    lcd.write(1);//left arrow
    lcd.setCursor(6,1); 
    //case statment for RGBDimming *************************************
    lcdKey = readLcdButtons();               
    switch(lcdKey)                          
    {
      case btnTOPLEFT://goes back to options menu
        menuOrder = CustomColor;
        delay(buttonDelaySlow);
      break;
      case btnTOPRIGHT://goes back to Color Picker Screen
        menuOrder = PickColor;
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMLEFT://Decrease case**********
        --RGBDimOrder;
        delay(buttonDelaySlow);
        if (RGBDimOrder < 0)                 
           RGBDimOrder = (RGBDimTotal - 1);  
      break;
      case btnBOTTOMRIGHT://Increase case*********       
        ++RGBDimOrder;
        delay(buttonDelaySlow);
        if (RGBDimOrder > RGBDimTotal-1)  
          RGBDimOrder = 0;                    
      break;
    }

    switch(RGBDimOrder)//dim selection case****
    {
      case 0:
        RGBDim = 0;
        lcd.print("Off");
      break;
      case 1: 
        RGBDim = 3;
        lcd.print("Low");
      break;
      case 2:
        RGBDim = 20;
        lcd.print("Med");
      break;
      case 3:
        RGBDim = 100;
        lcd.print("High");
      break;  }
    lcd.setCursor(15,1);
    lcd.write(2);//right arrow
    EEPROM.update(EERGBDimOrder, RGBDimOrder); //saves selection to memory 
    EEPROM.update(EERGBDim, RGBDim);  
    delay(menuDelay);
  break;
  case Debug://*****************************************************************************************************************************
        if (Serial.available() > 35)  // Make sure the buffer has read in 32 bytes. On this revision that leaves the buffer on average with 13 bytes in per loop.
        {
          prev = current;  // Drop the current byte and read in what's first off the buffer
          current = Serial.read();  // Shift the first byte in off the buffer
          next = Serial.peek();  // Read the next byte off the buffer without modifying anything. Saves on a register.
    
          /* This sequence loops three bytes over and over, in sequence through the three memory locations. Useful 3-byte search. We are looking for
            a particular sequence, specifically FF 00 xx but if the ignition is off FF 00 is present part the way through the stream so that needs to be determined
            Phil relied on the next number being a non-zero but this doesn't always work. I have updated it to look for the Program Number 3rd so it always works*/
    
          if (prev == 0xFF && current == 0x00)
          {   
            if (next == customProgramNumber) //  Looks for non zero
            {
            digitalWrite(13, HIGH); // Turn the L LED on to show we are reading in good data.
              for (Position = 0; Position < 34; Position++) {
                dataStream[Position] = Serial.read();  //  Loop 31 bytes into the array straight off the buffer. This is a quick operation.
              }
            } 
          }
        }
      digitalWrite(13, LOW);      // Now we have finished reading in data and averaging it, turn the LED off
      digitalWrite(0, LOW);       // Drop the input. In theory if the transistor isn't biased wrong the buffer won't fill up. Pointless but eh.
    lcd.clear();
    lcd.setCursor(0,0);
  switch(debugStream)
  {
  case 0:
    switch(debugStreamOrder)
    {
    case 0:
      lcd.print(dataStream[0]);
      lcd.setCursor(3,0);
      lcd.print(dataStream[1]);
      lcd.setCursor(6,0);
      lcd.print(dataStream[2]);
      lcd.setCursor(9,0);
      lcd.print(dataStream[3]);
      lcd.setCursor(12,0); 
      lcd.print(dataStream[4]);
      
      lcd.setCursor(0,1);
      lcd.print(dataStream[5]);
      lcd.setCursor(3,1);
      lcd.print(dataStream[6]);
      lcd.setCursor(6,1);
      lcd.print(dataStream[7]);  
      lcd.setCursor(9,1);
      lcd.print(dataStream[8]);
      lcd.setCursor(12,1); 
      lcd.print(dataStream[9]);
      lcd.setCursor(15,1);
      lcd.print("1");
      lcd.setCursor(15,1);
      lcd.blink();
    break;
    case 1:
      lcd.print(dataStream[10]);
      lcd.setCursor(3,0);
      lcd.print(dataStream[11]);
      lcd.setCursor(6,0);
      lcd.print(dataStream[12]);
      lcd.setCursor(9,0);
      lcd.print(dataStream[13]);
      lcd.setCursor(12,0); 
      lcd.print(dataStream[14]);
      
      lcd.setCursor(0,1);
      lcd.print(dataStream[15]);
      lcd.setCursor(3,1);
      lcd.print(dataStream[16]);
      lcd.setCursor(6,1);
      lcd.print(dataStream[17]);  
      lcd.setCursor(9,1);
      lcd.print(dataStream[18]);
      lcd.setCursor(12,1); 
      lcd.print(dataStream[19]);
      lcd.setCursor(15,1);
      lcd.print("2");
      lcd.setCursor(15,1);
      lcd.blink();
    break;
    case 2:  
      lcd.print(dataStream[20]);
      lcd.setCursor(3,0);
      lcd.print(dataStream[21]);
      lcd.setCursor(6,0);
      lcd.print(dataStream[22]);
      lcd.setCursor(9,0);
      lcd.print(dataStream[23]);
      lcd.setCursor(12,0); 
      lcd.print(dataStream[24]);
      
      lcd.setCursor(0,1);
      lcd.print(dataStream[25]);
      lcd.setCursor(3,1);
      lcd.print(dataStream[26]);
      lcd.setCursor(6,1);
      lcd.print(dataStream[27]);  
      lcd.setCursor(9,1);
      lcd.print(dataStream[28]);
      lcd.setCursor(12,1); 
      lcd.print(dataStream[29]);
      lcd.setCursor(15,1);
      lcd.print("3");
      lcd.setCursor(15,1);
      lcd.blink();
    break;
    case 3:  
      lcd.print(dataStream[30]);
      lcd.setCursor(3,0);
      lcd.print(dataStream[31]);
      lcd.setCursor(6,0);
      lcd.print(dataStream[32]);
      lcd.setCursor(9,0);
      lcd.print(dataStream[33]);
      lcd.setCursor(12,0); 
      lcd.print(dataStream[34]);
      
      lcd.setCursor(0,1);
      lcd.print(dataStream[35]);
      lcd.setCursor(3,1);
      lcd.print(dataStream[36]);
      lcd.setCursor(6,1);
      lcd.print(dataStream[37]);  
      lcd.setCursor(9,1);
      lcd.print(dataStream[38]);
      lcd.setCursor(12,1); 
      lcd.print(dataStream[39]);
      lcd.setCursor(15,1);
      lcd.print("4");
      lcd.setCursor(15,1);
      lcd.blink();     
    break;
    }//debugStreamOrder Switch
  break;
  case 1:
    switch(debugStreamOrder)
    {
    case 0:
      lcd.print(dataStream[0], HEX);
      lcd.setCursor(2,0);
      lcd.print(dataStream[1], HEX);
      lcd.setCursor(4,0);
      lcd.print(dataStream[2], HEX);
      lcd.setCursor(6,0);
      lcd.print(dataStream[3], HEX);
      lcd.setCursor(8,0);
      lcd.print(dataStream[4], HEX);
      lcd.setCursor(10,0);
      lcd.print(dataStream[5], HEX);
      lcd.setCursor(12,0);
      lcd.print(dataStream[6], HEX);
      lcd.setCursor(14,0);
      lcd.print(dataStream[7], HEX);
      
      lcd.setCursor(0,1);
      lcd.print(dataStream[8], HEX);
      lcd.setCursor(2,1);
      lcd.print(dataStream[9], HEX);
      lcd.setCursor(4,1);
      lcd.print(dataStream[10], HEX);
      lcd.setCursor(6,1);
      lcd.print(dataStream[11], HEX);
      lcd.setCursor(8,1);
      lcd.print(dataStream[12], HEX);
      lcd.setCursor(10,1);
      lcd.print(dataStream[13], HEX);
      lcd.setCursor(12,1);
      lcd.print(dataStream[14], HEX);
      lcd.setCursor(14,1);
      lcd.print(dataStream[15], HEX);
    break;
    case 1:
      lcd.print(dataStream[16], HEX);
      lcd.setCursor(2,0);
      lcd.print(dataStream[17], HEX);
      lcd.setCursor(4,0);
      lcd.print(dataStream[18], HEX);
      lcd.setCursor(6,0);
      lcd.print(dataStream[19], HEX);
      lcd.setCursor(8,0);
      lcd.print(dataStream[20], HEX);
      lcd.setCursor(10,0);
      lcd.print(dataStream[21], HEX);
      lcd.setCursor(12,0);
      lcd.print(dataStream[22], HEX);
      lcd.setCursor(14,0);
      lcd.print(dataStream[23], HEX);
      lcd.setCursor(0,1);
      lcd.print(dataStream[24], HEX);
      lcd.setCursor(2,1);
      lcd.print(dataStream[25], HEX);
      lcd.setCursor(4,1);
      lcd.print(dataStream[26], HEX);
      lcd.setCursor(6,1);
      lcd.print(dataStream[27], HEX);
      lcd.setCursor(8,1);
      lcd.print(dataStream[28], HEX);
      lcd.setCursor(10,1);
      lcd.print(dataStream[29], HEX);
      lcd.setCursor(12,1);
      lcd.print(dataStream[30], HEX);
      lcd.setCursor(14,1);
      lcd.print(dataStream[31], HEX);
    break;
    }//debugStreamOrder Switch
  break;  
  }//debugStream Switch
  
    lcdKey = readLcdButtons();//read bottom buttons to change value               
        switch(lcdKey)                          
        {
        case btnTOPLEFT://Decrease case**********
          --debugStreamOrder;
          if (debugStreamOrder < 0)                 
             debugStreamOrder = 3;
          delay(buttonDelaySlow);  
        break;
        case btnTOPRIGHT://Increase case*********       
          ++debugStreamOrder;
          if (debugStreamOrder > 3)  
            debugStreamOrder = 0;   
          delay(buttonDelaySlow);                 
        break;  
        case btnBOTTOMLEFT://Decrease case**********                 
          debugStream = 0;
          delay(buttonDelaySlow);  
        break;
        case btnBOTTOMRIGHT://Increase case*********         
          debugStream = 1;   
          delay(buttonDelaySlow);                 
        break;
        }
    
    delay(menuDelay);
    Serial.flush();  // We done, flush any junk out of the buffer, so we can sieve for the header and begin again
    digitalWrite(0, HIGH); // push the read line high to begin reading in again
  break;
  case Settings://Settings Selection Screen (Scrolling)***********************************************************************************
    lcd.noCursor();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("<");
    lcd.setCursor(15,0);
    lcd.print(">");
    lcd.setCursor(0,1);
    lcd.write(1);//left arrow
    lcd.setCursor(15,1);
    lcd.write(2);//right arrow
    
    lcdKey = readLcdButtons();//read buttons for changing screens               
    switch(lcdKey)                          
    {
      case btnTOPLEFT://goes back 1 menu
        --settingsOrder;
        if (settingsOrder < 0)
          settingsOrder = (settingsTotal - 1);
        delay(buttonDelaySlow);
      break;
      case btnTOPRIGHT://goes forward 1 menu
        ++settingsOrder;
        if (settingsOrder > (settingsTotal - 1))
          settingsOrder = 0;
        delay(buttonDelaySlow);
      break;
    }
    
    switch(settingsOrder)
    {
      case 0://Splash Screen*************
        splashScreen = EEPROM.read(EEsplashScreen);
        lcd.setCursor(1,0);
        lcd.print("Splash Screen");
        lcd.setCursor(6,1);
        if(splashScreen == 1) lcd.print("Yes");
        else lcd.print("No");
        
        lcdKey = readLcdButtons();//read bottom buttons to change value               
        switch(lcdKey)                          
        {
        case btnBOTTOMLEFT://Decrease case**********
          --splashScreen;
          if (splashScreen < 0)                 
             splashScreen = 1;
          delay(buttonDelaySlow);  
        break;
        case btnBOTTOMRIGHT://Increase case*********       
          ++splashScreen;
          if (splashScreen > 1)  
            splashScreen = 0;   
          delay(buttonDelaySlow);                 
        break;
        }
        EEPROM.update(EEsplashScreen, splashScreen);
        delay(menuDelay);
      break;
      case 1://Gauge Update Speed*******************
        updateSpeedMenu = EEPROM.read(EEupdateSpeedMenu);
        updateSpeed = EEPROM.read(EEupdateSpeed);
        lcd.setCursor(2,0);
        lcd.print("Update Speed");
        switch(updateSpeedMenu)//Gauge Update Speed selection case****
        {         
          case 0:
            updateSpeed = slow;
            lcd.setCursor(6,1);
            lcd.print("Slow");
            delay(buttonDelaySlow);
          break;
          case 1: 
            updateSpeed = med;
            lcd.setCursor(5,1);
            lcd.print("Medium");
            delay(buttonDelaySlow);
          break;
          case 2:
            updateSpeed = fast;
            lcd.setCursor(6,1);
            lcd.print("Fast");
            delay(buttonDelaySlow);
          break;
          case 3:
            updateSpeed = ludi;
            lcd.setCursor(4,1);
            lcd.print("Ludicrous");
            delay(buttonDelaySlow);
          break;  
         }
        
        lcdKey = readLcdButtons();//read bottom buttons to change value               
        switch(lcdKey)                          
        {
        case btnBOTTOMLEFT://Decrease case**********
          --updateSpeedMenu;
          if (updateSpeedMenu < 0)                 
             updateSpeedMenu = 3;
          delay(buttonDelaySlow);  
        break;
        case btnBOTTOMRIGHT://Increase case*********       
          ++updateSpeedMenu;
          if (updateSpeedMenu > 3)  
            updateSpeedMenu = 0;  
        }     
        EEPROM.update(EEupdateSpeed, updateSpeed);
        EEPROM.update(EEupdateSpeedMenu, updateSpeedMenu);
        delay(menuDelay);
      break;
      case 2://Reset Screen*************
        lcd.setCursor(1,0);
        lcd.print("Reset Device?");
        lcd.setCursor(2,1);
        if (EEPROM.read(EEreset) == 255)
          lcd.print("Yes (Restart)");
        else lcd.print("    No");
        
        lcdKey = readLcdButtons();//read bottom buttons to change value               
        switch(lcdKey)                          
        {
        case btnBOTTOMLEFT://Decrease case**********
          EEPROM.update(EEreset, 0);
          delay(buttonDelaySlow);  
        break;
        case btnBOTTOMRIGHT://Increase case*********       
          EEPROM.update(EEreset, 255);   
          delay(buttonDelaySlow);                 
        break;
        }
        delay(menuDelay);
      break;
      case 3://Info Screen
      lcd.setCursor(1,0);
      lcd.print("Software Info");
      lcd.setCursor(1,1);
      lcd.print("V");
      lcd.print(softwareVersion);
      lcd.print(" Unit:");
      lcd.print(EEPROM.read(EEModelNumber));
      delay(menuDelay);
    }//Settings Order Case Switch End
  break;
  }//Root Menu System Case Switch End

}//Loop End
/* 
lcdKey = readLcdButtons();

    switch (lcdKey)
    {
      case btnTOPLEFT:
        delay(buttonDelaySlow);
      break;
      case btnTOPRIGHT:
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMLEFT:
        delay(buttonDelaySlow);
      break;
      case btnBOTTOMRIGHT:
        delay(buttonDelaySlow);
      break;
    }
    /*
/*center of 16 line screen
         ________________
               1        =7
               22       =7
              333       =6
              4444      =6
             55555      =5
             666666     =5
            7777777     =4
            88888888    =4
           999999999    =3
           1010101010   =3
          11111111111   =2
          121212121212  =2
         1313131313131  =1
         14141414141414 =1
        151515151515151 =0
        1616161616161616=0
 */
  /*
    //Debug bamf- /t is insert tab. Prints binary version (marking print() with x,BIN doesn't print preceding zeroes), hex and denary. Needs a lot of Terminal window lines.
    for (Position=0;Position<31;Position++){
    Serial.print("Byte ");
    Serial.print(Position);
    Serial.print("\t");
    Serial.print(bitRead(dataStream[Position],7));
    Serial.print(bitRead(dataStream[Position],6));
    Serial.print(bitRead(dataStream[Position],5));
    Serial.print(bitRead(dataStream[Position],4));
    Serial.print(bitRead(dataStream[Position],3));
    Serial.print(bitRead(dataStream[Position],2));
    Serial.print(bitRead(dataStream[Position],1));
    Serial.print(bitRead(dataStream[Position],0));
    Serial.print("\t");
    Serial.print(dataStream[Position],HEX);
    Serial.print("  \t");
    Serial.print(dataStream[Position]);
    Serial.println("\e[K");  // clear to end of line
    }
    //End debug
  */
  /*
    Glossary
      From what I have researched, this is the framing- it isn't correct for some on the GTA. Still working on it. Jeep Data points updated by Nick Risley. 
    Byte   : 4.0L Jeep              / 4 cyl renix (R21 turbo)   / 13,32 2.0 F3R AMC GTA (confirmed)
    Byte 0 : Program Version 1 & 2  / Program Version           / Program Version
    Byte 1 : PROM Version    3 & 4  / PROM Version              / PROM version
    Byte 2 : Calibration code 5 & 6 / Engine run flags (bin)    / Switch values. C = engine off, nothing selected. 4 = off idle stop
    Byte 3 : MAP                    / MAP                       / MAP
    Byte 4 : CLT                    / CLT                       / CLT
    Byte 5 : IAT                    / IAT                       / IAT
    Byte 6 : Battery Volts          / Battery Volts             / Battery Volts
    Byte 7 : Oxygen sensor          / Oxygen sensor
    Byte 8 : RPM LO byte            / RPM LO Byte               / RPM LO
    Byte 9 : RPM HI byte            / RPM HI byte               / RPM HI
    Byte 10: 9th & 10th PROM        / Inj pulsewidth LO byte    / Injector Pulse LO
    Byte 11: 7th & 8th PROM         / Inj pulsewidth HI byte    / Injector Pulse HI
    Byte 12: Throttle position      / Knock sensor
    Byte 13: Advance degrees        / Advance degrees           / Throttle Position
    Byte 14: *unknown               / Control
    Byte 15: *ECT_ADC               / Turbo correction value
    Byte 16: Baro before start      / unknown
    Byte 17: *Mode                  / Failure flags  %
    Byte 18: Loop/Exhaust, *ALFA - (byte/10=air ratio) {14.7:1) / Road speed
    Byte 19: Pulse width            / throttle stop
    Byte 20: *..?                   / Throttle position
    Byte 21: Throt Sw/Fuel Sync     / System Board Error flags %%
    Byte 22: *Advance + knock mask  / ..?
    Byte 23: *warmup                / warmup
    Byte 24: ST Fuel Trim           / Stored errors %%%
    Byte 25: *temperature adc errors / recent errors %%%
    Byte 26: LT Fuel Trim           / Advance + knock correction
    Byte 27: Knock                  / Barometric pressure before start
    Byte 28: *..?                   / Adjustment off MAP
    Byte 29: A/C Sw/Request         / Switch values %%%%         / Switches possibly. changes when air is selected, makes sense
    Byte 30: 11th & 12th PROM       / Errors %%%%%
    For bitRead(x,y) x=value byte, y= position where bit 7 = 128/msb, bit 0 = 1/lsb
    % 7:Bad injector/6:TDC unable to be found/5:Roadspeed/4:Fuel pump/3:MAP bad/2:Knock sensor bad/1:Crank Pos Sens bad/0:unkown
    %% 7:Bad PROM version/6:Injector circuit fault/5:CPS variance/4:EEPROM CRC/3:ROM CRC/2~0:unknown
    %%% 7:TPS open/6:TPS short/5:CTS open/4:CTS short/3:IAT open/2:IAT short/1:EGR open/0:EGR short
    %%%% 7:Unknown/6:unknown/5:Sync(cam pos on 6pot?)/4:unknown/3:starter/2:AC engage/1:AC request/0:PARK (gears)
    %%%%% 7:Ignition module bad/6~4:unknown/3:oxygen relay/2:latch relay/1:aircon relay/0:unknown
  */
