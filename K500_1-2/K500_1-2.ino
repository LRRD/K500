//////////////////////////////////////////////////////////////////////////////
// K500 1/2" Omega Paddlewheel with 1/2" NPT to 3/4" Barb Connectors/Tubing //
//////////////////////////////////////////////////////////////////////////////
// Updates and Changes  //////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/*

  Mason Parrone 4/2018 Code for digital flow controller redesign for use with GRI brushless submersible pump (0-5v speed control signal)
  Arduino IDE version 1.8.7
  K500 Code Version 1.12.0
  Optical Encoder:        62AG11-L5-060C
  LCD Display:            NHD-0216K3Z-NSW-BBW-V3            
  DAC:                    MCP 4725 Adafruit version
  Arduino:                Arduino Nano V3 (328P 5V)
  Pump:                   EX489-3-449 
  Calibration spreadsheets and supporting documentation located in parent folder on Sandbar titled: "Data Documents Installers"             
  Stripped down encoder code to bare bones and moved all inputs to digital pins that aren't PWM. This previously used the analog pins that we may need for the DAC in the future.
  Added in LCD code from RS-232 display code - created using hex codes from product data sheet
  Added in DAC code for MCP 4725 Adafruit version
  Volumetric test input into excel: A,B,C Flow values found and input
  Implemented ml/s calculation and readout
  Copied total L and elapsed time from Alix controller code
  Added paddlewheel code as interrupt on pin 2
  Added PID loop and controls for paddlewheel flow measurement and close loop
  Moved PID loop calculation into refresh LCD. This gives some time to between last output adjustment and next calculation
  Kp values scheduled for high and low flows
  Added timer back onto display
  Added switch case for menu control, broke functions into multiple smaller functions for better menu control and display
  Added hydrograph menu and other hydrograph functions
  Optimized hydrograph code, added instructions to add new hydrographs, and imported hydrographs from old Alix controller
  Added paddlewheel indicator in top right corner, displayed when PID loop is active
  Bug fix, paddlewheel indicator only appears on menu 1 and 6 now
  Major PID loop changes
  Removed I and D in PID loop
  Changed P to P scheduling where P values are higher at high flow and lower at low flow
  Recalculated volumetric curve for encoder count vs ml/s and duration vs ml/s
  Duration vs ml/s is only calculated using values from ~55ml/s - ~185ml/s
  Encoder count vs ml/s is only calculated using values from 0 ml/s - ~185ml/s
  Added time since last duration reading, this is used to switch back and forth between encoder count regression and P control loop
  All flows below 55 ml/s are described as less than 55ml/s rather than explicitly stated, this prevents user confusion. They aren't necessarily accurate in that range
  0 ml/s is explicitly stated
  Walking paddle character fixed
  P control loop only runs when setpoint >= 55ml/s and time since last paddlewheel reading is < 400 ms.
  Exponential smoothing was used to reduce raw duration reading volatility
  Encoder click timing was stepwise, now it is continuously variable
  Catch case for setpoint less than 1 ml, output zeroed
  Changed to Omega Paddlewheel
  Only runs in closed loop above minFlow
  Click timing removed, one click = 1 ml, this works well enough because there are shortcuts to zero and max flow in the menu
  Added catch cases for if the setpoint is above 0 ml/s and no flow is detected, prevents pump running dry or if sensor unplugged
  Changed timing function to count micros instead of millis, theoretically increased input resolution 250x
  Rewrote error sequence to simplify it
  Bug fixes for startup
  Bug fixes for hydrographs
  Bug fixes for standard flow setpoint
  E! is error code for an issue with the paddlewheel input reading
  Made alpha constant to help with over-adjustments
  Reworked KP gain scheduling - KP based on error magnitude, and calculated at setup, programmer chooses a "low error magnitude" and "high error magnitude" and corresponding KP values at each
  Hydrograph indicator
  Removed "Flow Zeroed" text, there was no "Flow Max" text and it is self-explanatory
  Actual flow displays next to setpoint so user can see how it is dynamically adjusting to maintain desired flow
  Copied 1/2" code, duration and flow limit values changed for 3/8" code
  Removed unused variables and sections of code
  Added atomic.h and atomic block routine to prevent erroneous shutoffs of flow
  Changed order of bootup check during encoder rotation check, bug fix for case that only happens on first click after bootup
  Changed timer to roll over after 99 hours
  
*/

////////////////////////////////////////////////////////////////////////////
// Libraries and Definitions ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#include <Wire.h>//Include the Wire library to talk I2C
#define MCP4725_ADDR 0x62 //Unique address for MCP4725 Adafruit version 
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

////////////////////////////////////////////////////////////////////////////
// Magic Numbers  & Global Variables ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

//Encoder
static uint8_t cw_gray_codes[4] = { 2, 0, 3, 1 }; 	//The sequence of gray codes in the increasing and decreasing directions.
static uint8_t ccw_gray_codes[4] = { 1, 3, 0, 2 };  //Gray code sequence
static uint8_t previous_gray_code = 0;				      //The gray code we last read from the encoder.
int released;                                       //Used for encoder button presses

//Input & Output variables
uint32_t last_display_update_ms = 0;				        //Used to refresh the lcd Screen
uint32_t last_display_update_ms2 = 0;               //Used to run hydrograph
const uint8_t encoderswitch = 7;					          //Encoder poin reference
const uint8_t channelB = 6;							            //Encoder pin reference
const uint8_t channelA = 4;							            //Encoder pin reference
const uint8_t led = 3;								              //LED pin reference
const uint8_t led2 = 5;								              //LED pin reference
const uint8_t paddlewheel = 2;                      //Pin assignment method that uses the least space

//Update variables
int lastsetpoint = 0;                               //Last setpoint
uint32_t lastzero = 0;                              //Millis since setpoint was last at zero, assuming it isnt at 0 now
uint16_t lastLiter = 0;                             //Last total liter pumped written to display

//Math variables
float duraVal = 5367.894844;                        //Power function regression (x: duration between pulses, y: ml/s) ax^b
float durbVal = -.953814;                           //Power function regression (x: duration between pulses, y: ml/s) ax^b
float kpA;                                          //Linear regression (x: error magnitude, y: desired KP value) - Populated at startup using errorMAG at KP at errorMag
float kpB;                                          //Linear regression (x: error magnitude, y: desired KP value) - Populated at startup using errorMAG at KP at errorMag
float duration = 300.0;                             //Raw data in from paddlewheel sensor
int setpoint = 0;                                   //Setpoint in desired ml/s
float input = 0.0;                                  //Input value from paddlewheel sensor
float error = 0.0;                                  //Error value between measured ml/s and setpoint in ml/s
float output = 0.0;                                 //Output of PID Loop to DAC
uint16_t index = 0;                                 //Used to step through hydrograph values
uint16_t indexmax = 0;                              //Used to define length of each hydrograph
uint32_t hydroStartTime = 0;                        //Hydrograph timer shows how uint32_t the hydrograph has been running
uint32_t totalSec = 0;                              //Total seconds the Arduino has been running
volatile uint32_t nowTime = 0;                      //Time marked during the interrupt routine
float smoothed = 300.0;                             //Duration value following exponentional smoothing process
float lastsmoothed = 300.0;                         //Last duration value following exponentional smoothing process
volatile float timesince = 0.0;                              //ms since last paddlewheel pulse
float timesince2 = 0.0;                             //Variable for time since moving from 0 flow to flow
uint16_t refreshtime = 500;                         //Time in ms between each refresh/calculation

//Tuning variables
float alpha = 0.6;                                  //Exponential smoothing constant, smaller makes it more smooth but increases settling time after change in setpoint
float KP;                                           //KP variable for closed loop
float errorMAGlow = 5.0;                            //Magnitude of error considered to be low
float errorMAGhigh = 100.0;                         //Magnitude of error considered to be high
float KPatMAGlow = 0.1;                             //KP value to be used at errorMAGlow
float KPatMAGhigh = 0.75;                           //KP value to be used at errorMAGhigh

//Limit variables
const uint16_t setmax = 410;                        //Max setpoint
const uint8_t minFlow = 55;                         //Min setpoint
const uint16_t maxhydro = 120;                      //Max number of 30s intervals in a hydrograph (Increasing uses lots of memory)
uint16_t slowturn = 300;                            //Ms length of slowest paddlewheen rotation duration
uint16_t disconnected = 10000;                      //Ms with no input cutoff point
const uint8_t menuhigh = 13;                        //Change menuhigh to the new highest menu # after adding hydrographs (Increase by 1 for each added hydrograph)

//Display variables
uint16_t flow = 0;                                  //Used to display flow in ml/s
uint16_t totalLiter = 0;                            //Used to display total L of water pumped
bool indicator = false;                             //Used to switch between + and * for hydrograph indicator
uint32_t cumulativeVolume = 0;                      //Total amount of ml/s of water pumped since last restart
bool bootup = true;                                 //Used for a one time run on startup, different than setup()
uint32_t runtime = 0;                               //Used for timer
uint16_t hour;                                      //""
uint16_t minute;                                    //""
uint16_t second;                                    //""

//Other variables
uint8_t menu = 0;									                  //Switch case variable for menu controls
uint16_t clicker = 500;								              //Ms delay for all button presses and menu navigation rotations
bool setinput = false;								              //Used to run hydrographs
volatile bool inputread = false;			              //Global flag to indicate paddlewheel reading
bool recentDC = false;                              //Recent disconnect on paddlewheel input bool


//////////////////////////////////////////////////////////////////////////////////
// Hydrographs ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//Each array is a new hydrograph, values are flow setpoints at 30s spacings.
//Max hydrograph length is .5 hr.
//Hydrgraph is constrained to close loop limits
//To make a new hydrograph define it with a unique identity as a const uint8_t as shown below
//Ctrl + F (Find) " Case 8 : " or travel to code line ~877
//Copy Case 8 and paste it below the highest number case
//Replace all case # references with the new current highest case number
//Replace text with hydrograph name and unique hydrograph identity
//Detailed instructions are in the case creation
const uint8_t flashy[] = { 40, 66, 105, 100, 90, 85, 82, 76, 66, 60, 58, 50, 45, 40, 25, 25 }; //8 Minutes
const uint8_t classichigh[] = { 29, 38, 44, 54, 58, 67, 74, 80, 91, 102, 110, 117, 115, 113, 102, 99, 93, 84, 82, 80, 78, 73, 69, 66, 62, 58, 55, 51, 47, 44, 40, 37, 33, 25, 25 }; //17.5 Minutes
const uint8_t megaflood[] = { 40, 68, 90, 110, 130, 150, 160, 170, 170, 165, 160, 155, 150, 146, 139, 132, 125, 121, 113, 107, 96, 89, 82, 75, 68, 60, 53, 51, 51, 45 }; //16.5 Minutes
const uint8_t classic[] = { 25, 30, 48, 56, 66, 82, 95, 106, 92, 80, 75, 72, 69, 65, 62, 60, 57, 55, 52, 50, 47, 45, 41, 38, 35, 31, 25, 25, 25 }; //14.5 Minutes
const uint8_t flashyhigh[] = { 25, 126, 160, 155, 120, 102, 80, 60, 50, 40, 30, 25, 25 }; //6.5 Minutes
const uint8_t regulated[] = { 40, 50, 50, 70, 70, 70, 70, 70, 70, 50, 50, 70, 70, 70, 70, 70, 50, 50, 50, 50, 70, 70, 70, 70, 50, 50, 50, 50, 50, 70, 70, 70, 70, 50, 50, 50, 70, 70, 70 }; //19.5 Minutes
uint16_t currenthydro[maxhydro] = {}; //The longest hydrograph can be .5 hour (60, 30-Second intervals) I needed a number for this, Increasing maxhydro uses up loads of memory


////////////////////////////////////////////////////////////////////////////
// DAC Controls ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void writePWM2() //Function to control analog voltage out from DAC
{
  if (setpoint < 1) //If point is set to not flow, override all calculations to ensure zero flow
  {
    output = 0;
  }
  else //Flow is desired, bypass large zone of output that does nothing
  {
    output = constrain(output, 1500, 4095);
  }
  int pwm = round(output); //Copy output to pwm
  output = constrain(output, 0, 4095); //Limits of the DAC are 0 - 4095
  Wire.beginTransmission(MCP4725_ADDR); //Beging communications with DAC
  Wire.write(64);                       //Command to update the DAC
  Wire.write(pwm >> 4);                 //Write PWM Value with the 8 most significant bits...
  Wire.write((pwm & 15) << 4);          //Add the 4 least significant bits...
  Wire.endTransmission();               //End communication with DAC
}


////////////////////////////////////////////////////////////////////////////
// LCD Functions ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// turn on display
void displayOn() {
  Serial.write(0xFE);
  Serial.write(0x41);
}

// move the cursor to the home position on line 2
void cursorLine2() {
  Serial.write(0xFE);
  Serial.write(0x45);
  Serial.write(0x40); //Hex code for row 2, column 1
}

// move the cursor to the home position on line 2
void cursorTopRight() {
  Serial.write(0xFE);
  Serial.write(0x45);
  Serial.write(0x0F); //Hex code for row 1, column 16
}

// move the cursor to the home position on line 2
void cursorBottomRight() {
  Serial.write(0xFE);
  Serial.write(0x45);
  Serial.write(0x4F); //Hex code for row 2, column 16
}

// move the cursor to the home position
void cursorHome() {
  Serial.write(0xFE);
  Serial.write(0x46);
}

// clear the LCD
void clearLCD() {
  Serial.write(0xFE);
  Serial.write(0x51);
}

// backspace and erase previous character
void backSpace(int back) {
  for (int i = 0; i < back; i++)
  {
    Serial.write(0xFE);
    Serial.write(0x4E);
  }
}

// move cursor left
void cursorLeft(int left) {
  for (int i = 0; i < left; i++)
  {
    Serial.write(0xFE);
    Serial.write(0x49);
  }
}

// move cursor right
void cursorRight(int right) {
  for (int i = 0; i < right; i++)
  {
    Serial.write(0xFE);
    Serial.write(0x4A);
  }
}

// set LCD contrast
void setContrast(int contrast) {
  Serial.write(0xFE);
  Serial.write(0x52);
  Serial.write(contrast); //Must be between 1 and 50
}

// turn on backlight
void backlightBrightness(int brightness) {
  Serial.write(0xFE);
  Serial.write(0x53);
  Serial.write(brightness); //Must be between 1 and 8
}

//Clear numbers on top line
void clearDataTopLine() {
  cursorHome();
  Serial.print(F("               ")); //Write 15 blanks and don't alter walking paddle
}

//Clear numbers on bottom line
void clearDataBottomLine() {
  cursorLine2();
  Serial.print(F("                ")); //Write 16 blanks
}


////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine (ISR)  ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void timing() //Runs when ISR is called from interrupt pin input
{
  duration = micros() - nowTime;    //Duration of paddlewheel rotation (microseconds)
  nowTime = micros();               //Assign time
  inputread = true;                 //Boolean to indicate new input was read
}


////////////////////////////////////////////////////////////////////////////
// Encoder handling  ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
static void check_encoder() // Look for encoder rotation by observing graycode on channel A & B
{
  int gray_code = ((digitalRead(channelA) == HIGH) << 1) | (digitalRead(channelB) == HIGH);
  if (bootup) //On bootup, first time check encoder runs it thought it was clicked towards increase flow
  {
    setpoint = 0;
    bootup = false;
  }
  else
  {
    if (gray_code != previous_gray_code)       //Encoder clicked in a direction
    {
      if (gray_code == cw_gray_codes[previous_gray_code])     //Knob twist CW
      {
        if (setpoint < minFlow) //At zero flow setpoint
        {
          setpoint = minFlow;   //Jump to min flow setpoint
        }
        else //Not at zero flow setpoint
        {
          setpoint++;   //Increase setpoint by 1
        }
      }

      else if (gray_code == ccw_gray_codes[previous_gray_code])     //Knob twist CCW
      {
        if (setpoint == minFlow)  //At min flow setpoint
        {
          setpoint = 0;           //Jump to zero flow setpoint
        }
        else    //Not at min
        {
          setpoint--;     //Decrease setpoint by 1
        }
      }
      previous_gray_code = gray_code; //Stores current gray code for future comparison
      setpoint = constrain(setpoint, 0, setmax); //Flow maxes at setmax
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
/// Screen Refresh   /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void refresh_lcd()
{
  uint32_t elapsed_ms = millis() - last_display_update_ms; //ms since last display update
  if ((last_display_update_ms == 0) || (elapsed_ms >= refreshtime)) //Periodically refresh display
  {
    compute(); //Compute adjustment
    writePWM2(); //Adjust flow
    computedisplay(); //Refresh display
    last_display_update_ms = millis(); //Assign current time to last update time
    timer();  //Display timer
    printtimer();
    uint16_t currentMls = (setpoint); //Ml/s / 2 because this runs every half second
    cumulativeVolume = (cumulativeVolume + currentMls); //Add current mls to total mls
    literdisplay(); //Display total liter
    walkingpaddle(); //Display walking paddle
  }
}

//Refresh without display
void refresh_nolcd()
{
  uint32_t elapsed_ms = millis() - last_display_update_ms;
  if ((last_display_update_ms == 0) || (elapsed_ms >= refreshtime)) //Periodically refresh display
  {
    compute(); //Compute adjustment
    writePWM2(); //Adjust flow
    last_display_update_ms = millis();  //Assign current time to last update time
    uint16_t currentMls = (setpoint); //Ml/s / 2 because this runs every half second
    cumulativeVolume = (cumulativeVolume + currentMls); //Add current mls to total mls
  }
}


//////////////////////////////////////////////////////////////////////////////
/// Hydrograph Refresh ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void hydrorefresh()
{
  uint32_t elapsed_ms2 = millis() - last_display_update_ms2; //ms since last setpoint change
  if (elapsed_ms2 >= 30000) //Periodically refresh display
  {
    last_display_update_ms2 = millis(); //Assign current time to last update time
    if (index < (indexmax - 1)) //Still stepping our way through the hydrograph every 30s
    {
      index++;
    }
    else if (index >= (indexmax - 1)) //The last flow setpoint is complete for the hydrograph
    {
      cursorHome(); //Update display when hydrograph completes
      Serial.print(F("<  Hydrograph  >"));
      cursorLine2();
      Serial.print(F("<   Complete   >"));
      delay(2000);
      menu = 0; //Return to main display
      setpoint = 0;
      output = 0;
      for (uint16_t i = 0; i < maxhydro; i++) //Clear currenthydro by filling it with 0's
      {
        currenthydro[i] = 0;
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
/// Liter Calc & Display  ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void literdisplay()
{
  totalLiter = ((cumulativeVolume) / 1000); //Convert milliliter to liter
  if (totalLiter > 9999) //Roll over total liter
  {
    cumulativeVolume = 0;
    totalLiter = 0;
  }
  if (totalLiter != lastLiter)
  {
    cursorBottomRight();
    cursorLeft(4);
    Serial.print(totalLiter);
    Serial.print(F("L"));
    lastLiter = totalLiter;
  }
}


//////////////////////////////////////////////////////////////////////////////
/// Timing Function  /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void timer()
{
  if (!setinput) //Running time during normal setpoint operation
  {
    totalSec = (millis() - runtime) / 1000;   //Time is has been powered on for convert to seconds
  }
  else  //Hydrograph time during hydrograph operation
  {
    totalSec = (millis() - hydroStartTime) / 1000; //Total time - time at start of hydro (seconds)
  }
  
  hour = totalSec / 3600;           //Seconds in an hour
  uint16_t remainder = totalSec % 3600;               //Remainder
  minute = remainder / 60;           //Seconds in a minute
  remainder = remainder % 60;                 //Remainder
  second = remainder;                //Seconds

  if (hour > 99)
  {
    runtime = millis();
    timer();
  }
}

void printtimer()
{
  cursorLine2();                              //Set cursor to line 2
  if (hour < 10)
  {
    Serial.print(F("0"));
    Serial.print(hour);
  }
  else
  {
    Serial.print(hour);
  }
  Serial.print(F(":"));
  if (minute < 10)
  {
    Serial.print(F("0"));
    Serial.print(minute);
  }
  else
  {
    Serial.print(minute);
  }
  Serial.print(F(":"));
  if (second < 10)
  {
    Serial.print(F("0"));
    Serial.print(second);
  }
  else
  {
    Serial.print(second);
  }
}

//////////////////////////////////////////////////////////////////////////////////
// Paddlewheel Indicator /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void walkingpaddle()
{
  cursorTopRight();   //Move cursor
  if ((setpoint > 0) && (timesince < slowturn)) //Conditions met for P control loop for flow
  {
    if (indicator) //Flip flop between two symbols
    {
      Serial.print(F("+"));
    }
    else
    {
      Serial.print(F("*"));
    }
    indicator = !indicator;
  }
  else
  {
    Serial.print(F(" "));
  }
}

//////////////////////////////////////////////////////////////////////////////
/// P Control Loop  //////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void compute() //How much do we adjust the output by?
{
  //What is the setpoint?
  //What is the measured?
  //What is the error?
  //What is the output?

  //Setpoint
  if (setinput) //Hydrograph chooses setpoint
  {
    setpoint = currenthydro[index]; //Read current hydro index value
    setpoint = constrain(setpoint, minFlow, setmax); //No zero flow values allowed, constrain setpoint within limits
  }
  else    //User chooses setpoint
  {
    setpoint = constrain(setpoint, 0, setmax); //Zero flow values allowed, constrain setpoint within limits
  }

  //Moving from no flow, to flow
  if ((lastsetpoint == 0) && (setpoint > 0)) //Reset time since last pulse
  {
    lastzero = millis();
  }
  lastsetpoint = setpoint;  //Assign current setpoint to lastsetpoint for updating
  
  timesince = (micros() - nowTime) / 1000.0;   //Time since last pulse is converted from microseconds
  timesince2 = millis() - lastzero;  //Time since last zero

  if (((timesince > disconnected) && (timesince2 > disconnected)) && (setpoint > 0)) //Move than x sec since last pulse && more than x sec since moving from 0 flow to > 0 flow
  {
    /*
     Troublesome troubleshooting - flow would stop and error sequence triggered at random time intervals hours into operation
     Volatile variable 'nowTime' was updating as it was being read
     For volatile variables longer than one byte such as nowTime (uint32_t) 8 bytes,
     Variables that call that reference value get read one byte at a time, and the interrrupt can change the rest of the values while it is reading the first
     Atomic block pauses the ISR while we read timesince, which references nowTime
     It doesnt always use atomic block, or it would miss many PW pulses
     Atomic block is used only to double-check and eliminate possibility of false error state
     It misses one, or a few pulses while atomically blocked, but recovers quickly
     */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)     //Block ISR and double-check error state before triggering sequence
    {
      timesince = (micros() - nowTime) / 1000.0;   //Time since last pulse is converted from microsecond
    }
    if (timesince > disconnected) //If there really is an error, trigger sequence
    {
      if (!recentDC)
      {
        setpoint = 0; //Set it to zero and display error code
        output = 0;
        writePWM2();
        clearLCD();
        cursorHome();
        Serial.print(F("Sensor Error")); //Let the user know that there is an issue with the sensor
        cursorLine2();
        Serial.print(F("No Sensor Input"));
        delay(5000);
        clearLCD();
        cursorHome();
        recentDC = true;
        menu = 0;
      }
    }
  }

  //Input
  if (inputread) //New duration from the paddlewheel received
  {
    //Time of last rotation
    duration = constrain(duration, 0, 500000); //Increase this next
    smoothed = alpha * (duration / 1000.0) + (1 - alpha) * lastsmoothed; //Exponential smoothing
    lastsmoothed = smoothed; //Assign current to last for future calculation
    inputread = false; //Reset bool
    input = duraVal * pow(smoothed, durbVal); //Power function regression
    if (recentDC) //Error resolved, clear code and reset bool
    {
      cursorTopRight(); //Erase error code
      cursorLeft(3);
      Serial.print(F("  "));
      recentDC = false; //Toggle boolean
    }
  }
  else
  {
    input = 0; //Power function regression
  }

  //Error
  error = (setpoint - input); //Not abs() sign matters
  KP = abs(error) * kpA + kpB; //KP gain scheduling based on magnitude of error
  KP = constrain(KP, KPatMAGlow, KPatMAGhigh);  //Constrain KP gain

  //Output
  float adjustment = round(error * KP); //Output adjustment calc
  output += adjustment; //Output adjustment
}


//////////////////////////////////////////////////////////////////////////////
/// Display Setpoint /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void computedisplay()
{
  clearDataTopLine(); //Update setpoint on top line
  cursorHome();
  Serial.print(setpoint);
  Serial.print(F(" ml/s "));
  Serial.print(input, 0);
  if ((setinput) && (!recentDC))
  {
    cursorTopRight();
    cursorLeft(2);
    Serial.print(F("H")); //Hydrograph indicator
  }
  if (recentDC)   //Paddlewheel error, display that
  {
    cursorTopRight();
    cursorLeft(4);
    Serial.print(F("E!"));
  }
}


//////////////////////////////////////////////////////////////////////////////////
// Menu Navigator ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void navigate()
{
  // Get the Gray-code state of the encoder.
  int gray_code = ((digitalRead(channelA) == HIGH) << 1) | (digitalRead(channelB) == HIGH);
  if (gray_code != previous_gray_code)   //Encoder clicked in a direction
  {
    if (gray_code == cw_gray_codes[previous_gray_code])     //Knob twist CW
    {
      delay(clicker);  //Standard delay
      if (menu == 5) //User cannot navigate to hydrograph (submenu) without clicking on the right menu
      {
        menu = -1;  //Adds one after
      }
      else if (menu == menuhigh) //User cannot navigate back to setpoint control without clicking 'Back'
      {
        menu = 6;  //Adds one after
      }
      menu++;
    }

    else if (gray_code == ccw_gray_codes[previous_gray_code])     //Knob twist CCW
    {
      delay(clicker);
      //Create a scrolling (submenu) effect (Its actually just one big menu)
      if (menu == 7) //User cannot navigate out of hydrograph (submenu) without clicking 'Back'
      {
        menu = (menuhigh + 1);  //Adds one after
      }
      menu--;
    }
    previous_gray_code = gray_code; //Stores current gray code for future comparison
  }
}


//////////////////////////////////////////////////////////////////////////////////
// Menu  /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void menuselect()
{
  switch (menu)
  {
    //Manual control of setpoint and timer display
    case 0:
      clearLCD();
      cursorHome();
      setinput = false;
      compute();
      cursorHome();
      Serial.print(setpoint);
      Serial.print(F(" Ml/s "));
      while (menu == 0)
      {
        check_encoder();
        refresh_lcd();
        if (digitalRead(encoderswitch) == LOW)
        {
          delay(clicker);
          menu++;
        }
      }
      break;

    //Main menu screen
    case 1:
      clearLCD();
      cursorHome();
      Serial.print(F("<     Menu     >"));
      cursorLine2();
      Serial.print(F("<   Selector   >"));
      while (menu == 1)
      {
        refresh_nolcd();
        navigate();
      }
      break;

    //Flow reset
    case 2:
      clearLCD();
      cursorHome();
      Serial.print(F("<     Zero     >"));
      cursorLine2();
      Serial.print(F("<     Flow     >"));
      while (menu == 2)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setpoint = 0;
          output = 0;
          delay(clicker);
          menu = 0;
        }
      }
      break;

    //Maximum Flow
    case 3:
      clearLCD();
      cursorHome();
      Serial.print(F("<   Maximize   >"));
      cursorLine2();
      Serial.print(F("<     Flow     >"));
      while (menu == 3)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setpoint = setmax;
          delay(clicker);
          menu = 0;
        }
      }
      break;


    //Hydrograph Selector
    case 4:
      clearLCD();
      cursorHome();
      Serial.print(F("<  Hydrograph  >"));
      cursorLine2();
      Serial.print(F("<   Selector   >"));
      while (menu == 4)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          delay(clicker);
          menu = 8;
        }
      }
      break;

    //Controller Information
    case 5:
      clearLCD();
      cursorHome();
      Serial.print(F("<  Controller  >"));
      cursorLine2();
      Serial.print(F("<  Information >"));
      while (menu == 5)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          clearLCD();
          cursorHome();
          Serial.print(F("  K500  Closed  "));
          cursorLine2();
          Serial.print(F("Loop  Controller"));
          delay(2500);

          clearLCD();
          cursorHome();
          Serial.print(F("v1.12.0  Written"));
          cursorLine2();
          Serial.print(F("By:  MAP  6/7/18"));
          delay(2500);

          clearLCD();
          cursorHome();
          Serial.print(F("      LRRD      "));
          cursorLine2();
          Serial.print(F("WWW.EMRIVER.COM "));
          delay(2500);
          menu = 0;
        }
      }
      break;

    //Running hydrograph menu item
    case 6:
      last_display_update_ms2 = millis();
      clearLCD();
      cursorHome();
      setinput = true;
      hydroStartTime = millis(); //Assign current time as start time of hydrograph
      while (menu == 6)
      {
        hydrorefresh();
        refresh_lcd();
        if (digitalRead(encoderswitch) == LOW) //Button press ends hydrograph
        {
          setinput = false;
          cursorHome();
          Serial.print(F("<  Hydrograph  >"));
          cursorLine2();
          Serial.print(F("<   Stopped    >"));
          delay(2500);
          menu = 0; //Return to main display
          currenthydro[0] = {};
          setpoint = 0;
          output = 0;
        }
      }
      break;

    //Hydrograph Back Selector
    case 7:
      clearLCD();
      cursorHome();
      Serial.print(F("<     Back     >"));
      cursorLine2();
      Serial.print(F("<              >"));
      while (menu == 7)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          delay(clicker);
          menu = 4;
        }
      }
      break;

    //Flash Flood Hydrograph
    //When making new hydros, replace '8' references with new highest case number
    case 8:
      clearLCD();
      cursorHome();
      //Replace text with desired text
      Serial.print(F("< Flash  Flood >"));
      cursorLine2();
      //Replace text with desired text
      Serial.print(F("<  8  Minutes  >"));
      //Replace 8 with new highest case number
      while (menu == 8)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW) //Current hydrograph selected
        {
          setinput = true; //Toggle hydrograph bool
          //Change flashy to new hydrograph identifier
          indexmax = sizeof(flashy); //Fetch the size of the current hydrograph
          index = 0; //Initialize index
          for (int i = 0; i < indexmax; i++) //Copy each value from hydrograph to current hydro
          {
            //Change flashy to new hydrograph name
            currenthydro[i] = flashy[i];
          }
          delay(clicker);
          menu = 6; //Send us to menu 6, used to run all hydrographs
        }
      }
      break;

    //Classic High Hydrograph
    case 9:
      clearLCD();
      cursorHome();
      Serial.print(F("< Classic High >"));
      cursorLine2();
      Serial.print(F("< 17.5 Minutes >"));
      while (menu == 9)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setinput = true;
          indexmax = sizeof(classichigh);
          index = 0;
          for (int i = 0; i < indexmax; i++)
          {
            currenthydro[i] = classichigh[i];
          }
          delay(clicker);
          menu = 6;
        }
      }
      break;

    //Megaflood Hydrograph
    case 10:
      clearLCD();
      cursorHome();
      Serial.print(F("<  Megaflood   >"));
      cursorLine2();
      Serial.print(F("< 16.5 Minutes >"));
      while (menu == 10)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setinput = true;
          indexmax = sizeof(megaflood);
          index = 0;
          for (int i = 0; i < indexmax; i++)
          {
            currenthydro[i] = megaflood[i];
          }
          delay(clicker);
          menu = 6;
        }
      }
      break;

    //Classic Hydrograph
    case 11:
      clearLCD();
      cursorHome();
      Serial.print(F("<   Classic    >"));
      cursorLine2();
      Serial.print(F("< 14.5 Minutes >"));
      while (menu == 11)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setinput = true;
          indexmax = sizeof(classic);
          index = 0;
          for (int i = 0; i < indexmax; i++)
          {
            currenthydro[i] = classic[i];
          }
          delay(clicker);
          menu = 6;
        }
      }
      break;

    //High Flash Flood Hydrograph
    case 12:
      clearLCD();
      cursorHome();
      Serial.print(F("< Flash  Flood >"));
      cursorLine2();
      Serial.print(F("< 6.5  Minutes >"));
      while (menu == 12)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setinput = true;
          indexmax = sizeof(flashyhigh);
          index = 0;
          for (int i = 0; i < indexmax; i++)
          {
            currenthydro[i] = flashyhigh[i];
          }
          delay(clicker);
          menu = 6;
        }
      }
      break;


    //Regulated Hydrograph
    case 13:
      clearLCD();
      cursorHome();
      Serial.print(F("<  Regulated   >"));
      cursorLine2();
      Serial.print(F("< 19.5 Minutes >"));
      while (menu == 13)
      {
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          setinput = true;
          indexmax = sizeof(regulated);
          index = 0;
          for (int i = 0; i < indexmax; i++)
          {
            currenthydro[i] = regulated[i];
          }
          delay(clicker);
          menu = 6;
        }
      }
      break;

    //Catch all case
    default:
      menu = 0;
      break;
  }
}


//////////////////////////////////////////////////////////////////////////////////
// Setup   ///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void setup()
{
  //DAC Initilization
  Wire.begin();           //Initialize wire library for DAC
  setpoint = 0;           //Keep the water from surging after DAC initialization
  output = 0;
  writePWM2();

  //LCD Initialization
  Serial.begin(9600);     //Begin serial for data out to LCD Display
  displayOn();            //Initialize the LCD Display (Without this it displays gibberish upon data recieve.)
  setContrast(40);
  backlightBrightness(6);
  clearLCD();             //Clear the display

  //Pinmode configuration
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(encoderswitch, INPUT_PULLUP);
  pinMode(channelA, INPUT_PULLUP);
  pinMode(channelB, INPUT_PULLUP);
  pinMode(paddlewheel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(paddlewheel), timing, RISING); //Sense paddlewheel pin using interrupts: attachInterrupt(PIN, ISR, MODE)

  //KP math
  kpA = (KPatMAGhigh - KPatMAGlow) / (errorMAGhigh - errorMAGlow); //Slope of line through two points, linear regression for KP scheduling
  kpB = KPatMAGhigh - (kpA * errorMAGhigh);     //Y intercept of line using point slope formula, linear regression for KP scheduling

  //Splash Screen
  cursorHome();
  Serial.print(F("K500 Closed Loop"));
  cursorLine2();
  Serial.print(F("Flow Controller"));
  delay(2500);
  clearLCD();
  cursorHome();
  Serial.print(F("1/2 Paddlewheel"));
  cursorLine2();
  Serial.print(F("Software v1.12.0"));
  delay(2500);
  clearLCD();
}

//////////////////////////////////////////////////////////////////////////////////
// Loop  /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void loop()
{
  menuselect();
}
