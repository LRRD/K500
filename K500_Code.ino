////////////////////////////////////////////////////////////////////////////
// Updates and Changes  ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//Mason Parrone 4/2018 Code for digital flow controller redesign for use with new Brushless submersible pump (0-5v speed control signal)
//Stripped down encoder code to bare bones and moved all inputs to digital pins that aren't PWM. This previously used the analog pins that we may need for the DAC in the future.
//Added in LCD code from RS-232 display code
//Added in DAC code for MCP 4725 Adafruit version
//Volumetric test input into excel: A,B,C Flow values found and input
//Implemented ml/s calculation and readout
//Copied total L and elapsed time from Alix controller code
//Added paddlewheel code as interrupt on pin 2
//Added PID loop and controls for paddlewheel flow measurement and close loop
//Moved PID loop calculation into refresh LCD. This gives some time to between last output adjustment and next calculation
//Kp values scheduled for high and low flows
//Added timer back onto display
//Added switch case for menu control, broke functions into multiple smaller functions for better menu control and display
//Added hydrograph menu and other hydrograph functions
//Optimized hydrograph code, added instructions to add new hydrographs, and imported hydrographs from old Alix controller
//Added paddlewheel indicator in top right corner, displayed when PID loop is active
//Bug fix, paddlewheel indicator only appears on menu 1 and 6 now
//Major PID loop changes
//Removed I and D in PID loop
//Changed P to P scheduling where P values are higher at high flow and lower at low flow
//Recalculated volumetric curve for encoder count vs ml/s and duration vs ml/s
//Duration vs ml/s is only calculated using values from ~55ml/s - ~185ml/s
//Encoder count vs ml/s is only calculated using values from 0 ml/s - ~185ml/s
//Added time since last duration reading, this is used to switch back and forth between encoder count regression and P control loop
//All flows below 55 ml/s are described as less than 55ml/s rather than explicitly stated, this prevents user confusion. They aren't necessarily accurate in that range
//0 ml/s is explicitly stated
//Walking paddle character fixed
//P control loop only runs when setpoint >= 55ml/s and time since last paddlewheel reading is < 400 ms.
//Exponential smoothing was used to reduce raw duration reading volatility


////////////////////////////////////////////////////////////////////////////
// Libraries and Definitions ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#include <Wire.h>//Include the Wire library to talk I2C
#define MCP4725_ADDR 0x62 //Unique address for MCP4725 Adafruit version 


////////////////////////////////////////////////////////////////////////////
// Magic Numbers  & Global Variables ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
static uint8_t cw_gray_codes[4] = { 2, 0, 3, 1 }; 	//The sequence of gray codes in the increasing and decreasing directions.
static uint8_t ccw_gray_codes[4] = { 1, 3, 0, 2 };  //Gray code sequence
static uint8_t previous_gray_code = 0;				//The gray code we last read from the encoder.
uint32_t last_display_update_ms = 0;				//Used to refresh the lcd Screen
uint32_t last_display_update_ms2 = 0;				//Used to refresh the hydrograph time
const uint8_t encoderswitch = 7;					//Encoder poin reference
const uint8_t channelB = 6;							//Encoder pin reference
const uint8_t channelA = 4;							//Encoder pin reference
int encoder_count = 0;								//Used to adjust setpoint from encoder
int released;										//Used for encoder button presses
const uint8_t led = 3;								//LED pin reference
const uint8_t led2 = 5;								//LED pin reference
uint32_t lastClick = 0;								//Millisec since last encoder click used for encoder
uint32_t clickTimer = 0;							//Time since last click
uint32_t cumulativeVolume = 0;						//Total amount of ml/s of water pumped since last restart.
uint16_t lastLiter = 0;								//Last total liter pumped written to display
const uint16_t startFlow = 2000;					//Encoder count slightly before flow starts
const uint8_t paddlewheel = 2;						//Pin assignment method that uses the least space
float duraVal = 8303.8;							//Power function regression (x: duration between pulses, y: ml/s) ax^b
float durbVal = -.971;							//Power function regression (x: duration between pulses, y: ml/s) ax^b
float encaVal = .1396;								//Linear regression (x: encoder count, y: ml/s)
float encbVal = -291.5;							//Linear regression (x: encoder count, y: ml/s)
float kpAval = .00833;              //Linear regression (x: |error|, y: desired kp value)
float kpBval = .16667;              //Linear regression (x: |error|, y: desired kp value)
uint32_t duration = 0;								//Raw data in from paddlewheel sensor
int setpoint = 0;								//Setpoint in desired ml/s
uint8_t input = 0;									//Input value from paddlewheel sensor
float error = 0;									//Error value between measured ml/s and setpoint in ml/s
int output = 0;								//Output of PID Loop to DAC
float kp = 0.0;									//K factor for proportional
uint16_t lastflow = 0;								//Used to refresh display when setpoint changes
uint8_t menu = 0;									//Switch case variable for menu controls
uint16_t clicker = 500;								//Ms delay for all button presses and menu navigation rotations
uint16_t flow = 0;									//Used to display flow in ml/s
uint16_t totalLiter = 0;							//Used to display total L of water pumped
bool setinput = false;								//Used to run hydrographs
uint16_t index = 0;									//Used to step through hydrograph values
uint16_t indexmax = 0;								//Used to define length of each hydrograph
const uint16_t maxhydro = 120;						//Max number of 30s intervals in a hydrograph (Increasing uses lots of memory)
bool indicator = false;								//Used to switch between + and * for hydrograph indicator
uint32_t hydroStartTime = 0;						//Hydrograph timer shows how uint32_t the hydrograph has been running
uint32_t totalSec = 0;								//Total seconds the Arduino has been running
const uint8_t minFlow = 35;							//Minimum flow for paddlewheel to spin
volatile uint32_t nowTime = 0;						//Time marked during the interrupt routine
volatile bool inputread = false;					//Global flag to indicate paddlewheel reading
uint32_t smoothed = 0;								//Duration value following exponentional smoothing process
uint32_t lastsmoothed = 0;							//Last " "
float alpha = 0.075;								//Exponential smoothing constant, smaller makes it more smooth but increases settling time after change in setpoint
uint16_t slowTurn = 400;							//400ms for the slowest paddlewheel turning
uint32_t timesince = 0;           //ms since last paddlewheel pulse
bool paddleactive = false;      //Global flag, paddlewheel is spinning
bool setpointinrange = false;     //Global flag, setpoint is in paddlewheel operation range


//////////////////////////////////////////////////////////////////////////////////
// Hydrographs ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//Each array is a new hydrograph, values are flow setpoints at 30s spacings.
//Max hydrograph length is 1 hr.
//Max flow value is 187.
//To make a new hydrograph define it with a unique identity as a const uint8_t as shown below
//Ctrl + F (Find) " Case 8 : " or travel to code line ~877
//Copy Case 8 and paste it below the highest number case
//Replace all case # references with the new current highest case number
//Replace text with hydrograph name and unique hydrograph identity
//Detailed instructions are in the case creation
//Change menuhigh to the new highest menu # after adding hydrographs (Increase by 1 for each added hydrograph)
const uint8_t menuhigh = 13;
const uint8_t flashy[] = { 40, 66, 105, 100, 90, 85, 82, 76, 66, 60, 58, 50, 45, 40, 25, 20 }; //8 Minutes
const uint8_t classichigh[] = { 29, 38, 44, 54, 58, 67, 74, 80, 91, 102, 110, 117, 115, 113, 102, 99, 93, 84, 82, 80, 78, 73, 69, 66, 62, 58, 55, 51, 47, 44, 40, 37, 33, 15, 15 }; //17.5 Minutes
const uint8_t megaflood[] = { 40, 68, 90, 110, 130, 150, 160, 170, 170, 165, 160, 155, 150, 146, 139, 132, 125, 121, 113, 107, 96, 89, 82, 75, 68, 60, 53, 51, 51, 45 }; //16.5 Minutes
const uint8_t classic[] = { 25, 30, 48, 56, 66, 82, 95, 106, 92, 80, 75, 72, 69, 65, 62, 60, 57, 55, 52, 50, 47, 45, 41, 38, 35, 31, 25, 22, 20 }; //14.5 Minutes
const uint8_t flashyhigh[] = { 20, 126, 160, 155, 120, 102, 80, 60, 50, 40, 30, 25, 20 }; //6.5 Minutes
const uint8_t regulated[] = { 40, 50, 50, 70, 70, 70, 70, 70, 70, 50, 50, 70, 70, 70, 70, 70, 50, 50, 50, 50, 70, 70, 70, 70, 50, 50, 50, 50, 50, 70, 70, 70, 70, 50, 50, 50, 70, 70, 70 }; //19.5 Minutes
uint16_t currenthydro[maxhydro] = {}; //The longest hydrograph can be .5 hour (60, 30-Second intervals) I needed a number for this, Increasing maxhydro uses up loads of memory


////////////////////////////////////////////////////////////////////////////
// DAC Controls ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void writePWM2() //Function to control analog voltage out from DAC
{
  output = encoder_count;
  output = constrain(output, 0, 4095);
  int pwm = output; //Relate current output to value being sent to DAC.
  uint16_t pwm2 = (encoder_count * 255 / 4095); //Relate encoder count to value being sent to LED
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write(pwm >> 4);        //Write PWM Value with the 8 most significant bits...
  Wire.write((pwm & 15) << 4); //Add the 4 least significant bits...
  Wire.endTransmission();
  analogWrite(led, pwm2); //Write to LED with encoder count value
  analogWrite(led2, pwm2);  //Write to LED with encoder count value
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
void timing()
{
  duration = millis() - nowTime; //Use time to figure out duration in milli since last pulse
  nowTime = millis(); //Assign time
  inputread = true;
}


////////////////////////////////////////////////////////////////////////////
// Encoder handling  ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
static void reset_encoder()
{
  encoder_count = 0;
  output = 0;
  clearLCD();
  cursorHome();
  Serial.print(F("System Reset"));
  delay(750);
  clearLCD();
}


static void check_encoder() // Look for encoder rotation, updating encoder_count as necessary.
{
  // Get the Gray-code state of the encoder.
  int gray_code = ((digitalRead(channelA) == HIGH) << 1) | (digitalRead(channelB) == HIGH);
  if (gray_code != previous_gray_code)   //Assign current gray code to last gray code
  {
    //Encoder clicked in a direction
    clickTimer = (millis() - lastClick); //Time between last click and current click
    lastClick = millis(); //Last click time is right now

    //Knob twist CW
    if (gray_code == cw_gray_codes[previous_gray_code])
    {
      //Bigger increment, less than flow start value
      if ((clickTimer < 100) && (encoder_count < startFlow))       //LESS THAN 100 MS between clicks
      {
        encoder_count = encoder_count + 150;
      }
      else if ((clickTimer >= 100) && (clickTimer < 350) && (encoder_count < startFlow))   //BETWEEN 100 and 300 MS between clicks
      {
        encoder_count = encoder_count + 15;
      }
      else if ((clickTimer >= 350) && (encoder_count < startFlow))     //GREATER THAN 300 MS between clicks
      {
        encoder_count = encoder_count + 3;
      }
      //Regular increment, within flow range
      else if ((clickTimer < 100) && (encoder_count >= startFlow))       //LESS THAN 100 MS between clicks
      {
        encoder_count = encoder_count + 50;
      }
      else if ((clickTimer >= 100) && (clickTimer < 350) && (encoder_count >= startFlow))   //BETWEEN 100 and 300 MS between clicks
      {
        encoder_count = encoder_count + 5;
      }
      else if ((clickTimer >= 350) && (encoder_count >= startFlow))     //GREATER THAN 300 MS between clicks
      {
        encoder_count = encoder_count + 1;
      }
    }

    //Knob twist CCW
    else if (gray_code == ccw_gray_codes[previous_gray_code])
    {
      //Bigger increment, less than flow start value
      if ((clickTimer < 100) && (encoder_count < startFlow))       //LESS THAN 100 MS between clicks
      {
        encoder_count = encoder_count - 150;
      }
      else if ((clickTimer >= 100) && (clickTimer < 350) && (encoder_count < startFlow))   //BETWEEN 100 and 300 MS between clicks
      {
        encoder_count = encoder_count - 15;
      }
      else if ((clickTimer >= 350) && (encoder_count < startFlow))     //GREATER THAN 300 MS between clicks
      {
        encoder_count = encoder_count - 3;
      }
      //Regular increment, within flow range
      else if ((clickTimer < 100) && (encoder_count >= startFlow))       //LESS THAN 100 MS between clicks
      {
        encoder_count = encoder_count - 50;
      }
      else if ((clickTimer >= 100) && (clickTimer < 350) && (encoder_count >= startFlow))   //BETWEEN 100 and 300 MS between clicks
      {
        encoder_count = encoder_count - 5;
      }
      else if ((clickTimer >= 350) && (encoder_count >= startFlow))     //GREATER THAN 300 MS between clicks
      {
        encoder_count = encoder_count - 1;
      }
    }
    previous_gray_code = gray_code; //Stores current gray code for future comparison
    encoder_count = constrain(encoder_count, 0, 4095); //Flow maxes at ~185 ml/s at enc 3695
  }
}


////////////////////////////////////////////////////////////////////////////
// Switch handling  ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

static unsigned int switch_was_down = 0;
static unsigned int switch_released()
{
  // Becuase it's an input pullup switch reads high when pressed.
  int switch_is_down = (digitalRead(encoderswitch) == HIGH);

  // The action takes place when the switch is released.
  released = (switch_was_down && !switch_is_down);

  // Remember the state of the switch.
  switch_was_down = switch_is_down;

  // Was the switch just released?
  return released;
}


//////////////////////////////////////////////////////////////////////////////
/// Screen Refresh   /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void refresh_lcd()
{
  uint32_t elapsed_ms = millis() - last_display_update_ms;
  if ((last_display_update_ms == 0) || (elapsed_ms >= 500)) //Periodically refresh display
  {
    compute(); //Compute adjustment
    writePWM2(); //Adjust flow
    last_display_update_ms = millis();
    timer();  //Display timer
    uint16_t currentMls = ((setpoint) / 2); //Ml/s / 2 because this runs every half second
    cumulativeVolume = (cumulativeVolume + currentMls); //Add current mls to total mls
    literCalc(); //Calculate total liter
    literdisplay(); //Display total liter
    walkingpaddle(); //Display walking paddle
  }
}

//Refresh without display
void refresh_nolcd()
{
  uint32_t elapsed_ms = millis() - last_display_update_ms;
  if ((last_display_update_ms == 0) || (elapsed_ms >= 500)) //Periodically refresh display
  {
    compute();
    writePWM2();
    last_display_update_ms = millis();
    uint16_t currentMls = ((setpoint) / 2); //Ml/s / 2 because this runs every half second
    cumulativeVolume = (cumulativeVolume + currentMls); //Add current mls to total mls
    literCalc(); //Calculate total liter
  }
}


//////////////////////////////////////////////////////////////////////////////
/// Hydrograph Refresh ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void hydrorefresh()
{
  uint32_t elapsed_ms2 = millis() - last_display_update_ms2;
  if (elapsed_ms2 >= 30000) //Periodically refresh display
  {
    last_display_update_ms2 = millis();
    if (index < (indexmax - 1)) //Still stepping our way through the hydrograph every 30s
    {
      index++;
    }
    else if (index >= (indexmax - 1)) //The last flow setpoint is complete for the hydrograph
    {
      setinput = false;
      cursorHome();
      Serial.print(F("<  Hydrograph  >"));
      cursorLine2();
      Serial.print(F("<   Complete   >"));
      delay(2000);
      menu = 0; //Return to main display
      //Clear currenthydro by filling it with 0's
      for (uint16_t i = 0; i < maxhydro; i++)
      {
        currenthydro[i] = 0;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Liter Calc & Display  ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void literCalc()
{
  totalLiter = ((cumulativeVolume) / 1000); //Convert milliliter to liter
  if (totalLiter < 0) //Total liters pumped cannot be less than 0
  {
    totalLiter = 0;
  }
  if (totalLiter > 9999) //Total liters, can't write more than 3 digits
  {
    totalLiter = 9999;
  }
}

void literdisplay()
{
  totalLiter = ((cumulativeVolume) / 1000); //Convert milliliter to liter
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
  if (setinput == false)
  {
    totalSec = (millis() / 1000); //Total seconds arduino has been powered on for
  }
  else if (setinput == true)
  {
    totalSec = (millis() - hydroStartTime) / 1000; //Total time - time at start of hydro (seconds)
  }
  uint16_t hour = (totalSec / 3600);         //Number of hours elapsed
  uint16_t remainder = (totalSec % 3600);    //Remainder of Sec that don't total an hour
  uint16_t minute = (remainder / 60);        //Number of minutes elapsed
  remainder = (remainder % 60);         //Remainder of Sec that don't toal a minute
  uint16_t second = (remainder);             //Number of seconds elapsed
  cursorLine2();                        //Set cursor to line 2
  //Display elapsed time since last restart
  if (hour > 99)
  {
    hour = 99;
  }
  if (minute > 59)
  {
    minute = 59;
  }
  if (second > 59)
  {
    second = 59;
  }
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


//Paddlewheel indicator
void walkingpaddle()
{
  cursorTopRight();
  if ((setpointinrange) && (paddleactive)) //Conditions met for P control loop for flow
  {
    if(indicator)
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
  if (!setinput) //User chooses setpoint
  {
    setpoint = (encoder_count * encaVal) + encbVal; //Linear regression
  }
  else //Hydrograph chooses setpoint
  {
    setpoint = currenthydro[index]; //Read current hydro index value
  }
  setpoint = constrain(setpoint, 0, 180); //Setpoint can only be between 0 and 180 ml/s

  //Measured duration from paddlewheel
  if (inputread) //New duration from the paddlewheel received
  {
    smoothed = alpha * duration + (1 - alpha) * lastsmoothed; //Exponential smoothing
    lastsmoothed = smoothed; //Assign current to last for future calculation
    inputread = false; //Reset bool
  }
  input = duraVal * pow(smoothed, durbVal); //Power function regression

  //Check for time since last duration received to determine if paddlewheel is running/connected
  timesince = millis() - nowTime;

  if (timesince < slowTurn) //Paddle moving faster than slowest turn
  {
    paddleactive = true; //The paddlewheel is active
  }
  else
  {
    paddleactive = false; //The paddle wheel isn't spinning regularly, or is disconnected
  }

  if (setpoint >= 55)
  {
    setpointinrange = true; //The setpoint is within the working range of the paddlewheel
  }
  else
  {
    setpointinrange = false; //The setpoint is outside the working range of the paddlewheel
  }

  if ((setpointinrange) && (paddleactive)) //Conditions met for P control loop for flow
  {
    //Error
    error = (setpoint - input); //Not abs() sign matters
    kp = (abs(error)) * kpAval + kpBval; //Magnitude of the error is magnitudially related to the output adjustment
    output += (error * kp);
  }
  else //Run based on encoder count
  {
    output = ((setpoint - encbVal) / encaVal); //For user setpoint, this is essentially the same as encoder count, for hydrographs it is different
  }
}


void computedisplay()
{
  flow = setpoint;
  if (flow != lastflow) //If top line changes clear and update display
  {
    clearDataTopLine();
    cursorHome();
    if (setpoint >= 55)
    {
      Serial.print(setpoint);
      Serial.print(F(" Ml/s "));
    }
    else if ((setpoint < 55) && (setpoint >=1))
    {
      Serial.print(F("< 55 Ml/s "));
    }
    else
    {
      Serial.print(F("0 Ml/s "));
    }
    lastflow = flow;
  }
}


//////////////////////////////////////////////////////////////////////////////////
//Menu Navigator /////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void navigate()
{
  // Get the Gray-code state of the encoder.
  int gray_code = ((digitalRead(channelA) == HIGH) << 1) | (digitalRead(channelB) == HIGH);
  if (gray_code != previous_gray_code)   //Assign current gray code to last gray code
  {
    //Encoder clicked in a direction

    //Knob twist CW
    if (gray_code == cw_gray_codes[previous_gray_code])
    {
      delay(clicker);
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

    //Knob twist CCW
    else if (gray_code == ccw_gray_codes[previous_gray_code])
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
//Menu   /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void menuselect()
{
  switch (menu)
  {
    //Manual control of setpoint and timer display
    case 0:
      clearLCD();
      cursorHome();
      Serial.print(flow);
      Serial.print(F(" Ml/s"));
      while (menu == 0)
      {
        check_encoder();
        //compute();
        //writePWM2();
        refresh_lcd();
        computedisplay();
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          reset_encoder();
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
        compute();
        writePWM2();
        refresh_nolcd();
        navigate();
        if (digitalRead(encoderswitch) == LOW)
        {
          output = 4095;
          encoder_count = 4095;
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
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
          Serial.print(F("  v1.0 Written  "));
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

    //Hydrograph control of setpoint and timer display
    case 6:
      last_display_update_ms2 = millis();
      clearLCD();
      cursorHome();
      Serial.print(flow);
      Serial.print(F(" Ml/s"));
      hydroStartTime = millis(); //Assign current time as start time of hydrograph
      while (menu == 6)
      {
        hydrorefresh(); //Steps us through the hydrograph setpoints every 30s
        compute();
        writePWM2();
        refresh_lcd(); //Runs PID based on current setpoint
        computedisplay(); //Display
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
        compute();
        writePWM2();
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
    //Replace 8 with new highest case number
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
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
        compute();
        writePWM2();
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
  Wire.begin();           //Initialize wire library for DAC
  Serial.begin(9600);     //Begin serial for data out to LCD Display
  displayOn();            //Initialize the LCD Display (Without this it displays gibberish upon data recieve.)
  setContrast(40);
  backlightBrightness(6);
  clearLCD();             //Clear the display
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(encoderswitch, INPUT_PULLUP);
  pinMode(channelA, INPUT_PULLUP);
  pinMode(channelB, INPUT_PULLUP);

  //Splash Screen
  cursorHome();
  Serial.print(F("K500 Closed Loop"));
  cursorLine2();
  Serial.print(F("Flow Controller"));
  delay(2500);
  clearLCD();
  pinMode(paddlewheel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(paddlewheel), timing, RISING); //Sense paddlewheel pin using interrupts: attachInterrupt(PIN, ISR, MODE)
}

//////////////////////////////////////////////////////////////////////////////////
// Loop  /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void loop()
{
  int switch_event;
  switch_event = switch_released();  //Run routine that checks switch
  menuselect();
}
