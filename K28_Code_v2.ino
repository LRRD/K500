//K28 Code written from scratch for SMT version
//2 Available interrupts, chA, chB, and button would be attached, be we are limited, instead just button is interrupt
//Tested working with analogWrite for led and pump voltage outputs - each step is ~20mV
//Migrating from analog write to on-board DAC


////////////////////////////////////////////////////////////////////////////
// Magic Numbers & Global Variables ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
const uint8_t button = 2;         //Encoder button
const uint8_t channelA = 5;       //Encoder channel A
const uint8_t channelB = 6;       //Encoder channel B
const uint8_t pump = 9;           //Pump mosfet signal pin
const uint8_t led = 10;           //LED pump speed pin
int encoder_count = 0;            //Current flow setting
int lowFlow = 1500;               //~Encoder count below lowest flow setting


////////////////////////////////////////////////////////////////////////////
// DAC Controls ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void writePWM() //Function to control analog voltage out from DAC
{
  int flowvalue = map(encoder_count, 0, 4095, 0, 255);
  analogWrite(pump, flowvalue);
  analogWrite(led, flowvalue);
}

////////////////////////////////////////////////////////////////////////////
// Encoder handling  ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// The sequence of gray codes in the increasing and decreasing directions.
static uint8_t cw_gray_codes[4] = { 2, 0, 3, 1 };     
static uint8_t ccw_gray_codes[4] = { 1, 3, 0, 2 };
static uint8_t previous_gray_code = 0;

// Look for encoder rotation, updating encoder_count as necessary.
void check_encoder()
{
  // Get the Gray-code state of the encoder.
  // A line is "low" if <= 512; "high" > 512 or above.
  int gray_code = ((digitalRead(channelA) == HIGH) << 1) | (digitalRead(channelB) == HIGH);

  // If the gray code has changed, adjust the intermediate delta.
  if (gray_code != previous_gray_code)
  {
    if (encoder_count < lowFlow)  //Coarse adjustment
    {
      //Knob twist CW
      if (gray_code == cw_gray_codes[previous_gray_code])
      {
        encoder_count += 50;
      }
  
      //Knob twist CCW
      else if (gray_code == ccw_gray_codes[previous_gray_code])
      {
        encoder_count -= 50;
      }
    }
    
    else     //Fine adjustment
    {
      //Knob twist CW
      if (gray_code == cw_gray_codes[previous_gray_code])
      {
        encoder_count += 16;     //Allows for 256 values (4096/16 = 256) good enough for fine controls, but not so much that you need to turn the knob a lot
      }
  
      //Knob twist CCW
      else if (gray_code == ccw_gray_codes[previous_gray_code])
      {
        encoder_count -= 16;
      }
    }
    previous_gray_code = gray_code; //Stores current gray code for future comparison
    encoder_count = constrain(encoder_count, 0, 4095);
  }
}

////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine (ISR)  ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void timing() //Runs when ISR is called from interrupt pin input
{
  encoder_count = 0;
}

//////////////////////////////////////////////////////////////////////////////////
// Setup   ///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  pinMode(button, INPUT_PULLUP);      //Set the encoder button to input pullup. Most buttons work best as input pullup.
  pinMode(channelA, INPUT_PULLUP);    //For grayhill 62AG11-L5-060C reverse compatible with older grayhill op encoder as well
  pinMode(channelB, INPUT_PULLUP);    //For grayhill 62AG11-L5-060C reverse compatible with older gryahill op encoder as well
  encoder_count = 0;                  //Initialize encoder count
  attachInterrupt(digitalPinToInterrupt(button), timing, FALLING); //Sense button pin using interrupts: attachInterrupt(PIN, ISR, MODE)
}

//////////////////////////////////////////////////////////////////////////////////
// Loop  /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void loop()
{
  check_encoder();                    //Check for rotation
  writePWM();                         //Update output to pump and LED
}
