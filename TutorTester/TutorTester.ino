/***************************************************

   Author:  Bruce Hall, W8BH
     Date:  17 Jun 2020
  Purpose:  Test Basic PocketTutor Inputs/Outputs
            
    Notes:  At most basic level, this is a blink
            sketch which double-blinks the onboard LED
            every 3 seconds.

            Audio Test: 700Hz beep-beep in sync with LED

            Encoder Test: rotating the knob changes pitch 
            up (CW) or down (CCW) at 4 Hz per detent. 
            A button press resets pitch to 700Hz; 
            Holding down button = triple-beep.

            Screen Test:  colored background & border make
            it easy to check alignment with panel. 
            The audio, encoder, and paddles can all be
            tested without a working screen.

            Paddle Test: hold down one or both paddles.
            Paddle A = single short beep; 
            Paddle B = two long beeps.  
            Both A & B = single, long beep

            Backlight Test: Backlight brightness ramps up/
            down with pitch/encoder rotation: 0% at 500Hz-,
            50% at 700Hz, 100% at 900Hz+
            
 ****************************************************/
 
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_DC             PA0                    // pin attached to display pin "DC"
#define TFT_CS             PA1                    // pin attached to display pin "CS"
#define SPEAKER            PA2                    // pin attached to speaker
#define ENCODER_A          PA9                    // Rotary Encoder output A
#define ENCODER_B          PA8                    // Rotary Encoder output B
#define ENCODER_BUTTON     PB15                   // Rotary Encoder switch
#define PADDLE_A           PB8                    // Morse Paddle "dit"
#define PADDLE_B           PB7                    // Morse Paddle "dah"
#define BATTERY            PB0                    // Battery voltage monitor

#define LED                PC13                   // pin for onboard LED

#define DURATION           200                    // tone on/off duration, in mS
#define PERIOD             3000                   // time between beeps, in mS 
#define PITCH              700                    // starting pitch, in Hz
#define bgColor            ILI9341_BLUE
#define fgColor            ILI9341_YELLOW
#define BACKLIGHT          PB1
#define PNP_DRIVER         true                   // true = using PNP to drive backlight


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
HardwareTimer timer(3);

int level;

//===================================  Rotary Encoder Code  =============================
volatile int      rotaryCounter    = PITCH;       // "position" of rotary encoder (increments CW) 
volatile boolean  button_pressed   = false;       // true if the button has been pushed
volatile boolean  button_released  = false;       // true if the button has been released (sets button_downtime)
volatile long     button_downtime  = 0L;          // ms the button was pushed before released

void buttonISR()
{  
  static boolean button_state = false;
  static unsigned long start, end;  
  boolean pinState = digitalRead(ENCODER_BUTTON);
  
  if ((pinState==LOW) && (button_state==false))                     
  {                                               // Button was up, but is now down
    start = millis();                             // mark time of button down
    if (start > (end + 10))                       // was button up for 10mS?
    {
      button_state = true;                        // yes, so change state
      button_pressed = true;
    }
  }
  else if ((pinState==HIGH) && (button_state==true))                       
  {                                               // Button was down, but now up
    end = millis();                               // mark time of release
    if (end > (start + 10))                       // was button down for 10mS?
    {
      button_state = false;                       // yes, so change state
      button_released = true;
      button_downtime = end - start;              // and record how long button was down
    }
  }
}

void rotaryISR()
{
  const int states[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  static byte transition = 0;                     // holds current and previous encoder states   
  transition <<= 2;                               // shift previous state up 2 bits
  transition |= (digitalRead(ENCODER_A));         // put encoder_A on bit 0
  transition |= (digitalRead(ENCODER_B) << 1);    // put encoder_B on bit 1
  transition &= 0x0F;                             // zero upper 4 bits
  rotaryCounter += states[transition];            // update counter +/- 1 based on rotation
}

boolean buttonDown()                              // check CURRENT state of button
{
  return (digitalRead(ENCODER_BUTTON)==LOW);
}

bool ditPressed()
{
  return (digitalRead(PADDLE_A)==0);              // pin is active low
}

bool dahPressed()
{
  return (digitalRead(PADDLE_B)==0);              // pin is active low
}
//===================================  Rotary Encoder ENDS  =============================


void showLED(boolean ON)
{
  const int XPOS=180, YPOS=120, SIZE=15;
  int color = (ON)? ILI9341_GREEN :               // on color = green
    bgColor;                                      // off color = background
  tft.fillRect(XPOS,YPOS,SIZE,SIZE,color);        // display LED as a filled square
}

void showBatteryVoltage()
{   
  float reading = analogRead(BATTERY);            // get raw data from voltage divider
  reading *= 2;                                   // we divided by 2, so multiply back
  reading *= 3.3;                                 // Multiply by 3.3V, our reference voltage
  reading /= 4096;                                // convert to voltage
  tft.print(reading); tft.print(" V");            // display result
}

void setBrightness(int level)                     // level 0 (off) to 100 (full on)       
{
   if (PNP_DRIVER) level = 100-level;             // invert levels if PNP driver present
   pwmWrite(BACKLIGHT, level*72);                 // conver 0-100 to 0-7200
   delay(5);  
}

void backlightInit(int brightness)
{
  level = brightness;                             // update global variable
  pinMode(BACKLIGHT, PWM);                        // set up backlight as PWM  
  timer.setPrescaleFactor(1);                     // do not scale 72MHz sys clock
  timer.setOverflow(7200);                        // 72MHz clock/7200 = 10KHz PWM
  setBrightness(level);                           // set initial brightness (0-100)
}

void beep()
{
  digitalWrite(LED,LOW);                          // LED indicates tone is on.
  showLED(1);                                     // Screen display of LED
  tone(SPEAKER,rotaryCounter);                    // Start the tone
  if (dahPressed()) delay(DURATION*2);            // LONG tone if PADDLE_B pressed
  else delay(DURATION/2);                         // keep it on for a while... 
  
  noTone(SPEAKER);                                // then turn off speaker
  digitalWrite(LED,HIGH);                         // and LED, too.   
  showLED(0);                                     // and screen indicator.
  delay(DURATION/2);                              // Keep them off for a while
}

void setup() {
  pinMode(LED,OUTPUT);                            // set up onboard LED
  pinMode(PADDLE_A, INPUT_PULLUP);                // two paddle inputs, both active low
  pinMode(PADDLE_B, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);               // set up encoder inputs
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(          // set up encoder interrupts
    ENCODER_BUTTON),buttonISR,CHANGE);   
  attachInterrupt(digitalPinToInterrupt(
    ENCODER_A),rotaryISR,CHANGE); 
  attachInterrupt(digitalPinToInterrupt(
    ENCODER_B),rotaryISR,CHANGE);
  backlightInit(100);                             // start backlight pwm @ 100% 
  tft.begin();                                    // initialize screen
  tft.setRotation(3);
  tft.fillScreen(bgColor);                        // paint screen w/background
  tft.drawRoundRect(0,0,320,240,20,    
    ILI9341_YELLOW);
  tft.setTextColor(ILI9341_YELLOW);               // set up text color/size
  tft.setTextSize(2);
  tft.setCursor(50,40);
  tft.print("W8BH TUTOR TESTER");
  tft.setCursor(50,80);
  tft.print("Backlight:");
  tft.setCursor(50,100);
  tft.print("Pitch:");
  tft.setCursor(50,120);
  tft.print("Audio/LED:");
  tft.setCursor(50,140);
  tft.print("Paddles:");
  tft.setCursor(50,160);
  tft.print("Battery:");
}

void loop() { 
  rotaryCounter = min(rotaryCounter,2000);        // set pitch max
  rotaryCounter = max(rotaryCounter,300);         // set pitch min
  if (button_pressed) rotaryCounter = PITCH;      // reset pitch
  button_pressed = false;  
  
  level = 50+(rotaryCounter-700)/4;               // 100 range over 500-900
  level = constrain(level,0,100);                 // keep level in 0-100 range
  setBrightness(level);                           // set display brightness
  tft.fillRect(180,80,80,16,bgColor);             // erase previous level
  tft.setCursor(180,80); 
  tft.print(level);                               // show display brightness

  tft.fillRect(180,100,80,16,bgColor);            // erase previous pitch
  tft.setCursor(180,100); 
  tft.print(rotaryCounter);                       // display current pitch
  
  tft.fillRect(180,140,40,16,bgColor);            // erase previous paddles
  tft.setCursor(180,140);                         // show which paddles are pressed
  if (ditPressed()) tft.print("A");               // Paddle A (dit)
  if (dahPressed()) tft.print("B");               // Paddle B (dah)

  tft.fillRect(180,160,120,16,bgColor);           // erase previous voltage
  tft.setCursor(180,160);                         
  showBatteryVoltage();                           // show current voltage

  beep();                                         // 1 beep if PaddleA pressed
  if (!ditPressed()) beep();                      // 2 beeps normally
  if (buttonDown()) beep();                       // 3 beeps if encoder button pressed
  delay(PERIOD-DURATION*2);                       // wait between beeps 
}
