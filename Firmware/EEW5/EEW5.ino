// eew5
// Maurice Ribble 
// March 16, 2017
// Open Source, licensed under Creative Commons Attribution 3.0 License (http://creativecommons.org/licenses/by-sa/3.0/)
// Compiled with Arduino Software 1.8.1 (http://arduino.cc) plus the attiny module
// This site explains how to setup tiny: http://highlowtech.org/?p=1695

// Here are the steps to loading the firmware:
//   Set Tools->Board to: ATtiny25/45/85
//   Set Tools->Processor to: ATtiny45
//   Set Tools->Clock to: Internal 16 MHz
//   Set Tools->Programmer to: USBtinyISP
//   Tools->Burn Bootloader - This sets some fuses so the chip runs at 16 Mhz.  If you forget to do this the chip will run
//     at the default of 1 Mhz.  A way to spot this happened is there is a green light on the EEW board that blinks once
//     per second.  If you foget this step it will blink much slower.
//   Upload the program

////////////////////////////////////////////////////////////////////////////////////////////////////
// REVISIONS:
// 1.00 April 7, 2015 - Maurice Ribble
//      Initial version
// 1.01 Sep 8, 2015 - Nedd
//      - Simplified some math lines to improve loop speed
//      - Increased system clock to 16mhz (was 1mhz). This improves the resolution of the micros() counter, reduces signal jitter, allows faster PWM frequencies.
//      - Added "TESTMODE" for repeated LV programming during testing. (In TESTMODE fuse RSTDISBL is unprogrammed, Reset (pin 1) is open, PB0 (pin 5) is rewired to M_EN, 
//        MOT_EN_PIN & PEDAL_EN_PIN definitions are swapped).
// 1.02 Oct 31, 2015 - Maurice Ribble
//      - Reduced math by pre-caching high and low speed
// 1.03 Nov 13, 2015 - Nedd
//      - Single adc conversions are now started manually (w/ADSC bit) and complete in the background. 
//      - Adc registers are setup manually. Conversion result is left adjusted (ADLAR bit set), only the upper 8-bit value (ADCH) is read when needed.
//      - Pulse width delays are now timed with the more accurate MicroSecond() function. Resolution is 1uS. 
//      - A pulse width array is used for all high side pulse widths.
//      - The array can optionally be filled with coefficients to create a speed curve. Calculations are done before the main code runs. 
//      - Motor off positions are defined by a zeros within the array.
//      - Total loop time has been greatly reduced. The small remaining loop time is compensated for by subtracting it from the high time.
//      - The motor control block is now written with no external calls, (except delayuS() ).
// 1.04 Dec 14, 2015 - Nedd
//      - I/O pins revised for new PCB design, (version 6). 
//      - Spare I/O pin is now used as a Status output, with an optional bi-color LED to indicate Stop (Red), Run Green), & Wait (Yellow)
//      - Added Fwd/Rev testing for new pcb MOSFET drive system.
//      - Testing of old PWM method using new prototype PCB design, (version 6).
// 1.05 Feb 02, 2016 - Nedd
//      - Added register setups to use direct PWM output on PB1, PWM comes from OC1A signal, PW is adjusted by setting OCR1A (compare A) of timer1.
//      - Added Dead Zone adjustment (expands range where motor is at 0 speed)
//      - Added Slow Zone adjustment (reduces range where motor does not spin)
//      - Added safe mode at first power on. Operator must adjust speed knob up then back to zero to enable motor. Status line toggles detection points
// 1.05a Mar 12, 2016 - Nedd
//      - Added double pedal press option to exit safe mode
// 1.06 Mar 26, 2016 - Maurice Ribble
//      - Updated control logic be easier to use
//      - Added green led
//      - Lots of code cleanup and simplification
// 1.07 Jan 19, 2017 - Maurice Ribble
//      - Updated upload procedure
// 1.08 March 16, 2017 - Maurice Ribble
//      - Fix bug with foot pedal, motor direction, and unplugging EEW5
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration flags
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Digital/analog pins
#define PIN_STATUS            0    // Green status LED output     -- LOW=On       HIGH=Off
#define PIN_MOT_EN            1    // Enable motor output         -- LOW=Stop     HIGH=Run
#define PIN_PEDAL             2    // Foot pedal switch input     -- LOW=Pressed  HIGH=Not Pressed
#define APIN_SPEED            3    // Speed control dial analog input (ADC3 = PB3)
#define PIN_MOT_DIR           4    // Direction motor output      -- LOW=CCW      HIGH=CW

// Green Status LED
#define LED_ON  LOW
#define LED_OFF HIGH

// Bi-color LED - (Note: Different Bi-color LED types may need an adjustment to the pwm # to acheive a good yellow)
// This LED is not installed by default
//#define ENABLE_BI_LED
#ifdef ENABLE_BI_LED
  #define BI_LED_RED    0
  #define BI_LED_YELLOW 155
  #define BI_LED_GREEN  255
#endif

// Speed dial related controls
#define DEAD_ZONE 15               // Range close to zero where motor will be forced off
#define SLOW_ZONE 90               // Shift motor's minimum speed faster to get motor spinning at lower speed dial settings
#define T1_MAX_TIME 168            // Used to help set pwm durations, see setPwmParams() for more details

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// digitalReadFast - Fast version of digitalRead()
//  input p - The pin you want to read
//  return  - LOW or HIGH for the requested pin
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline byte digitalReadFast(byte p)
{
  return bitRead(PINB, p);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// digitalWriteFastHigh - This format does not disable other peripherals that may be connected to the pin
//  input p - The pin you want to write
//  return  - void
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void digitalWriteFastHigh(byte p)
{
  bitSet(PORTB, p);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// digitalWriteFastLow - This format does not disable other peripherals that may be connected to the pin
//  input p - The pin you want to write
//  return  - void
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void digitalWriteFastLow(byte p)
{
  bitClear(PORTB, p);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// analogReadSpeedDial - Fast readAnalog for the pin connected to the speed dial
//  input   - void
//  return  - 8 bit analog value
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline byte analogReadSpeedDial()
{
  bitSet(ADCSRA, ADSC);
  while( bitRead(ADCSRA, ADSC) == 1) {};  // ADSC bit goes to zero when complete so this waits for complete
  return ADCH;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motorPwmOff - Turns off the pulse width modulation signal being sent to the motor
//  input       - void
//  return      - void
//  Side effect - COM1A1 and COM1A0 (of TCCR1)
//  Note        - Register name TCCR1 seems to be equivalent to TCCR1A
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void motorPwmOff()
{
  TCCR1 &= ~(1<<COM1A1);  TCCR1 &= ~(1<<COM1A0);  //disconnect pwm from PB1
}  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motorPwmOn - Turns on the pulse width modulation signal being sent to the motor
//  input       - void
//  return      - void
//  Side effect - COM1A1 and COM1A0 (of TCCR1)
//  Note        - Register name TCCR1 seems to be equivalent to TCCR1A
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void motorPwmOn()
{ 
  TCCR1 |= (1<<COM1A1);  
  TCCR1 |= (1<<COM1A0);  //connect pwm to PB1
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motorRun - Turns motor on
//  input motorSpeed - Speed of motor in range of 0 to 127 with 0 being full off and 127 being full on
//  return           - void
//  Side effect      - Changes OCR1A, motor enable pin, and others in motorPwmOn
//  Note             - It is possible to have motor on and not spinning if motorSpeed is 0
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void motorRun(byte motorSpeed)
{
  //calculate pwm value to use for compare A (OCR1A). Maps speedIndex to resolution of counter timer1, (Note: T1_MAX_TIME is full off, 0 is full on)
  byte motorPwmVal = (motorSpeed == 0) ? T1_MAX_TIME : map(motorSpeed,DEAD_ZONE,127,T1_MAX_TIME-SLOW_ZONE,0);

  OCR1A = motorPwmVal;  // Adjusts pwm speed

  motorPwmOn();
  digitalWrite(PIN_MOT_EN, HIGH);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motorRun - Turns motor off
//  input       - void
//  return      - void
//  Side effect - Changes motor enable pin, and others in motorPwmOn
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void motorStop()
{
  motorPwmOff();
  digitalWrite(PIN_MOT_EN, LOW);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// blinkStatusLed - Blinks the green status LED for 1/10 of a second every 1 second
//  input       - void
//  return      - void
//  Side effect - Changes status port pin
//  Note1       - Due to a timer rollover you'll get one incorrect blink about every 50 days, and I don't care
//  Note2       - Must be more often than once every tenth of a second or it won't work
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void blinkStatusLed()
{
  unsigned long t = millis()%1000;

  if (t<100)
  {
    digitalWrite(PIN_STATUS, LED_ON);
  }
  else
  {
    digitalWrite(PIN_STATUS, LED_OFF);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setAdcParams - Sets the ADC parameters
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setAdcParams()
{
 // Since the ADC will now be triggered and read manually the setup parameters will be manually setup as needed. 
 // Manually triggering the adc allows conversions to run in the background once initiated.
 // In manual mode ADCH must be read to allow data from the next conversion to load into the result registers.
 // The Arduino system sets up some of these parameters as default.
 // Note: Performing an analogRead() may change some of the manually setup parameters.

  // ADCSRA , ADC control & status reg A:
  // ADC clock divisor
  // ADPS2  ADPS1  ADPS0  Div Factor
  // 0      0      0      2
  // 0      0      1      2          // Fastest
  // 0      1      0      4
  // 0      1      1      8
  // 1      0      0      16
  // 1      0      1      32
  // 1      1      0      64
  // 1      1      1      128        // Slowest (default)
  // Default set by wiring is 128 (1 MHz/128 = 8 Khz)
  // Since a conversion takes 13 clocks the sample rate at 8Khz/13 = 630 Hz or about 1.59 ms (with a 1 mhz clock)
  // With a 16mhz system clock the div factor will be set to 16, (adc clock = 16 mhz/16 = 1 mhz).
  // 1mhz is at the highest suggested speed of the adc clock when full resolution is not required.
  // At a 1mhz clock the 13 required cycles will give an adc sample rate of 1 mhz/13 = 76.9 khz (13 uS).
  ADCSRA &= ~(1<<ADPS0);   // 0, for a divison factor of 16 need "B100"
  ADCSRA &= ~(1<<ADPS1);   // 0
  ADCSRA |=  (1<<ADPS2);   // 1
  ADCSRA &= ~(1<<ADIE) ;   // 0, ADC Interrupt Enable
  //ADCSRA &= ~(1<<ADIF);  // 0, ADC Interupt Flag, since IE is not enabled no need to this setup
  ADCSRA &=  ~(1<<ADATE);  // 0, ADC Auto Trigger Enable
  //ADCSRA &= ~(1<<ADSC);  // 0, ADC Start Conversion, no need to setup here
  ADCSRA |=  (1<<ADEN)  ;  // 1, ADC Enable
   
  //ADMUX, ADC Mux Seletion Register
  ADMUX |=  (1<<MUX0);   // 1, adc Mux channel (ADMUX bits MUX[3-0], for ADC3(PB3) single ended need "B0011")
  ADMUX |=  (1<<MUX1);   // 1
  ADMUX &= ~(1<<MUX2);   // 0
  ADMUX &= ~(1<<MUX3);   // 0
  
  ADMUX &= ~(1<<REFS0);  // 0, adc reference (ADMUX bits REF[2-0], for Vcc need "B000")
  ADMUX &= ~(1<<REFS1);  // 0
  ADMUX &= ~(1<<REFS2);  // 0
  
  ADMUX |=  (1<<ADLAR);  // 1, when left adjust is set the upper 8 bits are found in ADCH, reading ADCH satisfies update requirement.

//ADCSRB, ADC control & status reg B:  ADCSRB default values are good..
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setPwmParams - Sets up PWM parameters for motor.  Must be called before using the pwm from PB1
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setPwmParams()
{
  // Note: Since Timer0 is used for other standard time functions use only Timer1 for motor PWM timing.

  // Disable motor enable pin 
  digitalWrite(PIN_MOT_EN, LOW);

  // Setup pwm registers for output on PB1 using the timer counter 1
  // Temporarily stop counters by setting Timer Sync Mode bit to 1 
  GTCCR |=  (1<<TSM);
  // Place Timer1 into Reset by setting Timer1 Prescaler to 1
  GTCCR |=  (1<<PSR1);
   
  // Set up timer1 prescaler for a raw pwm freq that can be further divided by the 8 bit timer1 max counter to achieve a value close to 25khz.
  // To achieve a 25khz PWM: (CK/Prescaler)/(maxCnt+1) = 25,000
  // PWM freq = freq TCK1 / (0CR1C+1) = 25,000
  // TCK1 can use PCK or CK, select with PCKE
  // Could use 64mhz /16 /168, thids needs CS1=0x05 for PCK/16, 0CR1C (maxCnt-1)=168, (This is the async clock mode)
  // Or could use 16mhz /4 /168, this needs CS1=0x03 for CK/4, 0CR1C (maxCnt-1)=168, (This is the sync clock mode)
  // use the sync mode 16mhz CK, so set PCKE to 0 in PLLCSR, (this might already be the default)
  PLLCSR &= ~(1<<PCKE); //sets timer1 input (TCK1) to 16mhz (CK), in sync clock mode

  // set up all 8-bits of TCCR1, (Timer/Counter1 Control Register)
  // set CTC1 to 1,(Clear Timer/Counter on Compare), this will clear timer1 to 0x00 on match of 0CR1C (maxCnt)   
  // set PWM1A to 1, (PWM A enable), this enables pwm based on match of 0CR1A   
  // set COM1A0 & COM1A0 to 3, (compare A output mode), this connects 0C1A to PB1, and the output will be set on a CMP A.  (PB1 must be set as an output)
  // set CS13,CS12,CS11,CS10 to 3, (timer1 prescaler) this selects TCK1/4
  TCCR1 = 1<<CTC1 | 1<<PWM1A | 3<<COM1A0 | 3<<CS10;
  
  // GTCCR,(General Timer Counter Control Register), setup/disable other timer counter 1 items
  // set PWM1B to 0, this will disable PWM1B
  // set COM1B1 & COM1B0 to 0x00, this will disable the connection of 0C1B to PB4
  GTCCR &= ~(1<<PWM1B);
  GTCCR &= ~(1<<COM1B0);
  GTCCR &= ~(1<<COM1B1);

  // set up Timer1 Max Count to T1_MAX_TIME, this sets the counter timer1 Top (max count)
  OCR1C = T1_MAX_TIME-1;
   
  // preset CMP A register value to achiev 0% on.  (values >T1_MAX_TIME will be full off)
  OCR1A = T1_MAX_TIME;
   
  // Restart Timers, clear TSM
  GTCCR &=  ~(1<<TSM);
   
  // interupts for timer1 should already be cleared as a default.
   
  // To achieve a specific PWM on PB1: 
  // Adjust OCR1A from T1_MAX_TIME to 0x00, with T1_MAX_TIME for 100% off, and 0x00 for 100% on.
}

#ifdef ENABLE_BI_LED
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//setBiLedStatus - Outputs a high, low, or pwm on the Status line, this line optionally has a bi-color LED, or two separate LEDs.
//  input color  - A predefined value corresponding to red, green, or yellow
//  returns      - void
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setBiLedStatus(byte color)
{
  if (color==BI_LED_RED)
  {
    digitalWrite(PIN_STATUS, LOW);
  }
  else if (color==BI_LED_GREEN)
  {
    digitalWrite(PIN_STATUS, HIGH);
  }
  else
  {
    analogWrite(PIN_STATUS, color);
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup - Main program init (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  setAdcParams();
  setPwmParams();

  pinMode(PIN_MOT_EN,  OUTPUT);
  pinMode(PIN_MOT_DIR, OUTPUT);
  pinMode(PIN_STATUS,  OUTPUT);
  pinMode(PIN_PEDAL,   INPUT_PULLUP);

#ifdef ENABLE_BI_LED
  setBiLedStatus(BI_LED_RED);
  delay(100);
  setBiLedStatus(BI_LED_YELLOW);
  delay(100);
  setBiLedStatus(BI_LED_GREEN);
  delay(100);
  setBiLedStatus(BI_LED_YELLOW);
  delay(100);
#endif

  delay(100);    // Let things settle
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// loop - Main program loop (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  byte    motorSpeed;
  byte motorCw;
  byte    motorPwmVal;
  byte    rawSpeedVal = analogReadSpeedDial();
  
  static boolean pedalOn     = false;
  static byte prevMotorCw    = 2;     // prevMotorCw must be started in a unique state

  blinkStatusLed();

  // Convert raw value (0-255) into a 0-127 range plus direction
  // This new mapping makes center of potentiometer 0 and both extremes 127 
  if (rawSpeedVal>127)
  {
    motorSpeed = rawSpeedVal-128;
    motorCw    = true;
  } 
  else 
  {
    motorSpeed = 127-rawSpeedVal;
    motorCw    = false;
  }

  motorSpeed = (motorSpeed < DEAD_ZONE) ? 0 : motorSpeed;

  if (digitalRead(PIN_PEDAL) == LOW)    // If pedel is pressed
  {
    digitalWrite(PIN_STATUS, LED_OFF);  // Make sure status led is off
    pedalOn = !pedalOn;                 // Toggle pedal press state

    // Pause the motor while the pedal is pressed
    motorStop();

    do  // Debounce
    { 
      delay(10);
    } while(digitalReadFast(PIN_PEDAL) == LOW);
  }

  // By forcing pedalOn when speed is 0 the wheel can be started without a pedal by turning speed to 0
  // Also prevents people from being confused why wheel isn't working
  if (motorSpeed == 0)
  {
    pedalOn = true;
  }

  if (!pedalOn)
  {
    motorSpeed = 0;
  }

  if (motorCw != prevMotorCw) // If direction has changed
  {
    motorPwmOff();  // Pause the motor
    prevMotorCw = motorCw;

    if (motorCw)
    {
      digitalWrite(PIN_MOT_DIR, HIGH);  // Set motor direction to cw
    }
    else
    {
      digitalWrite(PIN_MOT_DIR, LOW);  // Set motor direction to ccw
    }
  }

  motorRun(motorSpeed);
}

