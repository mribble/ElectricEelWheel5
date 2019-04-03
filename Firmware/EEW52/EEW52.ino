// eew5
// Maurice Ribble 
// August 1, 2018
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
// 1.00 Oct 19, 2018 - Maurice Ribble
//      Port from EEW 5.1 with initial modifications

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration flags
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Digital/analog pins
#define PIN_STATUS            0    // Green status LED output     -- LOW=off       HIGH=on
#define PIN_MOT_EN            1    // Enable motor output         -- LOW=Stop     HIGH=Run
#define PIN_PEDAL             2    // Foot pedal switch input     -- LOW=Pressed  HIGH=Not Pressed
#define APIN_SPEED            3    // Speed control dial analog input (ADC3 = PB3)
#define PIN_MOT_DIR           4    // Direction motor output      -- LOW=CCW      HIGH=CW

// Green Status LED
#define LED_OFF LOW
#define LED_ON  HIGH

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
inline byte motorRun(byte motorSpeed)
{
  static byte prevMotorSpeed = 0;

  if (motorSpeed < prevMotorSpeed) {
    prevMotorSpeed--;
    motorSpeed = prevMotorSpeed;
  }
  else if (motorSpeed > prevMotorSpeed) {
    prevMotorSpeed++;
    if (prevMotorSpeed < DEAD_ZONE) {  // Jump motor to slow speed so we don't have lag starting motor
      prevMotorSpeed = DEAD_ZONE;
    }
    motorSpeed = prevMotorSpeed;
  }
  // This delay can happen a max of 127 times so the ramp time to reach max speed is 127*delay
  delay(16);
 
  //calculate pwm value to use for compare A (OCR1A). Maps speedIndex to resolution of counter timer1, (Note: T1_MAX_TIME is full off, 0 is full on)
  byte motorPwmVal = (motorSpeed == 0) ? T1_MAX_TIME : map(motorSpeed,DEAD_ZONE,127,T1_MAX_TIME-SLOW_ZONE,0);

  OCR1A = motorPwmVal;  // Adjusts pwm speed
  return motorSpeed;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup - Main program init (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  uint8_t i;
  setAdcParams();
  setPwmParams();

  pinMode(PIN_MOT_EN,  OUTPUT);
  pinMode(PIN_MOT_DIR, OUTPUT);
  pinMode(PIN_STATUS,  OUTPUT);
  pinMode(PIN_PEDAL,   INPUT_PULLUP);

  // Blink the LED 3 times during startup
  for (i=0; i<3; ++i) {
    digitalWrite(PIN_STATUS, LED_ON);
    delay(200);
    digitalWrite(PIN_STATUS, LED_OFF);
    delay(200);
  }
  digitalWrite(PIN_STATUS, LED_ON);

  motorPwmOn();
  digitalWrite(PIN_MOT_EN, HIGH);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// loop - Main program loop (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  byte motorSpeed;
  byte motorCw;
  byte rawSpeedVal = analogReadSpeedDial();

  static boolean pedalOn = false;
  static byte prevMotorCw = 2;  // prevMotorCw must be started in a unique state

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
    pedalOn = !pedalOn;                 // Toggle pedal press state

    do  // Debounce
    { 
      motorRun(0);
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
    while(motorRun(0) != 0){};  // Ramp down motor
    prevMotorCw = motorCw;

    if (motorCw)
    {
      digitalWrite(PIN_MOT_DIR, LOW);  // Set motor direction to cw
    }
    else
    {
      digitalWrite(PIN_MOT_DIR, HIGH);  // Set motor direction to ccw
    }
  }

  motorRun(motorSpeed);
}

