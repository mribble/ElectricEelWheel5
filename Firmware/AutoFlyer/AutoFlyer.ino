// AutoFlyer
// Maurice Ribble 

// Open Source, licensed under Creative Commons Attribution 3.0 License (http://creativecommons.org/licenses/by-sa/3.0/)
// Compiled with Arduino Software 1.8.1 (http://arduino.cc) plus the attiny module
// This site explains how to setup tiny: http://highlowtech.org/?p=1695

////////////////////////////////////////////////////////////////////////////////////////////////////
// REVISIONS:
// July 25, 2018 - Maurice Ribble
//      - Initial version
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration flags
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Digital/analog pins
#define PIN_LED     2
#define PIN_MOT0    0
#define PIN_MOT1    1
#define PIN_BUTTON  3
//#define PIN_SHUNT   4
#define APIN_SHUNT  2

#define FORWARD 0
#define BACKWARD 1
#define STOP 2

uint8_t gDir = FORWARD;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup - Main program init (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_MOT0, OUTPUT);
  digitalWrite(PIN_MOT0, LOW);
  pinMode(PIN_MOT1, OUTPUT);
  digitalWrite(PIN_MOT1, LOW);
  pinMode(PIN_BUTTON, INPUT);
}

void runMotor(uint8_t dir) {
  if (dir == FORWARD) {
    digitalWrite(PIN_MOT0, HIGH);
    digitalWrite(PIN_MOT1, LOW);
  }
  else if (dir == BACKWARD) {
    digitalWrite(PIN_MOT0, LOW);
    digitalWrite(PIN_MOT1, HIGH);
  }
  else { // STOP
    digitalWrite(PIN_MOT0, LOW);
    digitalWrite(PIN_MOT1, LOW);
  }
}

void changeMotorDirection() {
  // Shunt resistor is 0.5 ohms.  Want to detect stall currents of over 200mA.  So 0.2A*0.5 = .1V.  Then scaling that to 1024 with 0->3.3V range gives us 31
  if (analogRead(APIN_SHUNT) >= 31) {
    gDir = (gDir == FORWARD) ? BACKWARD : FORWARD;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// loop - Main program loop (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if (digitalRead(PIN_BUTTON) == HIGH) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
  }

  runMotor(gDir);
  delay(100);
  changeMotorDirection();
  runMotor(STOP);
  
  delay(1000);
}

