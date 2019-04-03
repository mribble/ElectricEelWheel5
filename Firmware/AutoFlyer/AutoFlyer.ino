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
#define PIN_LED       2
#define PIN_HB_C0     7
#define PIN_HB_C1     8
#define PIN_HB_C2     9
#define PIN_HB_C3    10
#define PIN_BUTTON    1
#define APIN_SHUNT   A0

#define FORWARD 0
#define BACKWARD 1
#define STOP 2

#define MOTOR_THRESHOLD 60
#define MOTOR_OFF_MS 2000
#define MOTOR_ON_MS 2000

uint8_t gDir = FORWARD;


// https://en.wikipedia.org/wiki/H_bridge
// S? are the names being used by wikipedia
// ??G are the names used on the hbridge part being used
// C? are the names used on the schematic
//           S1/P1G/C3  |  S2/N1G/C0  |  S3/P2G/C2  |  S4/N2G/C1
// Stop      closed(1)     closed(0)     closed(1)     closed(0)
// Fordward  open(0)       closed(0)     closed(1)     open(1)
// Backward  closed(1)     open(1)       open(0)       closed(0)
// Note because the hbridge part being used used n and p mosfets the
// logic values used to control open/closed are inversed for N gates
void runMotor(uint8_t dir) {
  if (dir == FORWARD) {
    digitalWrite(PIN_HB_C0, LOW);
    digitalWrite(PIN_HB_C1, HIGH);
    digitalWrite(PIN_HB_C2, HIGH);
    digitalWrite(PIN_HB_C3, LOW);
  }
  else if (dir == BACKWARD) {
    digitalWrite(PIN_HB_C0, HIGH);
    digitalWrite(PIN_HB_C1, LOW);
    digitalWrite(PIN_HB_C2, LOW);
    digitalWrite(PIN_HB_C3, HIGH);
  }
  else { // STOP
    digitalWrite(PIN_HB_C0, LOW);
    digitalWrite(PIN_HB_C1, LOW);
    digitalWrite(PIN_HB_C2, HIGH);
    digitalWrite(PIN_HB_C3, HIGH);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup - Main program init (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_HB_C0, OUTPUT);
  pinMode(PIN_HB_C1, OUTPUT);
  pinMode(PIN_HB_C2, OUTPUT);
  pinMode(PIN_HB_C3, OUTPUT);
  runMotor(STOP);
  
  pinMode(PIN_BUTTON, INPUT);
  analogReference(INTERNAL);  // Sets an internal 1.1V analog read reference
}

bool changeMotorDirection() {
  boolean ret = false;
  // Shunt resistor is 0.5 ohms.  Want to detect stall currents of over 150mA.  So 0.15A*0.5 = .075V.  Then scaling that to 1024 with 0->1.1V range gives us 70
  if (analogRead(APIN_SHUNT) >= MOTOR_THRESHOLD) {
    gDir = (gDir == FORWARD) ? BACKWARD : FORWARD;
    ret = true;
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// loop - Main program loop (this is just how Arduino code works)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if (digitalRead(PIN_BUTTON) == HIGH) {
    digitalWrite(PIN_LED, HIGH);
    gDir = (gDir == FORWARD) ? BACKWARD : FORWARD;
    delay(100);
    digitalWrite(PIN_LED, LOW);
  }

  runMotor(gDir);
  delay(MOTOR_ON_MS);
  changeMotorDirection();
  runMotor(STOP);
  delay(MOTOR_OFF_MS);
}

