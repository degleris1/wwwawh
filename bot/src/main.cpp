#include <Arduino.h>
#include <Metro.h>



// TODO
// - Reduce line adjusting state to two states (instead of 4)


// Event: right tape sensor triggered
  // if (analogRead(PIN_LINE_RIGHT) < lineThresh) {
  //   Serial.println("Right tape sensor trigger. Reverse counterclockwise.");
  // }



// --- Enumerated types ---
enum State {
  FINDING_BEACON,

  LEFT_ATTACKING,
  REVERSING,
  ROTATING,
  RIGHT_ATTACKING,

  ADJUSTING_RIGHT,
  ADJUSTING_LEFT,

  STOPPED
};


// --- Constants ---
// Pins
const int PIN_LED = 13;

const int PIN_LINE_LEFT = 21;
const int PIN_LINE_RIGHT = 19;

const int PIN_IR_IN = 16;

const int PIN_MOTOR_LEFT_FWD = 8;
const int PIN_MOTOR_LEFT_REV = 9;
const int PIN_MOTOR_LEFT_PWM = 10;

const int PIN_MOTOR_RIGHT_FWD = 1;
const int PIN_MOTOR_RIGHT_REV = 3;
const int PIN_MOTOR_RIGHT_PWM = 4;

// Intervals and frequencies
const int INTERVAL_DEBUG = 1000000;  // 4 Hz
const int INTERVAL_SAMPLE = 200;  // 5 KHz
const int FREQ_PWM = 450;

// IR sensor
const float GAIN = 0.888;
const float IR_THRESH = 10.0;
const float INTEGRAL_GAIN = 0.99;
const float IR_SUM_THRESH = 35.0;

// Tape sensor
const int TAPE_NUM_BASELINES = 10;
const int TAPE_DELAY_BASELINE = 10;  // 10 ms
const float TAPE_REL_THRESH = 0.8;  // 50 %

// Motor
const int MOTOR_FWD = 1;
const int MOTOR_REV = -1;
const int MOTOR_STILL = 0;
const int MOTOR_POWER_FULL = 1023;
const int MOTOR_POWER_HALF = 500;
const int MOTOR_POWER_ROTATE = 800;

// Debug mode --- set to false for competitions and demonstrations
const int DEBUG_MODE = false;


// --- Prototypes ---
void setupMotors(void);
void setupSensors(void);

void printDebug(void);
void sampleSensors(void);

void setMotors(int, int, int, int);
void stopMotors(void);
void driveForwardL(void);
void driveForwardR(void);
void driveReverseL(void);
void rotateClockwiseR(void);
void adjustReverseLeft(void);
void adjustReverseRight(void);


// --- Module variables ---
// Timers
IntervalTimer debugTimer;  // Timer for printing out debug information
IntervalTimer sampleTimer;  // Timer for IR sensor sampling
Metro stateMetro = Metro(100);  // Timer for state transitions
Metro adjustMetro = Metro(100);  // Timer for tape sensor adjustments

// Sensors
float lineThreshLeft = 0;  // Threshold for line sensor (tape detection)
float lineThreshRight = 0;

float curIRSum = 0;
float lastIRSum = 0;

float curIROut = 0;
float lastIROut = 0;

float curIRIn = 0;
float lastIRIn = 0;

// States
State state = FINDING_BEACON;
State returnState = RIGHT_ATTACKING;


/**
 * Initialize Teensy.
 */
void setup() {
  if (DEBUG_MODE) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Serial initialized.");
  }
  
  // Initialize pins
  analogReadResolution(10);
  analogWriteResolution(10);

  // Activate Teensy LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // Set up sensors and motors
  setupSensors();
  setupMotors();

  // Set up timers
  if (DEBUG_MODE) debugTimer.begin(printDebug, INTERVAL_DEBUG);  // Debug timer
  sampleTimer.begin(sampleSensors, INTERVAL_SAMPLE);  // Sample timer

  // Leave time for the switch operator to move away from the bot
  delay(1000);  
}


void setupSensors() {
  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);
  pinMode(PIN_IR_IN, INPUT);

  // Line senor calibration  
  for (int i = 0; i < TAPE_NUM_BASELINES; i++) {
    delay(TAPE_DELAY_BASELINE);
    lineThreshLeft += analogRead(PIN_LINE_LEFT);
    lineThreshRight += analogRead(PIN_LINE_RIGHT);
  }
  lineThreshLeft = TAPE_REL_THRESH * (lineThreshLeft / TAPE_NUM_BASELINES);
  lineThreshRight = TAPE_REL_THRESH * (lineThreshRight / TAPE_NUM_BASELINES);

  if (DEBUG_MODE) {
    Serial.print("Left line threshold: ");
    Serial.println(lineThreshLeft);
    Serial.print("Right line threshold: ");
    Serial.println(lineThreshRight);
  }
}


void setupMotors() {
  pinMode(PIN_MOTOR_LEFT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_REV, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  analogWriteFrequency(PIN_MOTOR_LEFT_PWM, FREQ_PWM);
  
  pinMode(PIN_MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_REV, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  analogWriteFrequency(PIN_MOTOR_RIGHT_PWM, FREQ_PWM);

  rotateClockwiseR();
}


/**
 * Check events and respond with services.
 */
void loop() {
  // IR sensor triggered while finding beacon
  // ---> Attack left wall
  if ((state == FINDING_BEACON) && (curIRSum > IR_SUM_THRESH)) {
    driveForwardL();
    state = LEFT_ATTACKING;
    stateMetro.interval(7.5 * 1000);
    stateMetro.reset();
  }
  // Timer expired, move to reverse
  if ((state == LEFT_ATTACKING) && stateMetro.check()) {
    driveReverseL();
    state = REVERSING;
    stateMetro.interval(2 * 1000);
    stateMetro.reset();
  }
  // Timer expired, move to rotating
  if ((state == REVERSING) && stateMetro.check()) {
    rotateClockwiseR();
    state = ROTATING;
    stateMetro.interval(2.5 * 1000);
    stateMetro.reset();
  }
  // Timer expired, move to left attack
  if ((state == ROTATING) && stateMetro.check()) {
    driveForwardR();
    state = RIGHT_ATTACKING;
    stateMetro.interval(100 * 1000);  // TODO make a full 2:10
    stateMetro.reset();
  }
  // Final timer goes off, stop
  if ((state == RIGHT_ATTACKING) && stateMetro.check()) {
    stopMotors();
    state = STOPPED;
    // Et fini.
  }


  // Tape sensing logic
  // Left tape sensor goes off
  if ((state == RIGHT_ATTACKING || state == LEFT_ATTACKING) 
      && analogRead(PIN_LINE_LEFT) > lineThreshLeft) {
    adjustReverseRight();
    returnState = state;
    state = ADJUSTING_RIGHT;
    adjustMetro.interval(1 * 1000);
    adjustMetro.reset();
  }
  // Right tape sensor goes off (hit outer edge)
  if ((state == RIGHT_ATTACKING || state == LEFT_ATTACKING) 
      && analogRead(PIN_LINE_RIGHT) > lineThreshRight) {
    adjustReverseLeft();
    returnState = state;
    state = ADJUSTING_LEFT;
    adjustMetro.interval(1 * 1000);
    adjustMetro.reset();
  }
  // Return to state
  if ((state == ADJUSTING_LEFT || state == ADJUSTING_RIGHT) 
      && adjustMetro.check()) {
    if (returnState == LEFT_ATTACKING) {
      driveForwardL();
    } else {  // returnState == RIGHT_ATTACKING
      driveForwardR();
    }  
    state = returnState;
  }
}


void sampleSensors() {
  // Update inputs
  lastIRIn = curIRIn;
  curIRIn = analogRead(PIN_IR_IN);
  lastIRSum = curIRSum;

  // Update outputs
  lastIROut = curIROut;
  curIROut = GAIN * lastIROut + GAIN * (curIRIn - lastIRIn);

  // Update sums
  if (curIROut > IR_THRESH) {
    curIRSum = INTEGRAL_GAIN * lastIRSum + 1.0;
  } else {
    curIRSum = INTEGRAL_GAIN * lastIRSum;
  }
}


void driveForwardL() {
  // Left wheel -- forward
  // Right wheel -- forward, half
  setMotors(MOTOR_FWD, MOTOR_POWER_FULL, MOTOR_FWD, MOTOR_POWER_HALF);
}


void driveForwardR() {
  // Left wheel -- forward, half
  // Right wheel -- forward
  setMotors(MOTOR_FWD, MOTOR_POWER_FULL, MOTOR_FWD, MOTOR_POWER_FULL);
}


void driveReverseL() {
  // Left wheel -- reverse
  // Right wheel -- reverse, half
  setMotors(MOTOR_REV, MOTOR_POWER_FULL, MOTOR_REV, MOTOR_POWER_HALF);
}


void rotateClockwiseR() {
  // Left wheel - forwards
  // Right wheel - backwards
  setMotors(MOTOR_FWD, MOTOR_POWER_ROTATE, MOTOR_REV, MOTOR_POWER_ROTATE);
}


void adjustReverseLeft() {
  // Left wheel - reverse
  // Right wheel - reverse, half power
  setMotors(MOTOR_REV, MOTOR_POWER_FULL, MOTOR_REV, MOTOR_POWER_HALF);
}


void adjustReverseRight() {
  // Left wheel - reverse, half power
  // Right wheel - reverse
  setMotors(MOTOR_REV, MOTOR_POWER_HALF, MOTOR_REV, MOTOR_POWER_FULL);
}


void stopMotors() {
  setMotors(MOTOR_STILL, 0, MOTOR_STILL, 0);
}


void setMotors(int leftDir, int leftPower, int rightDir, int rightPower) {
  // Direction
  if (leftDir == MOTOR_FWD) {
    digitalWrite(PIN_MOTOR_LEFT_FWD, HIGH);
    digitalWrite(PIN_MOTOR_LEFT_REV, LOW);
  } else if (leftDir == MOTOR_REV) {
    digitalWrite(PIN_MOTOR_LEFT_FWD, LOW);
    digitalWrite(PIN_MOTOR_LEFT_REV, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_LEFT_FWD, LOW);
    digitalWrite(PIN_MOTOR_LEFT_REV, LOW);
  }
  if (rightDir == MOTOR_FWD) {
    digitalWrite(PIN_MOTOR_RIGHT_FWD, HIGH);
    digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
  } else if (rightDir == MOTOR_REV) {
    digitalWrite(PIN_MOTOR_RIGHT_FWD, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_REV, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_RIGHT_FWD, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
  }

  // Power
  analogWrite(PIN_MOTOR_LEFT_PWM, leftPower);
  analogWrite(PIN_MOTOR_RIGHT_PWM, rightPower);
}


/**
 * Print out relevant debugging information
 */
void printDebug() {
  // Read IR sensor input
  Serial.print("IR Sum -- ");
  Serial.println(curIRSum);

  // Print tape sensor output
  float tapeVal = analogRead(PIN_LINE_LEFT);
  Serial.print("Left tape sensor: ");
  Serial.print(tapeVal);
  if (tapeVal > lineThreshLeft) {
    Serial.println(" (HIGH)");
  } else {
    Serial.println(" (low)");
  }

  tapeVal = analogRead(PIN_LINE_RIGHT);
  Serial.print("Right tape sensor: ");
  Serial.print(tapeVal);
  if (tapeVal > lineThreshRight) {
    Serial.println(" (HIGH)");
  } else {
    Serial.println(" (low)");
  }

}