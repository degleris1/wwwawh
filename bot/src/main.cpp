#include <Arduino.h>
#include <Metro.h>

// --- Constants ---
// - Pins
const int PIN_LED = 13;

const int PIN_LINE_LEFT = 14;
const int PIN_LINE_RIGHT = 15;

const int PIN_IR_IN = 16;

const int PIN_MOTOR_LEFT_FWD = 8;
const int PIN_MOTOR_LEFT_REV = 9;
const int PIN_MOTOR_LEFT_PWM = 10;

const int PIN_MOTOR_RIGHT_FWD = 1;
const int PIN_MOTOR_RIGHT_REV = 3;
const int PIN_MOTOR_RIGHT_PWM = 4;

// - Intervals and frequencies
const int INTERVAL_DEBUG = 1000000;  // 4 Hz
const int INTERVAL_SAMPLE = 200;  // 5 KHz
const int FREQ_PWM = 450;

// - IR sensor
const float GAIN = 0.888;
const float IR_THRESH = 10.0;
const float INTEGRAL_GAIN = 0.99;


const int TAPE_NUM_BASELINES = 10;
const int TAPE_DELAY_BASELINE = 10;  // 10 ms
const int TAPE_REL_THRESH = 0.5;  // 50 %

// TODO all caps
enum State {
  findingBeacon,
  attackingLeftWall,
  reversing,
  rotating,
  attackingRightWall,
  stopped
};


// --- Prototypes ---
void printDebug(void);
void sampleSensors(void);

void stopMotors(void);
void driveForwardL(void);
void driveForwardR(void);
void driveReverseL(void);
void rotateClockwiseR(void);


// --- Module variables ---
// - Timers
IntervalTimer debugTimer;  // Timer for printing out debug information
IntervalTimer sampleTimer;
Metro stateMetro = Metro(100);

// - Sensors
float lineThresh = 0;  // Threshold for line sensor (tape detection)
float curIRSum = 0;
float lastIRSum = 0;

float curIROut = 0;
float lastIROut = 0;

float curIRIn = 0;
float lastIRIn = 0;


// - States
State state = findingBeacon;


/**
 * Initialize Teensy.
 */
void setup() {
  // Initialize serial
  Serial.begin(9600);
  // while(!Serial);
  Serial.println("Serial initialized.");

  // Initialize pins
  analogReadResolution(10);
  analogWriteResolution(10);

  // - LED on
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // - Input, line sensor
  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);
  for (int i = 0; i < TAPE_NUM_BASELINES; i++) {
    delay(TAPE_DELAY_BASELINE);
    lineThresh += analogRead(PIN_LINE_LEFT);
  }

  //Serial.println(lineThresh);  // TODO debug this
  lineThresh = (lineThresh / TAPE_NUM_BASELINES) / 2;
  //Serial.print("Line threshold: ");
  //Serial.println(lineThresh);

  // - Input, IR sensor
  pinMode(PIN_IR_IN, INPUT);

  // - Motors
  pinMode(PIN_MOTOR_LEFT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_REV, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  analogWriteFrequency(PIN_MOTOR_LEFT_PWM, FREQ_PWM);
  
  pinMode(PIN_MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_REV, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  analogWriteFrequency(PIN_MOTOR_RIGHT_PWM, FREQ_PWM);

  rotateClockwiseR();

  // Set up timers
  debugTimer.begin(printDebug, INTERVAL_DEBUG);  // Debug timer
  sampleTimer.begin(sampleSensors, INTERVAL_SAMPLE);  // Sample timer

  delay(1000);
}


/**
 * Check events and respond with services.
 */
void loop() {
  // Event: IR sensor triggered from findingBeacon state
  // Service: Move to drive forward state (with left motor overpowered)
  if ((state == findingBeacon) && (curIRSum > 35.0)) {
    driveForwardL();
    state = attackingLeftWall;
    stateMetro.interval(7.5 * 1000);
    stateMetro.reset();
  }

  // Event: timer expired from attackingLeftWall state
  // Service: Move to reverse state
  if ((state == attackingLeftWall) && stateMetro.check()) {
    driveReverseL();
    state = reversing;
    stateMetro.interval(2 * 1000);
    stateMetro.reset();
  }

  if ((state == reversing) && stateMetro.check()) {
    rotateClockwiseR();
    state = rotating;
    stateMetro.interval(2.5 * 1000);
    stateMetro.reset();
  }

  if ((state == rotating) && stateMetro.check()) {
    driveForwardR();
    state = attackingRightWall;
    stateMetro.interval(15 * 1000);
    stateMetro.reset();
  }

  if ((state == attackingRightWall) && stateMetro.check()) {
    stopMotors();
    state = stopped;
    // Et fini
  }
  

  // Event: right tape sensor triggered
  // if (analogRead(PIN_LINE_RIGHT) < lineThresh) {
  //   Serial.println("Right tape sensor trigger. Reverse counterclockwise.");
  // }
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
  digitalWrite(PIN_MOTOR_LEFT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_REV, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, 1023);

  // Right wheel -- forward, half
  digitalWrite(PIN_MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 500);
}

void driveForwardR() {
  // Left wheel -- forward, half
  digitalWrite(PIN_MOTOR_LEFT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_REV, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, 1000);

  // Right wheel -- forward
  digitalWrite(PIN_MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 1023);
}


void driveReverseL() {
  // Left wheel -- reverse
  digitalWrite(PIN_MOTOR_LEFT_FWD, LOW);
  digitalWrite(PIN_MOTOR_LEFT_REV, HIGH);
  analogWrite(PIN_MOTOR_LEFT_PWM, 1023);

  // Right wheel -- reverse, half
  digitalWrite(PIN_MOTOR_RIGHT_FWD, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_REV, HIGH);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 500);
}


void rotateClockwiseR() {
  // Left wheel - forwards
  digitalWrite(PIN_MOTOR_LEFT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_REV, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, 800);

  // Right wheel - backwards
  digitalWrite(PIN_MOTOR_RIGHT_FWD, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_REV, HIGH);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 800);
}


void stopMotors() {
  // Left wheel - stationary
  digitalWrite(PIN_MOTOR_LEFT_FWD, LOW);
  digitalWrite(PIN_MOTOR_LEFT_REV, LOW);

  // Right wheel - backwards
  digitalWrite(PIN_MOTOR_RIGHT_FWD, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
}


/**
 * Print out relevant debugging information
 */
void printDebug() {
  // Read IR sensor input
  Serial.print("IR Sum -- ");
  Serial.println(curIRSum);

  // Print tape sensor output
  // Serial.print("Left tape sensor: ");
  // Serial.println(analogRead(PIN_LINE_LEFT));
}