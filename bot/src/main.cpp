#include <Arduino.h>

// --- Constants ---
// - Pins
const int PIN_LED = 13;

const int PIN_LINE_LEFT = 14;
const int PIN_LINE_RIGHT = 15;

const int PIN_IR_IN = 16;

const int PIN_MOTOR_LEFT_FWD = 1;
const int PIN_MOTOR_LEFT_REV = 3;
const int PIN_MOTOR_LEFT_PWM = 4;

const int PIN_MOTOR_RIGHT_FWD = 8;
const int PIN_MOTOR_RIGHT_REV = 9;
const int PIN_MOTOR_RIGHT_PWM = 10;

// - Intervals and frequencies
const int INTERVAL_DEBUG = 1000000;  // 1 Hz
const int INTERVAL_SAMPLE = 200;  // 5 KHz
const int FREQ_PWM = 450;

// - Other
const float GAIN = 0.888;
const float IR_THRESH = 20.0;

const int TAPE_NUM_BASELINES = 10;
const int TAPE_DELAY_BASELINE = 10;  // 10 ms
const int TAPE_REL_THRESH = 0.5;  // 50 %


// - Debug mode states
const int DEBUG_IR_MODE = false;
const int DEBUG_TAPE_MODE = false;
const int DEBUG_MOTOR_MODE = true;


// --- Module variables ---
// - Timers
IntervalTimer debugTimer;  // Timer for printing out debug information
IntervalTimer sampleTimer;

// - Sensors
float lineThresh = 0;  // Threshold for line sensor (tape detection)
float curIROut = 0;
float lastIROut = 0;
float curIRIn = 0;
float lastIRIn = 0;

// - States
int onIR = false;
int stateMotorForward = true; 
int motorHigh = true;


// --- Prototypes ---
void printDebug(void);
void sampleSensors(void);
void driveForward(void);
void rotateClockwiseR(void);

// TODO debug teensy overheating (or perhaps serial nonsense)

/**
 * Initialize Teensy.
 */
void setup() {
  // Initialize serial
  Serial.begin(9600);
  // while (!Serial);
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
  Serial.println(lineThresh);  // TODO debug this
  lineThresh = (lineThresh / TAPE_NUM_BASELINES) / 2;
  Serial.print("Line threshold: ");
  Serial.println(lineThresh);

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
}


/**
 * Check events and respond with services.
 */
void loop() {
  if (DEBUG_TAPE_MODE) {
    // Event: left tape sensor triggered
    if (analogRead(PIN_LINE_LEFT) < lineThresh) {
      Serial.println("Left tape sensor trigger. Reverse clockwise.");
    }
  }

  if (Serial.available()) {
    Serial.read();
    if (motorHigh == true) {
      analogWrite(PIN_MOTOR_RIGHT_PWM, 500);
      motorHigh = false;
    } else {
      analogWrite(PIN_MOTOR_RIGHT_PWM, 1000);
      motorHigh = true;
    }
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

  // Update outputs
  lastIROut = curIROut;
  curIROut = GAIN * lastIROut + GAIN * (curIRIn - lastIRIn);

  // // Output outputs
  if (curIROut > IR_THRESH) {
    onIR = true;
  }
}

void driveForward() {
  // Left wheel -- forward
  digitalWrite(PIN_MOTOR_LEFT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_REV, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, 1000);

  // Right wheel -- forward
  digitalWrite(PIN_MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 1000);
}


void rotateClockwiseR() {
  // Left wheel - stationary
  digitalWrite(PIN_MOTOR_LEFT_FWD, LOW);
  digitalWrite(PIN_MOTOR_LEFT_REV, LOW);

  // Right wheel - backwards
  digitalWrite(PIN_MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 1000);
}


/**
 * Print out relevant debugging information
 */
void printDebug() {
  if (DEBUG_IR_MODE) {
    // Read IR sensor input
    if (onIR) {
      Serial.println("IR - activated");
    } else {
      Serial.println("IR - not activated");
    }
    onIR = false;
  }
  

  if (DEBUG_TAPE_MODE) {
    // Print tape sensor output
    Serial.print("Left tape sensor: ");
    Serial.println(analogRead(PIN_LINE_LEFT));
  }

  
  if (DEBUG_MOTOR_MODE) {
    // Currently in forward state, switch to reverse
    // if (stateMotorForward) {
    //   digitalWrite(PIN_MOTOR_LEFT_FWD, LOW);
    //   digitalWrite(PIN_MOTOR_LEFT_REV, HIGH);

    //   digitalWrite(PIN_MOTOR_RIGHT_FWD, LOW);
    //   digitalWrite(PIN_MOTOR_RIGHT_REV, HIGH);

    //   stateMotorForward = false;

    // // Currently in reverse state, switch to forward
    // } else { 
    //   digitalWrite(PIN_MOTOR_LEFT_FWD, HIGH);
    //   digitalWrite(PIN_MOTOR_LEFT_REV, LOW);

    //   digitalWrite(PIN_MOTOR_RIGHT_FWD, HIGH);
    //   digitalWrite(PIN_MOTOR_RIGHT_REV, LOW);

    //   stateMotorForward = true;
    // }
  }
}