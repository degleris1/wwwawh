#include <Arduino.h>

// --- Constants ---
// - Pins
const int PIN_LED = 13;
const int PIN_LINE_IN = 14;
const int PIN_IR_IN = 16;
const int PIN_IR_OUT = 20;

// - Intervals
// 1 Hz
const int INTERVAL_DEBUG = 1000000;
// 5 KHz
const int INTERVAL_SAMPLE = 200;

// - Other
const float GAIN = 0.888;


// --- Module variables ---
// - Timers
IntervalTimer debugTimer;  // Timer for printing out debug information
IntervalTimer sampleTimer;

// - Sensors
int lineThresh = 0;  // Threshold for line sensor (tape detection)
float curIROut = 0;
float lastIROut = 0;
float curIRIn = 0;
float lastIRIn = 0;

// - States
int onTape = false;  // Currently touching tape
int onIR = false;

// --- Prototypes ---
void printDebug(void);
void sampleSensors(void);

// TODO debug teensy overheating (or perhaps serial nonsense)

/**
 * Initialize Teensy.
 */
void setup() {
  // Initialize serial
  // Serial.begin(9600);
  // while (!Serial);
  // Serial.println("Serial initialized.");

  // Initialize pins
  analogReadResolution(10);

  // - LED on
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // - Input, line sensor
  pinMode(PIN_LINE_IN, INPUT);
  for (int i = 0; i < 10; i++) {
    delay(10);
    lineThresh += analogRead(PIN_LINE_IN);
  }
  lineThresh = lineThresh / 10 / 2;  // TODO make 5 a constant
  // Serial.println(lineThresh);

  // - Input, IR sensor
  pinMode(PIN_IR_IN, INPUT);
  pinMode(PIN_IR_OUT, OUTPUT);

  // Set up timers
  debugTimer.begin(printDebug, INTERVAL_DEBUG);  // Debug timer
  sampleTimer.begin(sampleSensors, INTERVAL_SAMPLE);  // Sample timer
}


/**
 * Check events and respond with services.
 */
void loop() {
  // Event: entering tape
  if (!onTape && analogRead(PIN_LINE_IN) < lineThresh) {
    onTape = true;
    // Serial.println("Now touching tape");
  }

  // Event: leaving tape
  if (onTape && analogRead(PIN_LINE_IN) > lineThresh) {
    onTape = false;
    // Serial.println("Now leaving tape");
  }

  // Event: facing IR sensor
  if (!onIR && curIROut > 20.0) {
    onIR = 1;
    digitalWrite(PIN_IR_OUT, HIGH);
  }
}


void sampleSensors() {
  // Update inputs
  lastIRIn = curIRIn;
  curIRIn = analogRead(PIN_IR_IN);

  // Update outputs
  lastIROut = curIROut;
  curIROut = GAIN * lastIROut + GAIN * (curIRIn - lastIRIn);

  // // Output outputs
  // if (curIROut > 15.0) {
  //   digitalWrite(PIN_IR_OUT, 1);
  // } else {
  //   digitalWrite(PIN_IR_OUT, 0);
  // }
}


/**
 * Print out relevant debugging information
 */
void printDebug() {
  // Read line sensor input
  // int x = analogRead(PIN_LINE_IN);
  // if (x > lineThresh / 2) {
  //   Serial.print("high :");
  // } else {
  //   Serial.print("low : ");
  // }
  // Serial.println(x);

  // Read IR sensor input
  // Serial.print("IR sensor - ");
  // Serial.println(analogRead(PIN_IR_IN));
  //Serial.print("curIROut - ");
  //Serial.println(curIROut);
}