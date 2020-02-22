#include <Arduino.h>

// --- Constants ---
// - Pins
#define PIN_LINE_IN         14

// - Intervals
#define INTERVAL_DEBUG       1000000

// - Other


// --- Module variables ---
// - Timers
IntervalTimer debugTimer;  // Timer for printing out debug information

// - Sensors
int lineThresh = 0;  // Threshold for line sensor (tape detection)

// - States
int onTape = false;  // Currently touching tape


// --- Prototypes ---
void printDebug(void);


/**
 * Initialize Teensy.
 */
void setup() {
  // Initialize serial
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial initialized.");

  // Initialize pins
  analogReadResolution(10);

  // - Input, line sensor
  pinMode(PIN_LINE_IN, INPUT);
  for (int i = 0; i < 10; i++) {
    delay(10);
    lineThresh += analogRead(PIN_LINE_IN);
  }
  lineThresh = lineThresh / 10 / 2;  // TODO make 5 a constant
  Serial.println(lineThresh);

  // Set up timers
  debugTimer.begin(printDebug, INTERVAL_DEBUG);  // Debug timer
}


/**
 * Check events and respond with services.
 */
void loop() {
  // Event: hit tape
  if (!onTape && analogRead(PIN_LINE_IN) < lineThresh) {
    onTape = true;
    Serial.println("Now touching tape");
  }
  // Event: left tape
  if (onTape && analogRead(PIN_LINE_IN) > lineThresh) {
    onTape = false;
    Serial.println("Now leaving tape");
  }
}


/**
 * Print out relevant debugging information
 */
void printDebug() {
  // Read line sensor output
  int x = analogRead(PIN_LINE_IN);
  if (x > lineThresh / 2) {
    Serial.print("high :");
  } else {
    Serial.print("low : ");
  }
  Serial.println(x);
}