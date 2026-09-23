/*
  PROJECT 7 - PEDESTRIAN DETECTION VEHICLE
  Arduino Uno Safety / Motor Controller

  Hardware:
    Arduino Uno
    L298N motor driver
    4 x 6V DC gear motors
    HC-SR04 ultrasonic sensor
    Buzzer
    Red LED
    Raspberry Pi communication over USB Serial

  NOTE:
    Arduino Uno GPIO is 5V, which matches the HC-SR04's 5V logic directly --
    no voltage divider is needed on ECHO (unlike a 3.3V board such as the Due).

  Pi commands:

    HB
    ZONE,FAR,2.50,0.95
    ZONE,CAUTION,1.50,0.90
    ZONE,DANGER,0.80,0.95
    CLEAR
    RESET

  Safety:
    - Heartbeat missing >2000 ms -> STOP
    - Ultrasonic <100 cm -> STOP
    - Camera DANGER -> STOP
    - STOP is latched
    - Resume only after CLEAR for 2 seconds
*/

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
//
// Pins 0/1 (RX/TX) are reserved for Serial communication with the Pi
// and are never used as GPIO here.
//
// The Uno only supports hardware external interrupts on pins 2 and 3
// (unlike the Due, which supports them on every digital pin), so the
// encoder inputs are pinned there.

// ---------- L298N ----------
// Left motor pair
const int ENA  = 5;     // PWM
const int IN1  = 7;
const int IN2  = 8;

// Right motor pair
const int ENB  = 6;     // PWM
const int IN3  = 9;
const int IN4  = 10;

// ---------- HC-SR04 ----------
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

// ---------- Warning devices ----------
const int BUZZER_PIN = 13;
const int LED_PIN    = A0;

// ---------- Optional encoder sensors ----------
// One sensor for each side for now.
// We can add the other two later.
const int LEFT_ENCODER_PIN  = 2;   // INT0
const int RIGHT_ENCODER_PIN = 3;   // INT1


// ============================================================
// SETTINGS
// ============================================================

const int FAR_SPEED     = 150;  // 0-255
const int CAUTION_SPEED = 60;   // ~40%

const float DANGER_DISTANCE_CM = 100.0;

// Safety heartbeat
//
// *** TEMPORARY BENCH-TEST VALUE -- REVERT TO 2000 BEFORE ANY REAL DRIVING ***
// 30000ms so manual Serial Monitor testing (typing HB by hand) isn't a race
// against the clock. This is NOT safe for actual operation: if the Pi ever
// crashes or disconnects while driving, the vehicle would keep going for up
// to 30 seconds before stopping. Set back to 2000 once the Pi is doing the
// heartbeat automatically again. Keep in sync with shared/config.py's
// heartbeat_timeout_ms when you do.
const unsigned long HEARTBEAT_TIMEOUT_MS = 30000;

// Resume requirements
const unsigned long CLEAR_TIME_MS = 2000;


// ============================================================
// STATE
// ============================================================

enum Zone
{
  ZONE_FAR,
  ZONE_CAUTION,
  ZONE_DANGER
};

Zone currentZone = ZONE_DANGER;

// Emergency stop is deliberately latched.
bool emergencyLatched = true;

bool cameraClear = false;
bool clearCommandReceived = false;

float cameraDistance = 999.0;
float cameraConfidence = 0.0;

float ultrasonicDistance = 999.0;

unsigned long lastHeartbeat = 0;
unsigned long clearStartTime = 0;

bool ultrasonicDanger = false;
bool heartbeatDanger = true;


// ============================================================
// ENCODER COUNTERS
// ============================================================

volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;

void leftEncoderISR()
{
  leftPulses++;
}

void rightEncoderISR()
{
  rightPulses++;
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


void driveForward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  // Left side forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right side forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
}


// ============================================================
// ULTRASONIC
// ============================================================

float readUltrasonicCM()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  // Timeout prevents the controller from hanging.
  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 25000UL);

  if (duration == 0)
  {
    return 999.0;
  }

  float distance = duration * 0.0343 / 2.0;

  return distance;
}


// ============================================================
// WARNING DEVICES
// ============================================================

void updateWarningDevices()
{
  if (emergencyLatched)
  {
    digitalWrite(LED_PIN, HIGH);

    // Buzzer ON during emergency
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else if (currentZone == ZONE_CAUTION)
  {
    digitalWrite(LED_PIN, LOW);

    // Simple caution beep
    if ((millis() / 250) % 2 == 0)
      digitalWrite(BUZZER_PIN, HIGH);
    else
      digitalWrite(BUZZER_PIN, LOW);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}


// ============================================================
// EMERGENCY STOP
// ============================================================

void emergencyStop(const char *reason)
{
  stopMotors();

  emergencyLatched = true;

  // A fresh emergency invalidates any resume timer that was already in
  // progress -- otherwise a DANGER that interrupts a pending CLEAR window
  // could let the vehicle resume using a stale, pre-emergency timestamp.
  clearCommandReceived = false;
  clearStartTime = 0;

  Serial.print("EMERGENCY_STOP,");
  Serial.println(reason);
}


// ============================================================
// SERIAL COMMAND PROCESSING
// ============================================================

void processCommand(char *command)
{
  // Trim in place (equivalent to String::trim(), but no allocation).
  while (*command == ' ' || *command == '\t')
    command++;

  size_t len = strlen(command);
  while (len > 0 && (command[len - 1] == ' ' || command[len - 1] == '\t'))
  {
    command[len - 1] = '\0';
    len--;
  }

  if (len == 0)
    return;


  // ----------------------------------------------------------
  // HEARTBEAT
  // ----------------------------------------------------------

  if (strcmp(command, "HB") == 0)
  {
    lastHeartbeat = millis();
    heartbeatDanger = false;

    Serial.println("HB_OK");

    return;
  }


  // ----------------------------------------------------------
  // RESET
  // ----------------------------------------------------------

  if (strcmp(command, "RESET") == 0)
  {
    emergencyLatched = true;
    clearCommandReceived = false;
    clearStartTime = 0;

    stopMotors();

    Serial.println("RESET_OK");

    return;
  }


  // ----------------------------------------------------------
  // CLEAR
  // ----------------------------------------------------------

  if (strcmp(command, "CLEAR") == 0)
  {
    clearCommandReceived = true;

    if (clearStartTime == 0)
    {
      clearStartTime = millis();
    }

    Serial.println("CLEAR_RECEIVED");

    return;
  }


  // ----------------------------------------------------------
  // ZONE COMMAND
  //
  // Example:
  // ZONE,FAR,2.50,0.95
  // ----------------------------------------------------------

  if (strncmp(command, "ZONE,", 5) == 0)
  {
    // strtok splits the buffer in place -- safe here since `command`
    // points at our own mutable serialBuffer, not a string literal.
    strtok(command, ",");               // "ZONE" (discarded)
    char *zoneText = strtok(NULL, ",");
    char *distanceText = strtok(NULL, ",");
    char *confidenceText = strtok(NULL, ",");

    if (zoneText == NULL || distanceText == NULL || confidenceText == NULL)
    {
      Serial.println("BAD_ZONE_COMMAND");
      return;
    }

    cameraDistance = atof(distanceText);
    cameraConfidence = atof(confidenceText);


    if (strcmp(zoneText, "FAR") == 0)
    {
      currentZone = ZONE_FAR;
      cameraClear = true;
    }
    else if (strcmp(zoneText, "CAUTION") == 0)
    {
      currentZone = ZONE_CAUTION;
      cameraClear = true;
    }
    else if (strcmp(zoneText, "DANGER") == 0)
    {
      currentZone = ZONE_DANGER;
      cameraClear = false;

      emergencyStop("CAMERA_DANGER");
    }
    else
    {
      Serial.println("BAD_ZONE");
      return;
    }

    return;
  }


  Serial.println("UNKNOWN_COMMAND");
}


// ============================================================
// SERIAL RECEIVER
// ============================================================

// Fixed-size buffer instead of the Arduino String class: String's heap
// allocations fragment the Uno's tiny 2KB SRAM over enough command
// cycles, eventually failing and locking up the sketch -- this is what
// was causing the connection to work fine for a while and then
// permanently stop responding, with no USB-level event involved at
// all. A plain char buffer never allocates, so it can't fragment.
char serialBuffer[101];
uint8_t serialLen = 0;

void readSerialCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (serialLen > 0)
      {
        serialBuffer[serialLen] = '\0';
        processCommand(serialBuffer);
        serialLen = 0;
      }
    }
    else
    {
      // Prevent runaway buffer
      if (serialLen < sizeof(serialBuffer) - 1)
      {
        serialBuffer[serialLen++] = c;
      }
      else
      {
        serialLen = 0;
      }
    }
  }
}


// ============================================================
// SAFETY CHECKS
// ============================================================

void updateSafety()
{
  unsigned long now = millis();


  // ----------------------------------------------------------
  // HEARTBEAT TIMEOUT
  // ----------------------------------------------------------

  if (now - lastHeartbeat > HEARTBEAT_TIMEOUT_MS)
  {
    if (!heartbeatDanger)
    {
      heartbeatDanger = true;

      emergencyStop("HEARTBEAT_TIMEOUT");
    }
  }


  // ----------------------------------------------------------
  // ULTRASONIC
  // ----------------------------------------------------------

  ultrasonicDistance = readUltrasonicCM();

  if (ultrasonicDistance < DANGER_DISTANCE_CM)
  {
    ultrasonicDanger = true;

    emergencyStop("ULTRASONIC_DANGER");
  }
  else
  {
    ultrasonicDanger = false;
  }
}


// ============================================================
// MOTOR STATE MACHINE
// ============================================================

void updateMotorState()
{
  // ----------------------------------------------------------
  // Any safety fault = STOP
  // ----------------------------------------------------------

  if (heartbeatDanger)
  {
    stopMotors();
    return;
  }

  if (ultrasonicDanger)
  {
    stopMotors();
    return;
  }

  if (currentZone == ZONE_DANGER)
  {
    stopMotors();
    return;
  }


  // ----------------------------------------------------------
  // LATCHED EMERGENCY STOP
  // ----------------------------------------------------------

  if (emergencyLatched)
  {
    stopMotors();

    // Need camera clear command
    if (!clearCommandReceived)
    {
      return;
    }

    // Start 2-second clear timer
    if (clearStartTime == 0)
    {
      clearStartTime = millis();
      return;
    }

    // Wait 2 seconds
    if (millis() - clearStartTime >= CLEAR_TIME_MS)
    {
      emergencyLatched = false;
      clearCommandReceived = false;
      clearStartTime = 0;

      Serial.println("RESUMED");
    }
    else
    {
      stopMotors();
      return;
    }
  }


  // ----------------------------------------------------------
  // NORMAL DRIVING
  // ----------------------------------------------------------

  if (currentZone == ZONE_FAR)
  {
    driveForward(FAR_SPEED);
  }
  else if (currentZone == ZONE_CAUTION)
  {
    driveForward(CAUTION_SPEED);
  }
  else
  {
    stopMotors();
  }
}


// ============================================================
// STATUS OUTPUT
// ============================================================

unsigned long lastStatus = 0;

void printStatus()
{
  if (millis() - lastStatus < 1000)
    return;

  lastStatus = millis();

  Serial.print("STATUS,ZONE=");

  if (currentZone == ZONE_FAR)
    Serial.print("FAR");
  else if (currentZone == ZONE_CAUTION)
    Serial.print("CAUTION");
  else
    Serial.print("DANGER");

  Serial.print(",CAM=");
  Serial.print(cameraDistance);

  Serial.print(",ULTRA=");
  Serial.print(ultrasonicDistance);

  Serial.print(",LATCH=");
  Serial.print(emergencyLatched ? 1 : 0);

  Serial.print(",HB=");
  Serial.println(heartbeatDanger ? 0 : 1);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // USB serial to Raspberry Pi
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Warning devices
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Encoders
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_PIN),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_PIN),
    rightEncoderISR,
    RISING
  );

  // FAIL SAFE:
  // Robot starts stopped and latched.
  stopMotors();

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  lastHeartbeat = millis();

  Serial.println();
  Serial.println("================================");
  Serial.println("PROJECT 7 UNO CONTROLLER");
  Serial.println("READY - MOTORS STOPPED");
  Serial.println("================================");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  readSerialCommands();

  updateSafety();

  updateMotorState();

  updateWarningDevices();

  printStatus();

  delay(20);
}
