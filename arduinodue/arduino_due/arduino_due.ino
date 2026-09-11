/*
  PROJECT 7 - PEDESTRIAN DETECTION VEHICLE
  Arduino Due Safety / Motor Controller

  Hardware:
    Arduino Due
    L298N motor driver
    4 x 6V DC gear motors
    HC-SR04 ultrasonic sensor
    Buzzer
    Red LED
    Raspberry Pi communication over USB Serial

  IMPORTANT:
    Arduino Due GPIO = 3.3V ONLY.
    HC-SR04 ECHO MUST NOT be connected directly to the Due.
    Use a resistor voltage divider / level shifter on ECHO.

  Pi commands:

    HB
    ZONE,FAR,2.50,0.95
    ZONE,CAUTION,1.50,0.90
    ZONE,DANGER,0.80,0.95
    CLEAR
    RESET

  Safety:
    - Heartbeat missing >300 ms -> STOP
    - Ultrasonic <100 cm -> STOP
    - Camera DANGER -> STOP
    - STOP is latched
    - Resume only after CLEAR for 2 seconds
*/

#include <Arduino.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// ---------- L298N ----------
// Left motor pair
const int ENA  = 5;     // PWM
const int IN1  = 22;
const int IN2  = 23;

// Right motor pair
const int ENB  = 6;     // PWM
const int IN3  = 24;
const int IN4  = 25;

// ---------- HC-SR04 ----------
const int TRIG_PIN = 30;
const int ECHO_PIN = 31;

// ---------- Warning devices ----------
const int BUZZER_PIN = 32;
const int LED_PIN    = 33;

// ---------- Optional encoder sensors ----------
// One sensor for each side for now.
// We can add the other two later.
const int LEFT_ENCODER_PIN  = 18;
const int RIGHT_ENCODER_PIN = 19;


// ============================================================
// SETTINGS
// ============================================================

const int FAR_SPEED     = 150;  // 0-255
const int CAUTION_SPEED = 60;   // ~40%

const float DANGER_DISTANCE_CM = 100.0;

// Safety heartbeat
const unsigned long HEARTBEAT_TIMEOUT_MS = 300;

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

void processCommand(String command)
{
  command.trim();

  if (command.length() == 0)
    return;


  // ----------------------------------------------------------
  // HEARTBEAT
  // ----------------------------------------------------------

  if (command == "HB")
  {
    lastHeartbeat = millis();
    heartbeatDanger = false;

    Serial.println("HB_OK");

    return;
  }


  // ----------------------------------------------------------
  // RESET
  // ----------------------------------------------------------

  if (command == "RESET")
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

  if (command == "CLEAR")
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

  if (command.startsWith("ZONE,"))
  {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    int thirdComma = command.indexOf(',', secondComma + 1);

    if (firstComma < 0 ||
        secondComma < 0 ||
        thirdComma < 0)
    {
      Serial.println("BAD_ZONE_COMMAND");
      return;
    }

    String zoneText =
        command.substring(firstComma + 1, secondComma);

    String distanceText =
        command.substring(secondComma + 1, thirdComma);

    String confidenceText =
        command.substring(thirdComma + 1);

    cameraDistance = distanceText.toFloat();
    cameraConfidence = confidenceText.toFloat();


    if (zoneText == "FAR")
    {
      currentZone = ZONE_FAR;
      cameraClear = true;
    }
    else if (zoneText == "CAUTION")
    {
      currentZone = ZONE_CAUTION;
      cameraClear = true;
    }
    else if (zoneText == "DANGER")
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

String serialBuffer = "";

void readSerialCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (serialBuffer.length() > 0)
      {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    }
    else
    {
      serialBuffer += c;

      // Prevent runaway buffer
      if (serialBuffer.length() > 100)
      {
        serialBuffer = "";
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
  Serial.println("PROJECT 7 DUE CONTROLLER");
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
