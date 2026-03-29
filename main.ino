//lfrv4_bt
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <EEPROM.h>  // ← Added for permanent storage

// ======================
// SENSOR SETUP
// ======================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int BLACK_THRESHOLD = 850;

// ======================
// MOTOR PINS (UNCHANGED)
// ======================
#define AIN1 7
#define BIN1 5
#define AIN2 8
#define BIN2 4
#define PWMA 9
#define PWMB 3
#define STBY 6

//=====String Buffer=====
String btBuffer = "";

const int offsetA = 1;
const int offsetB = 1;

Motor motor2 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// ======================
// BUTTON
// ======================
const int buttonPin = 10;
bool running = false;
bool lastButtonState = HIGH;

// ======================
// SPEED SETTINGS
// ======================
int baseSpeedA = 175;
int baseSpeedB = 175;
int maxSpeedA = 255;
int maxSpeedB = 255;

int TURN_SPEED = 125;

// ======================
// EEPROM ADDRESSES
// Each double/float = 4 bytes, so space them 4 apart
// ======================
#define EEPROM_MAGIC_ADDR  0   // 1 byte  — magic number to detect valid save
#define EEPROM_KP_ADDR     1   // 4 bytes
#define EEPROM_KI_ADDR     5   // 4 bytes
#define EEPROM_KD_ADDR     9   // 4 bytes
#define EEPROM_MAGIC_VAL   0xAB  // arbitrary magic value

// ======================
// FAST INTEGER PID
// ======================
double Kp = 85;
float  Ki = 0;
double Kd = 17;

int error = 0;
int lastError = 0;
long integral = 0;

// ======================
// EEPROM HELPERS
// ======================

// Save all three PID values to EEPROM
void savePIDtoEEPROM() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
  EEPROM.put(EEPROM_KP_ADDR, Kp);
  EEPROM.put(EEPROM_KI_ADDR, Ki);
  EEPROM.put(EEPROM_KD_ADDR, Kd);
}

// Load PID from EEPROM — only if a valid save exists
void loadPIDfromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VAL) {
    EEPROM.get(EEPROM_KP_ADDR, Kp);
    EEPROM.get(EEPROM_KI_ADDR, Ki);
    EEPROM.get(EEPROM_KD_ADDR, Kd);
    Serial.println("PID loaded from EEPROM:");
  } else {
    Serial.println("No saved PID found. Using defaults:");
  }
  Serial.print("  Kp="); Serial.println(Kp);
  Serial.print("  Ki="); Serial.println(Ki);
  Serial.print("  Kd="); Serial.println(Kd);
}

// ======================
// TURN STATE MACHINE
// ======================
enum TurnState { NONE, LEFT, RIGHT};
TurnState turning = NONE;
TurnState lastSeenTurn = NONE;

// ======================
// SETUP
// ======================
void setup() {
  Serial.begin(9600);

  // ← Load saved PID values from EEPROM on every boot
  loadPIDfromEEPROM();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // wait for button press before calibrating
  while(digitalRead(buttonPin) == HIGH){};
  delay(200);

  // Fast calibration
  for (uint16_t i = 0; i < 60; i++) {
    motor1.drive(60);
    motor2.drive(-60);
    qtr.calibrate();
    delay(10);
  }

  for (uint16_t i = 0; i < 60; i++) {
    motor1.drive(-60);
    motor2.drive(60);
    qtr.calibrate();
    delay(10);
  }

  brake(motor1, motor2);
}

// ======================
// MAIN LOOP
// ======================
void loop() {
  readBluetoothPID();
  handleButton();

  if (running) {
    PID_control();
  } else {
    brake(motor1, motor2);
  }
}

// ======================
// BUTTON TOGGLE (FAST)
// ======================
void handleButton() {
  bool currentState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && currentState == LOW) {
    running = !running;

    if (running) {
      integral = 0;
      lastError = 0;
    }
  }

  lastButtonState = currentState;
}

// ======================
// MOTOR DRIVE
// ======================
inline void driveMotors(int left, int right) {
  motor1.drive(left);
  motor2.drive(right);
}

// ======================
// PID + TURN LOGIC
// ======================
void PID_control() {

  uint16_t position = qtr.readLineBlack(sensorValues);

  // ---------- PID ----------
  error = (int)position - 3500;

  integral += error;
  if (integral > 3000) integral = 3000;
  if (integral < -3000) integral = -3000;

  int derivative = error - lastError;
  lastError = error;

  float correction =
      (Kp * error +
       Ki * integral +
       Kd * derivative);

  int leftSpeed  = baseSpeedA + correction;
  int rightSpeed = baseSpeedB - correction;

  leftSpeed  = constrain(leftSpeed, 0, maxSpeedA);
  rightSpeed = constrain(rightSpeed, 0, maxSpeedB);

  // ---------- SENSOR LOGIC ----------
  bool lineCentered =
      sensorValues[2] > BLACK_THRESHOLD ||
      sensorValues[3] > BLACK_THRESHOLD ||
      sensorValues[4] > BLACK_THRESHOLD ||
      sensorValues[5] > BLACK_THRESHOLD;

  bool rightTurn =
      sensorValues[0] > BLACK_THRESHOLD ||
      sensorValues[1] > BLACK_THRESHOLD;

  bool leftTurn =
      sensorValues[6] > BLACK_THRESHOLD ||
      sensorValues[7] > BLACK_THRESHOLD;

  bool allWhite = true;
  for (uint8_t i=0; i<SensorCount; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) {
      allWhite = false;
      break;
    }
  }

  // ---------- TURN EXECUTION ----------
  if (turning != NONE) {
    if (lineCentered) {
      turning = NONE;
    } else {
      if (turning == LEFT)
        driveMotors(-(TURN_SPEED/2), TURN_SPEED);
      else
        driveMotors(TURN_SPEED, -(TURN_SPEED/2));
      return;
    }
  }

  // ---------- TURN PRIORITY (LEFT) ----------
  if (leftTurn && rightTurn) {
    // straight through intersection — do nothing
  } else if (leftTurn) {
    turning = LEFT;
    lastSeenTurn = LEFT;
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (rightTurn) {
    turning = RIGHT;
    lastSeenTurn = RIGHT;
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (allWhite && lastSeenTurn != NONE) {
    turning = lastSeenTurn;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    driveMotors(leftSpeed, rightSpeed);
  }
}

// ======================
// PID Calibration via BT
// Values are saved to EEPROM so they survive power off
// Supported commands (case-insensitive):
//   kp=<value>
//   ki=<value>
//   kd=<value>
//   pid?          ← prints current values
// ======================
void readBluetoothPID() {

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      btBuffer.trim();
      btBuffer.toLowerCase();

      if (btBuffer.length() == 0) {
        btBuffer = "";
        return;
      }

      // ----- QUERY current values -----
      if (btBuffer == "pid?") {
        Serial.println("--- Current PID Values ---");
        Serial.print("Kp = "); Serial.println(Kp, 4);
        Serial.print("Ki = "); Serial.println(Ki, 4);
        Serial.print("Kd = "); Serial.println(Kd, 4);
      }

      // ----- KP -----
      else if (btBuffer.startsWith("kp")) {
        int eqIndex = btBuffer.indexOf('=');
        if (eqIndex != -1) {
          Kp = btBuffer.substring(eqIndex + 1).toFloat();
          savePIDtoEEPROM();  // ← persist to EEPROM
          Serial.print("OK: Kp saved = ");
          Serial.println(Kp, 4);
        } else {
          Serial.println("ERR: use kp=<value>");
        }
      }

      // ----- KI -----
      else if (btBuffer.startsWith("ki")) {
        int eqIndex = btBuffer.indexOf('=');
        if (eqIndex != -1) {
          Ki = btBuffer.substring(eqIndex + 1).toFloat();
          savePIDtoEEPROM();  // ← persist to EEPROM
          Serial.print("OK: Ki saved = ");
          Serial.println(Ki, 4);
        } else {
          Serial.println("ERR: use ki=<value>");
        }
      }

      // ----- KD -----
      else if (btBuffer.startsWith("kd")) {
        int eqIndex = btBuffer.indexOf('=');
        if (eqIndex != -1) {
          Kd = btBuffer.substring(eqIndex + 1).toFloat();
          savePIDtoEEPROM();  // ← persist to EEPROM
          Serial.print("OK: Kd saved = ");
          Serial.println(Kd, 4);
        } else {
          Serial.println("ERR: use kd=<value>");
        }
      }

      else {
        Serial.println("ERR: Unknown command. Try kp=, ki=, kd=, or pid?");
      }

      btBuffer = "";  // clear buffer
    }
    else {
      btBuffer += c;
    }
  }
}
