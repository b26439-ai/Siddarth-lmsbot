// Task 9 - ESP32 line follower with PID control (Appendix A hardware)
// Board: "ESP32 Dev Module" in the Arduino IDE. Serial Monitor at 115200 baud.

// ---------------- Pins (Appendix A) ----------------
const int SENSOR_PINS[5] = {34, 35, 32, 33, 25};   // S1 (leftmost) ... S5 (rightmost)

const int ENA = 14;   // left motor speed (PWM)
const int IN1 = 27;   // left motor direction A
const int IN2 = 26;   // left motor direction B
const int ENB = 13;   // right motor speed (PWM)
const int IN3 = 12;   // right motor direction A
const int IN4 = 4;    // right motor direction B
// Appendix A lists IN4 on GPIO 11, but GPIO 6-11 are used by the ESP32's flash
// memory and cannot be used, so IN4 is wired to GPIO 4 instead.

// ---------------- Settings ----------------
const float KP = 50;         // proportional gain
const float KI = 0;          // integral gain (kept at 0, see notes)
const float KD = 60;         // derivative gain
const int BASE_SPEED = 150;  // PWM when the line is centred (0-255)
const int MAX_SPEED = 255;   // largest PWM value
const int SEARCH_SPEED = 120;      // turning speed while looking for a lost line
const int LOST_TIMEOUT_MS = 300;   // stop if the line isn't found in this time

const bool TEST_MODE = true;  // true = run sample inputs only, false = drive the robot

// ---------------- Variables ----------------
int sensorMin[5];      // darkest/lightest readings seen during calibration
int sensorMax[5];
float lastError = 0;   // for the D term
float integral = 0;    // for the I term
float lastPosition = 0;          // where the line was last seen (for recovery)
bool lineLost = false;           // true while the line is lost
unsigned long lineLostTime = 0;  // when the line was lost


// 1. Read the five sensors (0-4095 each)
void readSensors(int raw[5]) {
  for (int i = 0; i < 5; i++) {
    raw[i] = analogRead(SENSOR_PINS[i]);
  }
}

// 2. Calibrate: move the robot across the line for 3 s and record min and max
void calibrateSensors() {
  for (int i = 0; i < 5; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
  Serial.println("Calibrating: move the sensors over the line for 3 seconds");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    int raw[5];
    readSensors(raw);
    for (int i = 0; i < 5; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    delay(5);
  }
  Serial.println("Calibration done");
}

// 3. Scale each reading to 0-1000 using the calibration (1000 = on the line)
//    Appendix A: a darker surface (the line) gives a higher voltage.
void scaleSensors(int raw[5], float scaled[5]) {
  for (int i = 0; i < 5; i++) {
    int range = sensorMax[i] - sensorMin[i];
    if (range <= 0) {
      scaled[i] = 0;            // this sensor never changed: ignore it
      continue;
    }
    scaled[i] = 1000.0 * (raw[i] - sensorMin[i]) / range;
    if (scaled[i] < 0) scaled[i] = 0;
    if (scaled[i] > 1000) scaled[i] = 1000;
  }
}

// 4. Weighted centroid: line position from -2 (far left) to +2 (far right)
//    Returns false in lineFound if no sensor sees the line.
float computePosition(float s[5], bool &lineFound) {
  float weights[5] = {-2, -1, 0, 1, 2};
  float weightedSum = 0;
  float total = 0;
  for (int i = 0; i < 5; i++) {
    weightedSum += weights[i] * s[i];
    total += s[i];
  }
  if (total < 200) {            // (almost) all zero: no line under the sensors
    lineFound = false;
    return lastPosition;        // never divide by zero
  }
  lineFound = true;
  return weightedSum / total;
}

// 5. PID: turn the error into a steering correction
float computePID(float error) {
  integral += error;
  float derivative = error - lastError;
  lastError = error;
  return KP * error + KI * integral + KD * derivative;
}

// 6. Limit both motor commands to MAX_SPEED proportionally: if either is too big,
//    scale BOTH down by the same factor, so the ratio between them (the turn) is kept.
void clampSpeed(float &left, float &right) {
  float peak = max(abs(left), abs(right));    // the bigger of the two sizes
  if (peak > MAX_SPEED) {
    float scale = MAX_SPEED / peak;            // e.g. 255 / 260 = 0.98
    left = left * scale;
    right = right * scale;
  }
}

// 7. Drive one motor through the L298N: sign = direction, size = speed
void setMotor(int enPin, int inA, int inB, int speed) {
  if (speed >= 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }
  analogWrite(enPin, abs(speed));
}

void driveMotors(int left, int right) {
  setMotor(ENA, IN1, IN2, left);
  setMotor(ENB, IN3, IN4, right);
}

// 8. Line lost: turn toward where the line was last seen; stop if it takes too long
void handleLostLine(int &left, int &right) {
  if (!lineLost) {                                   // just lost it: note the time
    lineLost = true;
    lineLostTime = millis();
  }
  if (millis() - lineLostTime > LOST_TIMEOUT_MS) {
    left = 0;                                         // safe stop
    right = 0;
  } else if (lastPosition >= 0) {
    left = SEARCH_SPEED;                              // line was on the right: spin right
    right = -SEARCH_SPEED;
  } else {
    left = -SEARCH_SPEED;                             // line was on the left: spin left
    right = SEARCH_SPEED;
  }
}

// 9. One complete control step: sensor values in, motor speeds out
void controlStep(float s[5], int &left, int &right) {
  bool lineFound;
  float position = computePosition(s, lineFound);

  if (!lineFound) {
    handleLostLine(left, right);
  } else {
    if (lineLost) {             // line found again: restart PID memory
      lastError = position;
      integral = 0;
      lineLost = false;
    }
    lastPosition = position;
    float error = position;     // target is 0 (line in the centre)
    float correction = computePID(error);
    float leftCmd = BASE_SPEED + correction;
    float rightCmd = BASE_SPEED - correction;
    clampSpeed(leftCmd, rightCmd);
    left = round(leftCmd);
    right = round(rightCmd);
  }

  Serial.printf("%.0f,%.0f,%.0f,%.0f,%.0f, pos=%.3f, L=%d, R=%d%s\n",
                s[0], s[1], s[2], s[3], s[4], position, left, right,
                lineFound ? "" : "  (line lost)");
}

// 10. Validation with hard-coded sensor values (no robot needed)
void runTests() {
  float tests[6][5] = {
    {0, 100, 1000, 100, 0},   // centred          -> L 150, R 150
    {0, 0, 200, 1000, 200},   // line to the right -> L 255, R 39
    {0, 0, 0, 500, 1000},     // further right     -> L 255, R 25
    {0, 0, 300, 1000, 100},   // coming back       -> L 144, R 156
    {0, 0, 0, 0, 0},          // line lost         -> L 120, R -120 (spin right)
    {300, 1000, 200, 0, 0},   // found on the left -> L 97, R 203
  };
  Serial.println("Test: s1,s2,s3,s4,s5, position, left, right");
  for (int t = 0; t < 6; t++) {
    int left, right;
    controlStep(tests[t], left, right);
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  driveMotors(0, 0);

  if (TEST_MODE) {
    runTests();
  } else {
    calibrateSensors();
  }
}

void loop() {
  if (TEST_MODE) return;       // tests already ran in setup()

  int raw[5];
  float scaled[5];
  int left, right;

  readSensors(raw);
  scaleSensors(raw, scaled);
  controlStep(scaled, left, right);
  driveMotors(left, right);
  delay(10);                   // fixed 10 ms loop, so the D term is consistent
}
