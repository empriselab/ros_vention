#define SERIAL_RX_BUFFER_SIZE 256
#include <Arduino.h>
#include <Basicmicro.h>
#include <SoftwareSerial.h>

static const uint32_t PC_BAUD = 115200;

static const uint32_t CONTROLLER_BAUD = 9600;
static const uint32_t TIMEOUT_US = 10000;

static const uint8_t ADDR_A = 128;
static const uint8_t ADDR_B = 128;

// Fresh-command watchdog:
// if no valid A=... B=... line arrives within this many ms,
// force commanded speeds back to zero.
static const uint32_t CMD_STALE_MS = 5000;

// Controller A: TX=11, RX=10
static const uint8_t RX_A = 10, TX_A = 11;
SoftwareSerial serialA(RX_A, TX_A);
Basicmicro ctrlA(&serialA, TIMEOUT_US);

// Controller B: TX=9, RX=8
static const uint8_t RX_B = 8, TX_B = 9;
SoftwareSerial serialB(RX_B, TX_B);
Basicmicro ctrlB(&serialB, TIMEOUT_US);

static int32_t speedA = 0, speedB = 0;
static int32_t lastA = INT32_MIN, lastB = INT32_MIN;
static uint32_t lastValidCmdMs = 0;

bool sendSpeedToController(Basicmicro &ctrl, SoftwareSerial &ss, uint8_t addr, int32_t speed) {
  for (int attempt = 0; attempt < 3; attempt++) {
    ss.listen();
    if (!ctrl.SpeedM1(addr, speed)) {
      Serial.print("WARN SpeedM1 failed addr="); Serial.print(addr);
      Serial.print(" speed="); Serial.print(speed);
      Serial.print(" attempt="); Serial.println(attempt);
      delay(5);
      continue;
    }
    delay(5);
    ss.listen();
    if (!ctrl.SpeedM2(addr, speed)) {
      Serial.print("WARN SpeedM2 failed addr="); Serial.print(addr);
      Serial.print(" speed="); Serial.print(speed);
      Serial.print(" attempt="); Serial.println(attempt);
      delay(5);
      continue;
    }
    return true;
  }
  Serial.print("ERROR send failed addr="); Serial.print(addr);
  Serial.print(" speed="); Serial.println(speed);
  return false;
}

bool parseLineAB(const char *line, int32_t &outA, int32_t &outB) {
  long a, b;
  char extra;
  if (sscanf(line, "A=%ld B=%ld %c", &a, &b, &extra) == 2) {
    outA = (int32_t)a;
    outB = (int32_t)b;
    return true;
  }
  else {
    Serial.print("WARN unparseable line: '"); Serial.print(line); Serial.println("'");
  }
  return false;
}

// Read one full line from Serial
bool readLine(char *buf, size_t buflen) {
  static size_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;
      return true;
    }
    if (idx < buflen - 1) buf[idx++] = c;
    else idx = 0; // overflow reset
  }
  return false;
}

void setup() {
  Serial.begin(PC_BAUD);
  while (!Serial && millis() < 3000) {}

  serialA.begin(CONTROLLER_BAUD);
  serialB.begin(CONTROLLER_BAUD);
  ctrlA.begin(CONTROLLER_BAUD);
  ctrlB.begin(CONTROLLER_BAUD);

  delay(200);

  // stop both
  sendSpeedToController(ctrlA, serialA, ADDR_A, 0);
  delay(10);
  sendSpeedToController(ctrlB, serialB, ADDR_B, 0);

  lastValidCmdMs = millis();

  Serial.println("Ready v6. Send: A=<int> B=<int>");
}

void loop() {
  // Drain ALL pending lines and keep only the latest valid one.
  char line[64];
  int32_t newA = speedA, newB = speedB;
  bool got = false;

  while (readLine(line, sizeof(line))) {
    if (parseLineAB(line, newA, newB)) {
      got = true;
    }
  }

  if (got) {
    speedA = newA;
    speedB = newB;
    lastValidCmdMs = millis();

    Serial.print("Parsed A="); Serial.print(speedA);
    Serial.print(" B="); Serial.println(speedB);
  }

  // Fresh-command watchdog:
  // if command stream goes stale, force zero velocities.
  if ((uint32_t)(millis() - lastValidCmdMs) > CMD_STALE_MS) {
    speedA = 0;
    speedB = 0;
  }

  // Only send when changed; only update last if send succeeded
  // (so failed sends are retried).
  if (speedA != lastA) {
    if (sendSpeedToController(ctrlA, serialA, ADDR_A, speedA)) {
      lastA = speedA;
    }
  }
  if (speedB != lastB) {
    delay(15); // spacing helps SoftwareSerial
    if (sendSpeedToController(ctrlB, serialB, ADDR_B, speedB)) {
      lastB = speedB;
    }
  }

  delay(2);
}