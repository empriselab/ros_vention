#include <Arduino.h>
#include <Basicmicro.h>
#include <SoftwareSerial.h>

static const uint32_t PC_BAUD = 115200;

static const uint32_t CONTROLLER_BAUD = 9600;
static const uint32_t TIMEOUT_US = 10000;

static const uint8_t ADDR_A = 128;
static const uint8_t ADDR_B = 128;

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

void sendSpeedToController(Basicmicro &ctrl, SoftwareSerial &ss, uint8_t addr, int32_t speed) {
  ss.listen();
  ctrl.SpeedM1(addr, (uint32_t)speed);
  delay(5);
  ss.listen();
  ctrl.SpeedM2(addr, (uint32_t)speed);
}

// Parse line like: "A=-300 B=200"
bool parseLineAB(const char *line, int32_t &outA, int32_t &outB) {
  // defaults: keep last if missing
  bool gotA = false, gotB = false;

  const char *p = line;
  while (*p) {
    while (*p == ' ') p++;

    if ((p[0] == 'A' || p[0] == 'a') && p[1] == '=') {
      p += 2;
      outA = (int32_t)strtol(p, (char**)&p, 10);
      gotA = true;
    } else if ((p[0] == 'B' || p[0] == 'b') && p[1] == '=') {
      p += 2;
      outB = (int32_t)strtol(p, (char**)&p, 10);
      gotB = true;
    } else {
      // skip token
      while (*p && *p != ' ') p++;
    }
  }

  return gotA || gotB;
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

  Serial.println("Ready. Send: A=<int> B=<int>");
}

void loop() {
  char line[64];
  if (readLine(line, sizeof(line))) {
    int32_t newA = speedA, newB = speedB;
    if (parseLineAB(line, newA, newB)) {
      speedA = newA;
      speedB = newB;

      Serial.print("Parsed A="); Serial.print(speedA);
      Serial.print(" B="); Serial.println(speedB);
    }
  }

  // Only send when changed
  if (speedA != lastA) {
    sendSpeedToController(ctrlA, serialA, ADDR_A, speedA);
    lastA = speedA;
  }
  if (speedB != lastB) {
    delay(15); // spacing helps SoftwareSerial
    sendSpeedToController(ctrlB, serialB, ADDR_B, speedB);
    lastB = speedB;
  }

  delay(2);
}