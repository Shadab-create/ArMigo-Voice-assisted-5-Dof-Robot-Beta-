#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

#define SERVOMIN 500
#define SERVOMAX 2500

int servoChannels[6] = {0, 1, 2, 3, 4, 5};
int currentPos[6]    = {90, 90, 90, 90, 90, 0};
int targetPos[6]     = {90, 90, 90, 90, 90, 0};

const int RAMP_STEP  = 3;   // ★ increased from 1
const int RAMP_DELAY = 8;   // ★ decreased from 15
unsigned long lastUpdate = 0;

int angleToPWM(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void updateServos() {
  unsigned long now = millis();
  if (now - lastUpdate < RAMP_DELAY) return;
  lastUpdate = now;

  for (int i = 0; i < 6; i++) {
    int maxAngle = (i == 5) ? 60 : 180;
    int t = constrain(targetPos[i], 0, maxAngle);

    if (abs(currentPos[i] - t) > RAMP_STEP) {
      if (t > currentPos[i]) currentPos[i] += RAMP_STEP;
      else                   currentPos[i] -= RAMP_STEP;
    } else {
      currentPos[i] = t;
    }

    pwm.writeMicroseconds(servoChannels[i], angleToPWM(currentPos[i]));
  }
}

void readSerial() {
  static String input = "";

  // ★ Stale buffer guard — if too long, something went wrong, flush it
  if (input.length() > 40) {
    input = "";
    while (Serial.available()) Serial.read();
    return;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int idx = 0;
      char buf[50];
      input.toCharArray(buf, sizeof(buf));
      char* token = strtok(buf, ",");

      while (token != NULL && idx < 6) {
        int maxAngle = (idx == 5) ? 60 : 180;
        targetPos[idx] = constrain(atoi(token), 0, maxAngle);
        idx++;
        token = strtok(NULL, ",");
      }

      if (idx == 6) {
        Serial.println("OK");
      }

      input = "";
    } else {
      input += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);

  for (int i = 0; i < 6; i++) {
    pwm.writeMicroseconds(servoChannels[i], angleToPWM(currentPos[i]));
  }

  Serial.println("ESP32 Ready");
}

void loop() {
  readSerial();
  updateServos();
}