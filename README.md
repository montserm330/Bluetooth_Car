#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

SoftwareSerial btSerial(2, 3);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servo1Channel = 0;
const int servo2Channel = 1;
const int servo3Channel = 2;

const int SERVOMIN = 125;
const int SERVOMAX = 575;

#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8

int targetAngle1 = 0;
int targetAngle2 = 0;
int targetAngle3 = 0;

bool servo3Toggle = false;

void setup() {
  Serial.begin(115200);
  btSerial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("🔌 Ready");
}

void loop() {
  if (btSerial.available()) {
    String data = btSerial.readStringUntil('\n');
    data.trim();
    Serial.print("Received: ");
    Serial.println(data);
    processReceivedData(data);
  }

  moveServoDirect(servo1Channel, targetAngle1);
  moveServoDirect(servo2Channel, targetAngle2);
  moveServoDirect(servo3Channel, targetAngle3);
}

void processReceivedData(String data) {
  if (data.startsWith("M1")) {
    targetAngle1 = constrain(data.substring(2).toInt(), 0, 180);
  } else if (data.startsWith("M2")) {
    targetAngle2 = constrain(data.substring(2).toInt(), 0, 180);
  } else if (data == "M3") {
    // زرار فقط، مش سلايدر
    servo3Toggle = !servo3Toggle;
    targetAngle3 = servo3Toggle ? 90 : 0;
  } else if (data == "F") {
    moveForward();
  } else if (data == "B") {
    moveBackward();
  } else if (data == "L") {
    turnLeft();
  } else if (data == "R") {
    turnRight();
  } else if (data == "S") {
    stopMotors();
  }
}

void moveServoDirect(int channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
