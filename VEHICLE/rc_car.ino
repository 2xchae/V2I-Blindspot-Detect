#include <Wire.h>
#include <Adafruit_TCS34725.h>

// 1. RGB 센서 설정
Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// 2. 핀 설정
// 왼쪽 앞
const int LF_DIR = 8;
const int LF_PWM = 11;

// 오른쪽 앞
const int RF_DIR = 7;
const int RF_PWM = 6;

// 왼쪽 뒤
const int LR_DIR = 4;
const int LR_PWM = 5;

// 오른쪽 뒤
const int RR_DIR = 2;
const int RR_PWM = 3;

// 초음파 센서 핀
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;


// 3. 모터 전진 방향 설정
const int LF_FORWARD = HIGH;
const int RF_FORWARD = LOW;
const int LR_FORWARD = HIGH;
const int RR_FORWARD = LOW;


// 4. 주행 속도 설정
const int SPEED_NORMAL = 200;
const int SPEED_SLOW = 160;
const int SPEED_STOP = 0;


// 5. RGB 판정 임계값
const float RED_RATIO_THRESHOLD = 0.50;
const float RED_GREEN_THRESHOLD = 1.80;
const float RED_BLUE_THRESHOLD = 2.20;

const int MIN_CLEAR = 30;
const int RED_CONFIRM_COUNT = 2;

const unsigned long RGB_INTERVAL = 100;

// 빨간색이 이 시간 동안 감지되지 않으면 보호구역 해제
// 서행 속도로 횡단보도를 지나가는 시간보다 길게 설정
const unsigned long RED_LOST_TIMEOUT = 1500;

// 5-1. 초음파 장애물 판정
const float OBSTACLE_STOP_CM = 5.0;     // 이 거리 이하면 정지
const float OBSTACLE_RELEASE_CM = 8.0;  // 이 거리 이상이면 정지 해제
const int OBSTACLE_CONFIRM_COUNT = 2;   // 연속 판정 횟수

const unsigned long ULTRA_INTERVAL = 60;     // 측정 주기 (ms)
const unsigned long ECHO_TIMEOUT_US = 6000;  // 약 1m 이상은 측정하지 않음

// 6. 통신 설정
const unsigned long TX_INTERVAL = 500;

// 단말기 통신이 일정 시간 끊기면 정지
const unsigned long RX_TIMEOUT = 2000;


// 7. 주행 상태 변수
bool schoolZone = false;
bool dangerDetected = false;
bool manualStop = false;
bool sensorReady = false;
bool terminalReady = false;

int redCount = 0;
int currentSpeed = -1;

unsigned long lastRGB = 0;
unsigned long lastTX = 0;
unsigned long lastRX = 0;

unsigned long lastRedSeen = 0;

bool obstacleDetected = false;

int obstacleCount = 0;
int clearCount = 0;

unsigned long lastUltra = 0;

// 8. 모터 제어 함수
void driveMotor(int pwmPin, int dirPin, int forwardLevel, int speed) {

  if (speed == 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  digitalWrite(dirPin, forwardLevel);

  if (forwardLevel == HIGH) {
    analogWrite(pwmPin, 255 - speed);
  } else {
    analogWrite(pwmPin, speed);
  }
}

void setMotorSpeed(int speed) {

  speed = constrain(speed, 0, 255);

  if (currentSpeed == speed) {
    return;
  }

  currentSpeed = speed;

  driveMotor(LF_PWM, LF_DIR, LF_FORWARD, speed);
  driveMotor(RF_PWM, RF_DIR, RF_FORWARD, speed);
  driveMotor(LR_PWM, LR_DIR, LR_FORWARD, speed);
  driveMotor(RR_PWM, RR_DIR, RR_FORWARD, speed);
}

void setupMotors() {

  pinMode(LF_PWM, OUTPUT);
  pinMode(RF_PWM, OUTPUT);
  pinMode(LR_PWM, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  pinMode(LF_DIR, OUTPUT);
  pinMode(RF_DIR, OUTPUT);
  pinMode(LR_DIR, OUTPUT);
  pinMode(RR_DIR, OUTPUT);

  setMotorSpeed(SPEED_STOP);
}


// 9. 최종 주행 상태 결정
// 우선순위: 정지 > 감속 > 정상
void updateDriving() {

  bool communicationLost =
    !terminalReady ||
    (millis() - lastRX >= RX_TIMEOUT);

  if (!sensorReady ||
      manualStop ||
      dangerDetected ||
      obstacleDetected ||
      communicationLost) {

    setMotorSpeed(SPEED_STOP);
  }

  else if (schoolZone) {

    setMotorSpeed(SPEED_SLOW);
  }

  else {

    setMotorSpeed(SPEED_NORMAL);
  }
}


// 10. 단말기로 보호구역 상태 전송
// N = 일반 도로
// Z = 어린이 보호구역
void sendZoneState() {

  if (schoolZone) {
    Serial.write('Z');
  }

  else {
    Serial.write('N');
  }

  lastTX = millis();
}


// 10-1. 초음파 거리 측정 및 장애물 판정
float measureDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  // 시간 초과: 측정 범위 안에 물체 없음
  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2.0;
}

void updateUltrasonic() {

  if (millis() - lastUltra < ULTRA_INTERVAL) {
    return;
  }

  lastUltra = millis();

  float distance = measureDistance();

  // 정지 전: 5cm 이내가 연속으로 측정되면 정지
  if (!obstacleDetected) {

    if (distance > 0 && distance <= OBSTACLE_STOP_CM) {

      obstacleCount++;

      if (obstacleCount >= OBSTACLE_CONFIRM_COUNT) {

        obstacleDetected = true;
        obstacleCount = 0;
        clearCount = 0;

        // 즉시 정지
        updateDriving();
      }
    }

    else {

      obstacleCount = 0;
    }
  }

  // 정지 후: 8cm 이상 또는 물체 없음이 연속으로 측정되면 해제
  else {

    if (distance < 0 || distance >= OBSTACLE_RELEASE_CM) {

      clearCount++;

      if (clearCount >= OBSTACLE_CONFIRM_COUNT) {

        obstacleDetected = false;
        clearCount = 0;
      }
    }

    else {

      clearCount = 0;
    }
  }
}


// 11. RGB 센서 측정 및 보호구역 판정
void updateRGB() {

  if (millis() - lastRGB < RGB_INTERVAL) {
    return;
  }

  lastRGB = millis();

  uint16_t r, g, b, c;

  tcs.getRawData(&r, &g, &b, &c);

  float total = (float)r + g + b;

  bool isRed = false;

  // 유효한 센서값일 때만 빨간색 판정
  if (total > 0 &&
      g > 0 &&
      b > 0 &&
      c >= MIN_CLEAR) {

    float rRatio = (float)r / total;
    float rgRatio = (float)r / g;
    float rbRatio = (float)r / b;

    isRed = (
      rRatio >= RED_RATIO_THRESHOLD &&
      rgRatio >= RED_GREEN_THRESHOLD &&
      rbRatio >= RED_BLUE_THRESHOLD
    );
  }

  // 보호구역 진입 전
  if (!schoolZone) {

    if (isRed) {

      redCount++;

      if (redCount >= RED_CONFIRM_COUNT) {

        schoolZone = true;
        redCount = 0;
        lastRedSeen = millis();

        // 1. 즉시 감속
        updateDriving();

        // 2. 단말기로 진입 신호 전송
        sendZoneState();
      }
    }

    else {

      redCount = 0;
    }
  }

  // 보호구역 진입 후
  else {

    if (isRed) {

      lastRedSeen = millis();
    }

    else if (millis() - lastRedSeen >= RED_LOST_TIMEOUT) {

      schoolZone = false;

      // 1. 정상 속도로 복귀
      updateDriving();

      // 2. 단말기로 해제 신호 전송
      sendZoneState();
    }
  }
}

// 12. 단말기 통신 수신
// D = 위험 상황 -> 정지
// N = 위험 없음/해제 -> 주행
// X = 수동 정지
// G = 수동 정지 해제
void receiveTerminal() {

  while (Serial.available() > 0) {

    char command = Serial.read();

    switch (command) {

      case 'D':

        lastRX = millis();
        terminalReady = true;
        dangerDetected = true;

        // 위험 신호 수신 즉시 정지
        updateDriving();

        break;

      case 'N':

        lastRX = millis();
        terminalReady = true;
        dangerDetected = false;

        // 보호구역이면 감속 주행
        // 일반 도로이면 정상 주행
        updateDriving();

        break;

      case 'X':

        manualStop = true;
        updateDriving();

        break;

      case 'G':

        manualStop = false;
        updateDriving();

        break;

      default:

        break;
    }
  }
}

// 13. 초기 설정
void setup() {

  // 단말기와 UART 통신
  Serial.begin(9600);

  // 초음파 센서 초기 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // 모터 초기 설정
  setupMotors();

  // RGB 센서 초기 설정
  Wire.begin();

  if (!tcs.begin()) {

    sensorReady = false;

    setMotorSpeed(SPEED_STOP);

    // 센서 연결 실패 시 주행 금지
    while (true) {
      delay(100);
    }
  }

  sensorReady = true;

  // 초기 보호구역 상태 전송
  sendZoneState();

  // 단말기 정상 신호 수신 전까지 정지
  updateDriving();
}

// 14. 메인 루프
void loop() {

  // 1. 위험 신호 우선 확인
  receiveTerminal();

  // 2. 초음파 장애물 확인
  updateUltrasonic();

  // 3. RGB 센서 측정
  updateRGB();

  // 4. 측정 중 도착한 위험 신호 확인
  receiveTerminal();

  // 5. 최종 주행 속도 적용
  updateDriving();

  // 6. 단말기로 현재 상태 주기적 전송
  if (millis() - lastTX >= TX_INTERVAL) {

    sendZoneState();
  }
}