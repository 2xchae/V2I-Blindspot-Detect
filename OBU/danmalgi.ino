#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 1. 하드웨어 설정
// DFPlayer
// Arduino RX = D4, TX = D5
SoftwareSerial dfSerial(4, 5);
DFRobotDFPlayerMini player;

// HC-12
// Arduino RX = D10, TX = D11
SoftwareSerial hc12(10, 11);

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// 2. 시간 및 테스트 설정
const unsigned long FLASH_INTERVAL = 300;
const unsigned long FLASH_MS = 0;

const unsigned long TX_INTERVAL = 500;
const unsigned long INFRA_TIMEOUT = 2000;

// true: 인프라 없이 RC카 + 단말기 테스트
// false: 최종 인프라 통합 데모시에 설정

const bool INFRA_TEST_MODE = true;

// 3. 상태 변수
bool schoolZone = false;
bool dangerDetected = false;
bool infraEverReceived = false;

bool displayReady = false;
bool playerReady = false;

bool inverted = false;

// 보호구역 안내 음성 재생 여부
bool zoneAudioPlayed = false;

// 위험 안내 음성 재생 여부
bool dangerAudioPlayed = false;

// -1 = 초기 상태
//  0 = 검은 화면
//  1 = 어린이 보호구역
//  2 = 위험 상황

int mode = -1;

char lastCarCommand = 0;

unsigned long lastInfraRx = 0;
unsigned long lastCarTx = 0;
unsigned long modeStart = 0;
unsigned long lastFlash = 0;

// 4. 화면 반전
void setInvert(bool on) {

  if (!displayReady) {
    return;
  }

  if (on != inverted) {

    display.invertDisplay(on);
    inverted = on;
  }
}

// 5. 가운데 정렬
void drawCenter(
  const __FlashStringHelper* s,
  int y,
  int size
) {

  int16_t x1, y1;
  uint16_t w, h;

  display.setTextSize(size);

  display.getTextBounds(
    s, 0, 0,
    &x1, &y1,
    &w, &h
  );

  display.setCursor(
    (SCREEN_WIDTH - w) / 2,
    y
  );

  display.print(s);
}

// 6. 기본 화면
// READY 제거 → 검은 화면
void drawBlank() {

  setInvert(false);

  display.clearDisplay();
  display.display();
}


// 7. 어린이 보호구역 진입 화면
void drawEnter() {

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // 바깥 삼각형
  display.fillTriangle(
    32, 1,
    1, 60,
    63, 60,
    SSD1306_WHITE
  );

  // 안쪽 삼각형
  display.fillTriangle(
    32, 12,
    10, 55,
    54, 55,
    SSD1306_BLACK
  );

  // 삼각형 안 30
  display.setTextSize(2);
  display.setCursor(20, 38);
  display.print(F("30"));

  // SCHOOL ZONE SLOW
  display.setTextSize(2);

  display.setCursor(56, 4);
  display.print(F("SCHOOL"));

  display.setCursor(56, 22);
  display.print(F("  ZONE"));

  display.setCursor(56, 40);
  display.print(F("  SLOW"));

  display.display();
}


// 8. 위험 경고 화면
void drawDanger() {

  display.clearDisplay();

  // 경고 삼각형
  display.fillTriangle(
    64, 0,
    44, 26,
    84, 26,
    SSD1306_WHITE
  );

  // 느낌표
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(58, 9);
  display.print(F("!"));

  display.setTextColor(SSD1306_WHITE);

  drawCenter(F("WATCH"), 30, 2);
  drawCenter(F("OUT!!"), 48, 2);

  display.display();
}

// 9. MP3 재생 및 정지
void playAudio(uint16_t track) {

  if (!playerReady) {
    return;
  }

  player.playMp3Folder(track);

  // HC-12 수신 우선
  hc12.listen();
}

void stopAudio() {

  if (!playerReady) {
    return;
  }

  player.stop();

  hc12.listen();
}


// 10. 인프라 통신 상태 확인
bool isInfraCommunicationLost() {

  // 테스트 모드에서는 인프라 없이 주행 가능
  if (INFRA_TEST_MODE && !infraEverReceived) {

    return false;
  }

  // 최종 모드에서는 최초 인프라 신호 필요
  if (!infraEverReceived) {

    return true;
  }

  // 통신 두절 확인
  if (millis() - lastInfraRx >= INFRA_TIMEOUT) {

    return true;
  }

  return false;
}


// 11. RC카에 주행 명령 전송
// N = 위험 없음 -> 주행 허용
// D = 위험 있음 -> 정지
void sendCarCommand(bool forceSend = false) {

  bool mustStop =
    !displayReady ||
    !playerReady ||
    dangerDetected ||
    isInfraCommunicationLost();

  char command = mustStop ? 'D' : 'N';

  bool stateChanged =
    command != lastCarCommand;

  bool intervalPassed =
    millis() - lastCarTx >= TX_INTERVAL;

  if (forceSend ||
      stateChanged ||
      intervalPassed) {

    Serial.write(command);

    lastCarCommand = command;
    lastCarTx = millis();
  }
}


// 12. 디스플레이 및 스피커 상태 변경
void setMode(int newMode) {

  // 동일한 상태 반복 처리 방지
  if (mode == newMode) {

    return;
  }

  mode = newMode;

  modeStart = millis();
  lastFlash = modeStart;

  setInvert(false);

  // 일반 도로 또는 통신 두절
  if (mode == 0) {

    drawBlank();
    stopAudio();
  }

  // 어린이 보호구역
  else if (mode == 1) {

    drawEnter();

    if (!zoneAudioPlayed) {

      playAudio(1);

      zoneAudioPlayed = true;
    }

    else {

      stopAudio();
    }
  }

  // 위험 상황
  else if (mode == 2) {

    drawDanger();

    if (!dangerAudioPlayed) {

      playAudio(2);

      dangerAudioPlayed = true;
    }
  }
}

// 13. 최종 화면 상태 결정
void updateTerminalMode() {

  // 실제 위험 신호를 수신한 경우에만 위험 화면 표시
  if (dangerDetected) {

    setMode(2);
  }

  // 통신 실패만으로 위험 화면을 표시하지 않음
  else if (isInfraCommunicationLost()) {

    setMode(0);
  }

  // RC카에서 보호구역 진입 신호를 받은 경우
  else if (schoolZone) {

    setMode(1);
  }

  // 일반 도로
  else {

    setMode(0);
  }
}


// 14. RC카에서 보호구역 상태 수신
// N = 일반 도로
// Z = 어린이 보호구역
void receiveCar() {

  while (Serial.available() > 0) {

    char command = Serial.read();

    switch (command) {

      case 'N':

        schoolZone = false;
        zoneAudioPlayed = false;

        break;

      case 'Z':

        schoolZone = true;

        break;

      default:

        break;
    }
  }
}

// 15. 인프라 HC-12 위험 상태 수신
// D = 위험 있음
// N = 위험 없음 / 위험 해제
void receiveInfrastructure() {

  while (hc12.available() > 0) {

    char command = hc12.read();

    switch (command) {

    
      // 위험 상황
      case 'D':
      case 'd':

        infraEverReceived = true;
        lastInfraRx = millis();

        dangerDetected = true;

        // RC카 정지 명령 우선으로
        sendCarCommand();

        break;

      
      // 위험 없음 또는 해제
      case 'N':
      case 'n':

        infraEverReceived = true;
        lastInfraRx = millis();

        dangerDetected = false;

        // 새로운 위험 발생 시 음성 재생 가능
        dangerAudioPlayed = false;

        sendCarCommand();

        break;

      default:

        break;
    }
  }
}


// 16. 화면 깜빡임
// 300ms 반전 효과 유지
void updateFlash() {

  if (mode != 1 && mode != 2) {

    return;
  }

  unsigned long now = millis();

  bool flashing =
    (FLASH_MS == 0) ||
    (now - modeStart < FLASH_MS);

  if (flashing) {

    if (now - lastFlash >= FLASH_INTERVAL) {

      lastFlash = now;

      setInvert(!inverted);
    }
  }

  else {

    setInvert(false);
  }
}

// 17. 초기 설정
void setup() {

  // RC카 UART 통신
  Serial.begin(9600);

  // 초기화 중에는 RC카 정지
  Serial.write('D');

  
  // OLED 초기화
  Wire.begin();

  displayReady = display.begin(
    SSD1306_SWITCHCAPVCC,
    0x3C
  );

  if (!displayReady) {

    while (true) {

      Serial.write('D');
      delay(500);
    }
  }

  display.setTextWrap(false);

  // 초기 화면은 아무것도 표시하지 않음
  drawBlank();

  // DFPlayer 초기화
  dfSerial.begin(9600);
  dfSerial.listen();

  delay(2000);

  // 최대 5회 초기화 시도

  for (int i = 0; i < 5; i++) {

    if (player.begin(dfSerial, true, true)) {

      playerReady = true;

      break;
    }

    Serial.write('D');

    delay(1000);
  }

  if (!playerReady) {

    drawBlank();

    while (true) {

      Serial.write('D');
      delay(500);
    }
  }

  // DFPlayer 응답 대기 방지
  // disableACK()가 private이므로 begin()으로 ACK를 끔
  player.begin(dfSerial, false, false);
  
  player.volume(30);

  // HC-12 초기화
  hc12.begin(9600);
  hc12.listen();

  // 초기 상태
  setMode(0);

  sendCarCommand(true);
}

// 18. 메인 루프
void loop() {

  // 1. 인프라 위험 신호 확인
  receiveInfrastructure();

  // 2. RC카 정지/주행 명령 전송
  sendCarCommand();

  // 3. RC카 보호구역 상태 확인
  receiveCar();

  // 4. 화면 및 음성 처리
  updateTerminalMode();

  // 5. 화면 반전 효과
  updateFlash();

  // 6. 화면 처리 중 들어온 위험 신호 확인
  receiveInfrastructure();

  // 7. RC카 주행 명령 갱신
  sendCarCommand();
}