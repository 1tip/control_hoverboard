// https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32
// MCU : ESP32 T-Display
// 호버보드 제어를 위한  Arduino 프로그램
// RC수신기(PPM, PWM) -> MCU -> 호버보드


/*  2025.10.21 19:25

호버보드용 시리얼 통신모듈 (REMOTE_UART) : RC PPM/PWM입력 -> 신호처리(MCU) -> SERIAL출력 -> 호버보드
  - 통신 프로토콜 맞춤
  - 수신코드 삭제(호버보드 -> MCU)
  - 바퀴 2개 독립제어(좌/우 조향 기능 추가)

<최근수정 - 조종기 움직임에 대한 최적화>
스틱 대각선 입력 시 speed와 steer 합산 벡터를 최대 속도에 맞춰 비율 축소 → 과도 동작 방지
steerScale 유지 → 좌/우 회전 민감도 조절
모터 2개 마주보기 고려 → motor2Value = -fSpeed + fSteer
iSpeedLeft만 제어, iSpeedRight 항상 0
*/

/*  2025.10.21 추가
---------------------------------------------------------------
 ESP32 PPM / PWM 입력 모드 선택형 코드 (버튼 + NVS 저장)
 버튼1(GPIO0) → PPM 모드 선택
 버튼2(GPIO35) → PWM 모드 선택
 비휘발성 메모리에 저장 후 재부팅 시 자동 복원
 LCD에 모드 선택 시 메시지 표시
---------------------------------------------------------------
*/

/* 2025.10.28 추가
   - 수신기능 (RX핀을 통해 Feedback 받음, 호버보드 배터리 전압표시)
   - FailSafe 기능 : 수신기 배터리분리시 모터 오동작 방지
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Preferences.h>

// ---------------- CONFIG ----------------
#define HOVER_TX1 21
#define HOVER_RX1 22
#define HOVER_TX2 17
#define HOVER_RX2 2
#define HOVER_BAUD 19200
#define CRC_POLY 0x1021
#define START_CHAR '/'

#define MAX_SPEED 500
#define MAX_STEER 500

#define BTN1_PIN 0
#define BTN2_PIN 35

// ---------------- 입력 모드 ----------------
enum InputMode { MODE_PPM = 1, MODE_PWM = 2 };
InputMode currentMode = MODE_PWM;

// ---------------- 핀 ----------------
#define PPM_PIN 27
#define PWM_CH1_PIN 27
#define PWM_CH2_PIN 26
#define PWM_CH3_PIN 25
#define PWM_CH4_PIN 33

#define CH_STEER 0
#define CH_SPEED 2

// ---------------- 전역 ----------------
HardwareSerial hoverSerial1(1);
HardwareSerial hoverSerial2(2);
TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

uint16_t channelValues[8] = {1500};
uint16_t lastValue[8] = {0};

// ---------------- 전송 구조체 ----------------
#pragma pack(push, 1)
typedef struct {
  uint8_t  cStart;
  int16_t  iSpeedLeft;
  int16_t  iSpeedRight;
  uint8_t  wStateMaster;
  uint8_t  wStateSlave;
  uint16_t checksum;
} SerialServer2HoverDual;
#pragma pack(pop)

SerialServer2HoverDual hoverTx;

// ---------------- 수신 구조체 ----------------
#pragma pack(push,1)
struct HoverboardData {
    uint16_t startFrame;     
    int16_t  speedL;         
    int16_t  speedR;         
    uint16_t voltage;        
    int16_t  currentL;       
    int16_t  currentR;       
    int32_t  odometerL;      
    int32_t  odometerR;      
    uint16_t checksum;       
};
#pragma pack(pop)

HoverboardData hoverboardData;

// ---------------- CRC ----------------
uint16_t CalcCRC(uint8_t *ptr, int count) {
  uint16_t crc = 0;
  while (--count >= 0) {
    crc ^= (uint16_t)(*ptr++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY : (crc << 1);
  }
  return crc;
}

uint16_t calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0;
    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY : (crc << 1);
    }
    return crc;
}

// ---------------- 모터 제어 ----------------
void sendHoverCmd(int16_t steer, int16_t speed) {
  const float steerScale = 0.5f;
  float fSpeed = (float)speed;
  float fSteer = (float)steer * steerScale;

  float mag = sqrt(fSpeed * fSpeed + fSteer * fSteer);
  if (mag > MAX_SPEED) {
    float scale = MAX_SPEED / mag;
    fSpeed *= scale;
    fSteer *= scale;
  }

  int16_t motor1Value = constrain((int16_t)(fSpeed + fSteer), -MAX_SPEED, MAX_SPEED);
  int16_t motor2Value = constrain((int16_t)(-fSpeed + fSteer), -MAX_SPEED, MAX_SPEED);

  hoverTx.cStart = START_CHAR;
  hoverTx.iSpeedLeft = motor1Value;
  hoverTx.iSpeedRight = 0;
  hoverTx.wStateMaster = 0;
  hoverTx.wStateSlave = 0;
  hoverTx.checksum = CalcCRC((uint8_t *)&hoverTx, sizeof(hoverTx)-2);
  hoverSerial1.write((uint8_t *)&hoverTx, sizeof(hoverTx));

  hoverTx.iSpeedLeft = motor2Value;
  hoverSerial2.write((uint8_t *)&hoverTx, sizeof(hoverTx));

  // TFT 표시
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 38);
  tft.printf("L:%+4d", motor1Value);
  tft.setCursor(120, 38);
  tft.printf("R:%+4d", -motor2Value);
}

// ---------------- 막대그래프 ----------------
void drawVerticalBars() {
  int baseY = 114;
  int maxH = 52;
  int barW = 20;
  int gap = 9;
  int startX = 15;

  uint16_t colors[8] = {
    TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN,
    TFT_CYAN, TFT_BLUE, TFT_MAGENTA, TFT_WHITE
  };

  int channelCount = (currentMode == MODE_PPM) ? 8 : 4;

  for (int i = 0; i < channelCount; i++) {
    uint16_t val = constrain(channelValues[i], 1000, 2000);
    if (abs((int)val - (int)lastValue[i]) < 5) continue;
    lastValue[i] = val;

    int h = map(val, 1000, 2000, 5, maxH);
    int x = startX + i * (barW + gap);
    int y = baseY - h;
    uint16_t fillColor = colors[i % 8];

    tft.fillRect(x, baseY - maxH, barW, maxH, TFT_BLACK);
    tft.fillRect(x, y, barW, h, fillColor);
    tft.drawRect(x, baseY - maxH, barW, maxH, TFT_DARKGREY);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(x + 2, baseY + 5);
    tft.printf("%d", i + 1);
  }
}

// ---------------- PPM ----------------
volatile uint32_t lastRise = 0;
volatile uint8_t ppmIndex = 0;
volatile bool ppmFrameComplete = false;
volatile unsigned long lastPPMTime = 0;

void IRAM_ATTR ppmISR() {
  uint32_t now = micros();
  uint32_t diff = now - lastRise;
  lastRise = now;
  if (diff > 3000) ppmIndex = 0;
  else if (ppmIndex < 8) {
    channelValues[ppmIndex++] = diff;
    if (ppmIndex >= 8) ppmFrameComplete = true;
  }
  lastPPMTime = millis(); // 마지막 PPM 신호 시간 갱신
}

// ---------------- PWM ----------------
uint32_t pulseInSafe(uint8_t pin) {
  uint32_t val = pulseIn(pin, HIGH, 40000);
  return val ? val : 1500;
}

// ---------------- 버튼 ----------------
void checkButtonLongPress() {
    static unsigned long btn1Start = 0;
    static unsigned long btn2Start = 0;

    bool btn1 = digitalRead(BTN1_PIN) == LOW;
    bool btn2 = digitalRead(BTN2_PIN) == LOW;

    if (btn1 && !btn2) {
        if (btn1Start == 0) btn1Start = millis();
        if (millis() - btn1Start >= 3000) {
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(10, 50);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setTextSize(2);
            tft.println("PPM MODE SELECTED");
            tft.println("Saving & Rebooting...");

            prefs.begin("io_mode", false);
            prefs.putUChar("mode", MODE_PPM);
            prefs.end();

            delay(1000);
            ESP.restart();
        }
    } else btn1Start = 0;

    if (btn2 && !btn1) {
        if (btn2Start == 0) btn2Start = millis();
        if (millis() - btn2Start >= 3000) {
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(10, 50);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setTextSize(2);
            tft.println("PWM MODE SELECTED");
            tft.println("Saving & Rebooting...");

            prefs.begin("io_mode", false);
            prefs.putUChar("mode", MODE_PWM);
            prefs.end();

            delay(1000);
            ESP.restart();
        }
    } else btn2Start = 0;
}

// ---------------- SETUP ----------------
void setup() {
  pinMode(BTN1_PIN, INPUT);
  pinMode(BTN2_PIN, INPUT);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 5);

  prefs.begin("io_mode", false);
  currentMode = (InputMode)prefs.getUChar("mode", MODE_PWM);
  prefs.end();

  if (currentMode == MODE_PPM) {
    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
    tft.println(" PPM MODE ");
  } else {
    pinMode(PWM_CH1_PIN, INPUT_PULLUP);
    pinMode(PWM_CH2_PIN, INPUT_PULLUP);
    pinMode(PWM_CH3_PIN, INPUT_PULLUP);
    pinMode(PWM_CH4_PIN, INPUT_PULLUP);
    tft.println(" PWM MODE ");
  }

  hoverSerial1.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX1, HOVER_TX1);
  hoverSerial2.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX2, HOVER_TX2);
}

// ---------------- 호버보드 상태 수신 ----------------
void receiveHoverData(HardwareSerial &serialPort) {
    const uint16_t START_FRAME = 0xABCD;
    if (serialPort.available() > 0) {
        if (serialPort.read() == (START_FRAME & 0xFF)) {
            unsigned long startTime = millis();
            while (serialPort.available() < 1) {
                if (millis() - startTime > 10) return;
            }
            if (serialPort.read() == (START_FRAME >> 8)) {
                size_t bytesToRead = sizeof(HoverboardData) - 2;
                startTime = millis();
                while (serialPort.available() < bytesToRead) {
                    if (millis() - startTime > 200) return;
                }
                uint8_t buffer[sizeof(HoverboardData)];
                buffer[0] = START_FRAME & 0xFF;
                buffer[1] = START_FRAME >> 8;
                serialPort.readBytes(buffer + 2, bytesToRead);
                memcpy(&hoverboardData, buffer, sizeof(HoverboardData));
                uint16_t crc = calculateCRC(buffer, sizeof(HoverboardData)-2);
                if (crc == hoverboardData.checksum) {
                    // TFT 표시
                    tft.setTextSize(2);
                    tft.setTextColor(TFT_GREEN, TFT_BLACK);
                    tft.setCursor(140, 5);
                    tft.printf("%.2fV ", hoverboardData.voltage / 100.0f);
                }
            }
        }
    }
}

// ---------------- LOOP ----------------
void loop() {
    checkButtonLongPress();

    bool signalLost = false;
    const unsigned long SIGNAL_TIMEOUT = 80; // GPT추천 적정값 100 ms,  RC수신기 신호 Lost 타임아웃

    if (currentMode == MODE_PPM) {
        if (ppmFrameComplete) ppmFrameComplete = false;
        if (millis() - lastPPMTime > SIGNAL_TIMEOUT) signalLost = true;
    } else {
        // PWM 모드에서는 pulseInSafe값이 0~1500 이상이면 정상, 아닐 경우 신호 끊김
        for (int i=0;i<4;i++){
            if(channelValues[i]<900 || channelValues[i]>2100){
                signalLost = true;
                break;
            }
        }
    }

    int16_t steer=0, speed=0;
    if(signalLost){
        steer=0;
        speed=0;
    }else{
        if(currentMode==MODE_PWM){
            channelValues[0] = pulseInSafe(PWM_CH1_PIN);
            channelValues[1] = pulseInSafe(PWM_CH2_PIN);
            channelValues[2] = pulseInSafe(PWM_CH3_PIN);
            channelValues[3] = pulseInSafe(PWM_CH4_PIN);
        }
        steer = map(constrain(channelValues[CH_STEER], 1000, 2000), 1000, 2000, -MAX_STEER, MAX_STEER);
        speed = map(constrain(channelValues[CH_SPEED], 1000, 2000), 1000, 2000, -MAX_SPEED, MAX_SPEED);
    }

    sendHoverCmd(steer, speed);
    drawVerticalBars();

    receiveHoverData(hoverSerial1);
    //receiveHoverData(hoverSerial2);
}
