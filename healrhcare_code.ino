#include <DFRobot_BloodOxygen_S.h>
#include <DFRobot_RTU.h>
#include <math.h> 
#include <TimerOne.h> 

// ==========================================
// 1. 하드웨어 핀 및 통신 설정 (Mega 최적화)
// ==========================================
// 블루투스는 Mega의 강력한 하드웨어 시리얼(Serial2) 사용 (TX: 17번, RX: 16번)
#define BT Serial2 

#define I2C_COMMUNICATION  
#ifdef I2C_COMMUNICATION
  #define I2C_ADDRESS 0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, I2C_ADDRESS);
#endif

// 핀 설정
const uint8_t Button1 = 2;    // 외부 인터럽트 가능 핀
const uint8_t Button2 = 3;    // 외부 인터럽트 가능 핀
const uint8_t Relay1 = 6;     // 히터
const uint8_t Relay2 = 7;     // 냉각기
const uint8_t Relay_M1 = 9;  
const uint8_t Relay_M2 = 4;   // Mega의 I2C 핀(20번)을 피해서 4번으로 이동
const uint8_t BuzzerPin = 12; // 알람 부저
const uint8_t thermistorPin = A1; 

// ==========================================
// 2. 기본 전역 변수 (기존의 직관적인 방식)
// ==========================================
int hbSampleCount = 0;   
int spo2SampleCount = 0; 
long heartbeatsum = 0;   
long spo2sum = 0; 
int heartbeataverage = 0;
int spo2average = 0;

volatile int motorState = 0;        
volatile int tempControlState = 0;  
int targetTemp = 34;       

unsigned long previousMillis2 = 0; 
unsigned long previousMillisTemp = 0; 
const long interval2 = 5000; 
const long intervalTemp = 1000; 
volatile bool flagReadSensor = false; 

// ==========================================
// 3. 온도 필터 & 시분할 PID 제어 변수
// ==========================================
float filteredTemp = -100.0;        
const float alpha = 0.2;            

const float Kp = 3000.0; 
const float Ki = 10.0;
const float Kd = 500.0;

float integral = 0;
float previousError = 0;
long pidOutput = 0; 

const unsigned long pidWindowSize = 10000; // 10초 주기
unsigned long windowStartTime = 0;

const float B_CONSTANT = 3435.0; 
const float R25 = 10000.0; 
const float R2 = 10000.0; 

// ==========================================
// 4. 인터럽트 서비스 루틴 (ISR)
// ==========================================
void timerIsr() {
  flagReadSensor = true; 
}

volatile unsigned long lastDebounce1 = 0;
void button1ISR() {
  if (millis() - lastDebounce1 > 200) { 
    tempControlState = (tempControlState + 1) % 2;
    lastDebounce1 = millis();
  }
}

volatile unsigned long lastDebounce2 = 0;
void button2ISR() {
  if (millis() - lastDebounce2 > 200) {
    motorState = (motorState + 1) % 3;
    lastDebounce2 = millis();
  }
}

// ==========================================
// 5. Setup
// ==========================================
void setup() {
  Serial.begin(9600); 
  BT.begin(9600); 

  while (false == MAX30102.begin()) {
    Serial.println("MAX30102 init fail!");
    delay(1000);
  }
  MAX30102.sensorStartCollect();

  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT); 
  pinMode(Relay_M1, OUTPUT);
  pinMode(Relay_M2, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  
  pinMode(Button1, INPUT_PULLUP);  
  pinMode(Button2, INPUT_PULLUP);  
  
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay_M1, HIGH);
  digitalWrite(Relay_M2, HIGH);

  attachInterrupt(digitalPinToInterrupt(Button1), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Button2), button2ISR, FALLING);

  Timer1.initialize(100000); 
  Timer1.attachInterrupt(timerIsr); 

  windowStartTime = millis(); 
}

// ==========================================
// 6. 메인 Loop
// ==========================================
void loop() { 
  unsigned long currentMillis = millis();

  // --- [센서 읽기 타이머 처리] ---
  if (flagReadSensor) {
    flagReadSensor = false; 
    readMax30102(); 
  }

  // --- [블루투스 통신] ---
  if(BT.available()) {
    static String receivedData = "";
    char incomingChar = BT.read();
    if(incomingChar >= '0' && incomingChar <= '9') {
      receivedData += incomingChar;
      if (receivedData.length() == 4) {
        int data = receivedData.toInt();
        targetTemp = data / 100;                
        motorState = (data / 10) % 10;          
        tempControlState = data % 10;           
        receivedData = "";
      }
    } else { receivedData = ""; }
  }

  // --- [생체 데이터 평균 계산 및 출력 (5초 마다)] ---
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;

    heartbeataverage = (hbSampleCount > 0) ? (heartbeatsum / hbSampleCount) : 0;
    spo2average = (spo2SampleCount > 0) ? (spo2sum / spo2SampleCount) : 0;

    Serial.print("HR: "); Serial.print(heartbeataverage);
    Serial.print(" BPM | SpO2: "); Serial.print(spo2average);
    Serial.println(" %");
    
    BT.print(String(spo2average) + "/" + String(heartbeataverage) + "\r\n");
    
    hbSampleCount = 0; spo2SampleCount = 0;
    heartbeatsum = 0; spo2sum = 0;
  }

  // --- [직관적인 부저 알람 실행] ---
  // 상태 머신 없이, 센서 값을 그대로 전달하여 경고음을 울리게 합니다.
  handleBuzzerAlarm(heartbeataverage, spo2average);

  // --- [온도 측정 및 PID 계산 (1초 마다)] ---
  if (currentMillis - previousMillisTemp >= intervalTemp) {
    previousMillisTemp = currentMillis;
    
    int adcValue = analogRead(thermistorPin);
    float voltage = (5.0 * adcValue) / 1023.0; 
    
    if(voltage > 0.0) {
      float R1 = (5.0 * R2) / voltage - R2; 
      float rawTemperature = 1.0 / ((1.0 / (273.15 + 25.0)) + (log(R1 / R25) / B_CONSTANT)) - 273.15;
      
      filteredTemp = (filteredTemp == -100.0) ? rawTemperature : (alpha * rawTemperature) + ((1.0 - alpha) * filteredTemp);
      
      float error = targetTemp - filteredTemp;
      integral += error;
      integral = constrain(integral, -100, 100); 
      
      float derivative = error - previousError;
      pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
      previousError = error;
      
      pidOutput = constrain(pidOutput, -(long)pidWindowSize, (long)pidWindowSize);
    }
  }

  // --- [릴레이 시분할 PID 구동 (매 사이클 확인)] ---
  if (currentMillis - windowStartTime >= pidWindowSize) {
    windowStartTime += pidWindowSize;
  }
  unsigned long windowElapsed = currentMillis - windowStartTime;

  if (tempControlState == 1) { // 원래 작성하셨던 직관적인 조건문으로 복구
    if (pidOutput > 0) { 
      if (windowElapsed < (unsigned long)pidOutput) {
        digitalWrite(Relay1, LOW);  
        digitalWrite(Relay2, HIGH); 
      } else {
        digitalWrite(Relay1, HIGH); 
        digitalWrite(Relay2, HIGH);
      }
    } else if (pidOutput < 0) {
      if (windowElapsed < (unsigned long)abs(pidOutput)) {
        digitalWrite(Relay1, HIGH);
        digitalWrite(Relay2, LOW);  
      } else {
        digitalWrite(Relay1, HIGH); 
        digitalWrite(Relay2, HIGH); 
      }
    } else {
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, HIGH);
    }
  } else { 
    digitalWrite(Relay1, HIGH);
    digitalWrite(Relay2, HIGH);
  }

  // --- [모터 구동 (원래 작성하셨던 직관적인 조건문)] ---
  if (motorState == 1) {
    digitalWrite(Relay_M1, LOW); 
    digitalWrite(Relay_M2, HIGH);
  } else if (motorState == 2) {
    digitalWrite(Relay_M1, HIGH); 
    digitalWrite(Relay_M2, LOW);
  } else {
    digitalWrite(Relay_M1, HIGH); 
    digitalWrite(Relay_M2, HIGH);
  }
}

// ==========================================
// [함수] 센서 측정 (이상치 제거 필터 유지)
// ==========================================
void readMax30102() {
  MAX30102.getHeartbeatSPO2();
  int currentHR = MAX30102._sHeartbeatSPO2.Heartbeat;
  int currentSpO2 = MAX30102._sHeartbeatSPO2.SPO2;

  if (currentHR >= 40 && currentHR <= 200) {
    heartbeatsum += currentHR;
    hbSampleCount++;
  }
  if (currentSpO2 >= 70 && currentSpO2 <= 100) {
    spo2sum += currentSpO2;
    spo2SampleCount++;
  }
}

// ==========================================
// [함수] 부저 알람 (직관적인 if-else 구조로 변경)
// ==========================================
void handleBuzzerAlarm(int hr, int spo2) {
  static unsigned long lastBuzzerToggle = 0;
  static bool buzzerIsOn = false;

  // 센서가 아직 값을 못 읽었을 때는 소리 끔
  if (hr == 0 || spo2 == 0) {
    digitalWrite(BuzzerPin, LOW); 
    buzzerIsOn = false;
    return;
  }

  int interval = 0; // 알람 간격(ms)
  
  // 1. 심각한 수치 (0.2초 간격으로 빠르게 울림)
  if (hr < 50 || hr > 150 || spo2 < 90) {
    interval = 200; 
  } 
  // 2. 경고 수치 (1초 간격으로 느리게 울림)
  else if (hr > 100 || spo2 < 95) {
    interval = 1000; 
  }

  // 알람 울리기 로직
  if (interval == 0) {
    digitalWrite(BuzzerPin, LOW); // 정상이면 끄기
    buzzerIsOn = false;
  } else {
    if (millis() - lastBuzzerToggle >= interval) {
      lastBuzzerToggle = millis();
      buzzerIsOn = !buzzerIsOn;
      digitalWrite(BuzzerPin, buzzerIsOn ? HIGH : LOW);
    }
  }
}
