#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <Wire.h>

//wiFi 
#define WIFI_SSID     "AL AML D2"
#define WIFI_PASSWORD "AMSH@@2026"


//render server 
#define WS_HOST  "targetdata.onrender.com"
#define WS_PORT  443
#define WS_PATH  "/esp32"


//MPU pins and i2c addresses
#define MPU_SDA   21
#define MPU_SCL   22
#define MPU_ADDR  0x68
#define MPU_PWR   0x6B
#define MPU_ACCEL 0x3B
#define MPU_GYRO  0x43


//motor config
#define MOTOR_IN1     16
#define MOTOR_IN2     17
#define MOTOR_PWM_PIN 18
#define PWM_FREQ      5000
#define PWM_BITS      8


//encoder
#define ENCODER_A 14
#define ENCODER_B 15


//timers
#define IMU_INTERVAL   10
#define SEND_INTERVAL  50
#define PRINT_INTERVAL 1000


WiFiMulti wifiMulti;
WebSocketsClient wsClient;

unsigned long lastIMURead = 0;
unsigned long lastSend    = 0;
unsigned long lastPrint   = 0;

float accelX = 0, accelY = 0, accelZ = 0;
float gyroX  = 0, gyroY  = 0, gyroZ  = 0;
bool  wsConnected = false;

//motor state
bool     motorOn    = false;
int      motorSpeed = 160;

//encoder(volatile bc modified in interrupt)
volatile long encoderCount = 0;

//encoder ISR
void IRAM_ATTR onEncoderA() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

long getEncoderCount() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}


//motor fns 
void motorStart(int speed, bool clockwise = true) {
  speed = constrain(speed, 0, 255);
  digitalWrite(MOTOR_IN1, clockwise ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, clockwise ? LOW  : HIGH);
  ledcWrite(MOTOR_PWM_PIN, speed);
  motorOn    = true;
  motorSpeed = speed;
  Serial.printf("[Motor] Started | Dir: %s | PWM: %d\n",
                clockwise ? "CW" : "CCW", speed);
}

void motorStop() {
  ledcWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  motorOn = false;
  Serial.println("[Motor] Stopped");
}


//MPU fns
void mpuWrite(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

void mpuRead6(uint8_t reg, float &a, float &b, float &c, float scale) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);

  int16_t ra = (Wire.read() << 8) | Wire.read();
  int16_t rb = (Wire.read() << 8) | Wire.read();
  int16_t rc = (Wire.read() << 8) | Wire.read();

  a = ra / scale;
  b = rb / scale;
  c = rc / scale;
}


//command parser
void handleCommand(const char* json) {

  //motor command
  if (strstr(json, "\"type\":\"motor\"")) {
    bool on = strstr(json, "\"on\":true") != nullptr;

    int speed = motorSpeed;
    char* sp  = strstr(json, "\"speed\":");
    if (sp) speed = atoi(sp + 8);

    if (on) { motorStart(speed);}
    else { motorStop();}
    return;
  }

  //encoder reset
  if (strstr(json, "\"type\":\"encoder_reset\"")) {
    noInterrupts();
    encoderCount = 0;
    interrupts();
    Serial.println("[Encoder] Reset to 0");
    return;
  }
}


//WebSocket event handler
void wsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      wsConnected = true;
      Serial.printf("[WS] Connected → %s\n", WS_HOST);
      break;

    case WStype_DISCONNECTED:
      wsConnected = false;
      //stop motor if connection drops not to break chaser's arms
      if (motorOn) {
        motorStop();
        Serial.println("[Safety] Motor stopped on WS disconnect");
      }
      Serial.println("[WS] Disconnected: retrying in 3s...");
      break;

    case WStype_TEXT:
      //command from dashboard
      handleCommand((const char*)payload);
      break;

    case WStype_ERROR:
      Serial.println("[WS] Error");
      break;

    default:
      break;
  }
}


void initMPU() {//manual init
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);
  mpuWrite(MPU_PWR, 0x00);
  Serial.println("MPU6050 Ready!");
}

void initMotor() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_BITS);
  motorStop();
  Serial.println("Motor Ready!");
}

void initEncoder() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), onEncoderA, RISING);
  Serial.println("Encoder Ready!");
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!  IP: " + WiFi.localIP().toString());
}

void connectWebSocket() {
  wsClient.beginSSL(WS_HOST, WS_PORT, WS_PATH);
  wsClient.onEvent(wsEvent);
  wsClient.setReconnectInterval(3000);
  Serial.println("[WS] Connecting to " + String(WS_HOST) + WS_PATH);
}


void setup() {
  Serial.begin(115200);
  delay(500);

  initMPU();
  initMotor();
  initEncoder();
  connectWiFi();
  connectWebSocket();

  Serial.println("Setup done.");
}


void loop() {
  wifiMulti.run();
  wsClient.loop();

  unsigned long now = millis();

  //Read IMU every 10ms
  if (now - lastIMURead >= IMU_INTERVAL) {
    lastIMURead = now;
    mpuRead6(MPU_ACCEL, accelX, accelY, accelZ, 16384.0f);
    mpuRead6(MPU_GYRO,  gyroX,  gyroY,  gyroZ,  131.0f);
  }

  //Send JSON every 50ms
  if (wsConnected && (now - lastSend >= SEND_INTERVAL)) {
    lastSend = now;

    char json[192];
    snprintf(json, sizeof(json),
      "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f"
       ",\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f"
       ",\"enc\":%ld,\"motorOn\":%s,\"motorSpeed\":%d}",
      accelX, accelY, accelZ,
      gyroX,  gyroY,  gyroZ,
      getEncoderCount(),
      motorOn ? "true" : "false",
      motorSpeed);

    wsClient.sendTXT(json);
  }

  //Serial print every 1s
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;

    Serial.println("======== IMU ========");
    Serial.printf("Accel  X: %7.3f g\n",     accelX);
    Serial.printf("Accel  Y: %7.3f g\n",     accelY);
    Serial.printf("Accel  Z: %7.3f g\n",     accelZ);
    Serial.printf("Gyro   X: %7.3f deg/s\n", gyroX);
    Serial.printf("Gyro   Y: %7.3f deg/s\n", gyroY);
    Serial.printf("Gyro   Z: %7.3f deg/s\n", gyroZ);
    Serial.println("======= Motor =======");
    Serial.printf("State:    %s\n",   motorOn ? "ON" : "OFF");
    Serial.printf("Speed:    %d/255\n", motorSpeed);
    Serial.println("====== Encoder ======");
    Serial.printf("Count:    %ld\n",  getEncoderCount());
    Serial.printf("WiFi:     %s\n",   WiFi.status() == WL_CONNECTED ? "✓" : "✗");
    Serial.printf("WS:       %s\n",   wsConnected ? "✓ Connected" : "✗ Disconnected");
    Serial.println("=====================");
  }
}