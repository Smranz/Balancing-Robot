/*
  BalanceBot - Self Balancing Robot with WiFi Live Tuning Dashboard
  ESP32 DevKit + MPU6050 + TB6612FNG + 2x Yellow Gear Motors
  
  Web Dashboard: Connect to WiFi "BalanceBot" (pass: 12345678)
  Then open: http://192.168.4.1
  
  Required Libraries (install via Arduino Library Manager):
    - WebSockets by Markus Sattler (v2.3.x)
    - ArduinoJson by Benoit Blanchon (v6.x)
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ===================== PINS =====================
#define SDA_PIN  21
#define SCL_PIN  22
#define PWMA     25
#define AIN1     27
#define AIN2     26
#define PWMB     14
#define BIN1     32
#define BIN2     33
#define STBY     13
#define BTN_PIN   4
#define BUZ_PIN  23

// ===================== SETTINGS =====================
#define MPU_ADDR   0x68
#define GYRO_SIGN  1
#define DIR        1
#define MIN_PWM    20
#define MAX_PWM    255
#define FALL_LIMIT 30.0f
#define HOLD_MS    1000
#define LOOP_US    5000    // 200 Hz
#define PWM_FREQ   20000
#define PWM_RES    8

// ===================== WiFi / WS =====================
const char* AP_SSID = "hello";
const char* AP_PASS = "12345678";

WebServer    httpServer(80);
WebSocketsServer webSocket(81);

// ===================== PID =====================
float Kp = 12.0f;
float Ki = 0.0f;
float Kd = 0.4f;
float setpoint  = 0.0f;
float setpoint0 = 0.0f;
float trimRate  = 0.0f;
float outAvg    = 0.0f;

// ===================== KALMAN =====================
struct Kalman {
  float Q_angle = 0.001f, Q_bias = 0.003f, R_meas = 0.03f;
  float angle = 0.0f, bias = 0.0f;
  float P[2][2] = {{0,0},{0,0}};

  float update(float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S  = P[0][0] + R_meas;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y  = newAngle - angle;
    angle += K0 * y;
    bias  += K1 * y;
    float P00 = P[0][0], P01 = P[0][1];
    P[0][0] -= K0*P00; P[0][1] -= K0*P01;
    P[1][0] -= K1*P00; P[1][1] -= K1*P01;
    return angle;
  }
};
Kalman kf;

// ===================== STATE =====================
float angle      = 0.0f;
float accAngleRaw = 0.0f;   // raw accel angle (for Kalman chart)
float gyroRate   = 0.0f;
float gyroOff    = 0.0f;
float integral   = 0.0f;
float pidOutput  = 0.0f;
int   motorPWM   = 0;
float pidError   = 0.0f;

float axOff = 0, ayOff = 0, azOff = 0;

unsigned long lastMicros   = 0;
unsigned long armTimer     = 0;
unsigned long beepOff      = 0;
unsigned long lastTelems   = 0;
unsigned long lastHz       = 0;
unsigned long loopCount    = 0;
int           loopHz       = 0;
unsigned long startMs      = 0;

bool armed       = false;
bool lastBtn     = false;
bool needRelease = false;
bool debug       = false;

// ===================== DASHBOARD HTML (PROGMEM) =====================
// Served from PROGMEM to save RAM
#include "dashboard.h"

// ===================== BUZZER =====================
void beep(int ms) {
  digitalWrite(BUZ_PIN, HIGH);
  beepOff = millis() + ms;
}

// ===================== PWM =====================
void pwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, 1);
#endif
}

void pwmWrite(int a, int b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWMA, a); ledcWrite(PWMB, b);
#else
  ledcWrite(0, a); ledcWrite(1, b);
#endif
}

// ===================== MPU6050 =====================
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void readMPU(float &aAngle, float &gRate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  int16_t ax = Wire.read()<<8 | Wire.read();
  int16_t ay = Wire.read()<<8 | Wire.read();
  int16_t az = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t gx = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read(); // gy
  Wire.read(); Wire.read(); // gz

  aAngle = atan2f((float)(ay - ayOff), (float)(az - azOff)) * 180.0f / PI;
  gRate  = GYRO_SIGN * gx / 131.0f;
}

// ===================== MOTORS =====================
void motorDrive(int pwm) {
  pwm = constrain(pwm, -255, 255) * DIR;
  int speed = abs(pwm);
  if (speed > 0) speed = map(speed, 1, 255, MIN_PWM, MAX_PWM);
  bool fwd = (pwm > 0);
  motorPWM = pwm > 0 ? speed : -speed;
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, fwd); digitalWrite(AIN2, !fwd);
  digitalWrite(BIN1, fwd); digitalWrite(BIN2, !fwd);
  pwmWrite(speed, speed);
}

void motorStop() {
  motorPWM = 0;
  pwmWrite(0, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  digitalWrite(STBY, LOW);
}

// ===================== TELEMETRY =====================
void sendTelemetry() {
  StaticJsonDocument<384> doc;
  doc["t"]        = millis() - startMs;
  doc["angle"]    = round(angle * 100.0f) / 100.0f;
  doc["accAngle"] = round(accAngleRaw * 100.0f) / 100.0f;
  doc["gyro"]     = round(gyroRate * 100.0f) / 100.0f;
  doc["output"]   = round(pidOutput * 10.0f) / 10.0f;
  doc["setpoint"] = round(setpoint * 100.0f) / 100.0f;
  doc["error"]    = round(pidError * 100.0f) / 100.0f;
  doc["integral"] = round(integral * 100.0f) / 100.0f;
  doc["Kp"]       = Kp;
  doc["Ki"]       = Ki;
  doc["Kd"]       = Kd;
  doc["armed"]    = armed;
  doc["motorPWM"] = motorPWM;
  doc["trimRate"] = trimRate;
  doc["loopHz"]   = loopHz;

  char buf[384];
  serializeJson(doc, buf);
  webSocket.broadcastTXT(buf);
}

// ===================== WEBSOCKET EVENT =====================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) return;

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, "set") == 0) {
      if (doc.containsKey("Kp")) Kp = doc["Kp"].as<float>();
      if (doc.containsKey("Ki")) Ki = doc["Ki"].as<float>();
      if (doc.containsKey("Kd")) Kd = doc["Kd"].as<float>();
      Serial.printf("[WS] PID updated Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
    }
    else if (strcmp(cmd, "setpoint") == 0) {
      if (doc.containsKey("value")) {
        setpoint  = doc["value"].as<float>();
        setpoint0 = setpoint;
      }
    }
    else if (strcmp(cmd, "trim") == 0) {
      if (doc.containsKey("value")) trimRate = doc["value"].as<float>();
    }
    else if (strcmp(cmd, "arm") == 0) {
      if (!armed && abs(angle) < 25.0f) {
        setpoint  = angle;
        setpoint0 = setpoint;
        integral  = 0; outAvg = 0;
        armed     = true;
        beep(300);
        Serial.printf("[WS] ARMED via web, setpoint=%.2f\n", setpoint);
      }
    }
    else if (strcmp(cmd, "disarm") == 0) {
      motorStop();
      armed = false;
      armTimer = 0;
      needRelease = true;
      beep(800);
      Serial.println("[WS] DISARMED via web");
    }
    else if (strcmp(cmd, "reset_integral") == 0) {
      integral = 0;
      Serial.println("[WS] Integral reset");
    }
    else if (strcmp(cmd, "calibrate") == 0) {
      armed = false;
      motorStop();
      Serial.println("[WS] Recalibrating...");
      float sum = 0, a, g;
      for (int i = 0; i < 500; i++) {
        readMPU(a, g);
        sum += g;
        delay(3);
      }
      gyroOff = sum / 500.0f;
      readMPU(a, g);
      angle = a;
      kf.angle = a;
      kf.bias  = 0;
      integral = 0;
      Serial.printf("[WS] Calibration done. gyroOff=%.4f\n", gyroOff);
      beep(200);
    }
  }
}

// ===================== SERIAL TUNING =====================
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;
  float v = Serial.parseFloat();
  switch (c) {
    case 'p': Kp = v; break;
    case 'i': Ki = v; break;
    case 'd': Kd = v; break;
    case 's': setpoint = v; setpoint0 = v; break;
    case 't': trimRate = v; break;
    case 'v': debug = !debug; break;
  }
  Serial.printf("Kp=%.2f Ki=%.2f Kd=%.2f set=%.2f trim=%.3f\n", Kp, Ki, Kd, setpoint, trimRate);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  startMs = millis();

  // GPIO init
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, LOW);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pwmSetup();
  motorStop();

  // MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00); // wake
  mpuWrite(0x1A, 0x02); // DLPF ~94Hz
  mpuWrite(0x1B, 0x00); // gyro ±250°/s
  mpuWrite(0x1C, 0x00); // accel ±2g
  delay(500);

  // Calibrate gyro
  Serial.println("Calibrating gyro - keep STILL...");
  float sum = 0, a, g;
  for (int i = 0; i < 1000; i++) {
    readMPU(a, g);
    sum += g;
    delay(3);
  }
  gyroOff = sum / 1000.0f;
  readMPU(a, g);
  angle = a;
  kf.angle = a;
  Serial.printf("gyroOff=%.4f  angle=%.2f\n", gyroOff, angle);

  // WiFi AP
  Serial.print("Starting WiFi AP...");
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf(" IP: %s\n", WiFi.softAPIP().toString().c_str());

  // HTTP server - serve dashboard
  httpServer.on("/", HTTP_GET, []() {
    httpServer.sendHeader("Content-Encoding", "identity");
    httpServer.send_P(200, "text/html", DASHBOARD_HTML);
  });
  httpServer.begin();

  // WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("Ready! Connect to WiFi 'BalanceBot' pw:12345678 then open http://192.168.4.1");
  Serial.println("Serial commands: p12 i0 d0.4 s1.5 t0 v");
  beep(100); delay(150); beep(100);

  lastMicros = micros();
  lastHz     = millis();
}

// ===================== LOOP =====================
void loop() {
  // --- Non-blocking network handlers ---
  webSocket.loop();
  httpServer.handleClient();
  handleSerial();

  // --- Buzzer off ---
  if (beepOff && millis() > beepOff) {
    digitalWrite(BUZ_PIN, LOW);
    beepOff = 0;
  }

  // --- 200 Hz control timing ---
  if (micros() - lastMicros < LOOP_US) return;
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  // Loop Hz counter
  loopCount++;
  if (millis() - lastHz >= 1000) {
    loopHz  = loopCount;
    loopCount = 0;
    lastHz  = millis();
  }

  // Button
  bool btn = (digitalRead(BTN_PIN) == LOW);
  bool btnPressed = btn && !lastBtn;
  lastBtn = btn;

  // Read IMU
  readMPU(accAngleRaw, gyroRate);
  gyroRate -= gyroOff;
  angle = kf.update(accAngleRaw, gyroRate, dt);

  // ---- NOT ARMED ----
  if (!armed) {
    motorStop();
    if (!btn) needRelease = false;

    if (btn && !needRelease && abs(angle) < 25.0f) {
      if (armTimer == 0) armTimer = millis();
      if (millis() - armTimer > HOLD_MS) {
        setpoint  = angle;
        setpoint0 = setpoint;
        armed     = true;
        integral  = 0; outAvg = 0;
        beep(300);
        Serial.printf("ARMED setpoint=%.2f\n", setpoint);
      }
    } else {
      armTimer = 0;
    }

    // Send telemetry even when disarmed
    if (millis() - lastTelems >= 50) {
      pidError  = angle - setpoint;
      pidOutput = 0;
      sendTelemetry();
      lastTelems = millis();
    }
    return;
  }

  // ---- ARMED ----
  pidError = angle - setpoint;

  // Disarm button
  if (btnPressed) {
    motorStop();
    armed = false; armTimer = 0; needRelease = true;
    beep(800);
    Serial.println("DISARMED");
    return;
  }

  // Fall detection
  if (abs(pidError) > FALL_LIMIT) {
    motorStop();
    armed = false; armTimer = 0; needRelease = true;
    beep(800);
    Serial.println("FALLEN - disarmed");
    return;
  }

  // PID
  integral += pidError * dt;
  integral  = constrain(integral, -30.0f, 30.0f);

  pidOutput = Kp * pidError + Ki * integral + Kd * gyroRate;

  // Auto-trim
  outAvg   = 0.995f * outAvg + 0.005f * pidOutput;
  setpoint += trimRate * outAvg * dt;
  setpoint  = constrain(setpoint, setpoint0 - 6.0f, setpoint0 + 6.0f);

  motorDrive((int)pidOutput);

  if (debug) Serial.printf("a=%.2f e=%.2f out=%.1f\n", angle, pidError, pidOutput);

  // 20 Hz telemetry
  if (millis() - lastTelems >= 50) {
    sendTelemetry();
    lastTelems = millis();
  }
}

