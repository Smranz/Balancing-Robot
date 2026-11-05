# 🤖 BalanceBot — Self-Balancing Robot with Live Web Tuner

ESP32 + MPU6050 + TB6612FNG + 2× Yellow Gear Motors  
**6V supply** from 5× 1.2V NiMH rechargeable cells

---

## 📁 Project Structure

```
balance-bot/
├── balance_bot/
│   ├── balance_bot.ino   ← Main firmware (flash this to ESP32)
│   └── dashboard.h       ← Web dashboard (PROGMEM HTML, included by .ino)
├── dashboard.html         ← Standalone preview (open in browser for demo)
└── README.md
```

---

## 📦 Required Arduino Libraries

Install via **Arduino IDE → Sketch → Include Library → Manage Libraries**:

| Library | Author | Version |
|---------|--------|---------|
| **WebSockets** | Markus Sattler | 2.3.x or later |
| **ArduinoJson** | Benoit Blanchon | 6.x (NOT v7) |

Built-in ESP32 libraries used (no install needed):
- `WiFi`, `WebServer` (included with ESP32 Arduino core)

---

## 🔌 Wiring

| ESP32 Pin | Connected To |
|-----------|-------------|
| 21 (SDA)  | MPU6050 SDA |
| 22 (SCL)  | MPU6050 SCL |
| 25 (PWMA) | TB6612 PWMA |
| 27 (AIN1) | TB6612 AIN1 |
| 26 (AIN2) | TB6612 AIN2 |
| 14 (PWMB) | TB6612 PWMB |
| 32 (BIN1) | TB6612 BIN1 |
| 33 (BIN2) | TB6612 BIN2 |
| 13 (STBY) | TB6612 STBY |
| 4         | Push button → GND |
| 23        | Active buzzer (+) |
| GND       | Common GND |
| VIN / 3V3 | As needed  |

**Motor power**: Connect 6V (5× NiMH) to TB6612 VM/VCC pins.

---

## 🚀 Setup & Usage

### 1. Flash the firmware
- Open `balance_bot/balance_bot.ino` in Arduino IDE
- Select board: **ESP32 Dev Module**
- Flash speed: 921600 baud
- Upload

### 2. Calibration (automatic on boot)
- Place robot **perfectly still** on a flat surface
- Wait ~3 seconds while gyro calibrates (Serial monitor shows progress)
- 2 short beeps = ready

### 3. Connect to web dashboard
1. On your phone or laptop, connect to WiFi: **`BalanceBot`** / password: **`12345678`**
2. Open browser: **`http://192.168.4.1`**
3. Dashboard loads with live charts

### 4. ARM the robot
- **Physical**: Hold robot upright (< 25° tilt), press and hold the button for 1 second → 1 long beep
- **Web**: Click the **▶ ARM** button (robot must be < 25° from vertical)
- The setpoint is automatically captured at the moment of arming

### 5. DISARM
- Press button once while armed
- Click **■ DISARM** on dashboard
- Robot falls > 30° → auto-disarm

---

## 🌐 Dashboard Panels

| Panel | Description |
|-------|-------------|
| **Robot Tilt** | Animated pendulum SVG showing real-time lean angle. Green/yellow/red zones. |
| **Angle vs Setpoint** | 10-second rolling chart: Kalman angle (blue), setpoint (green dashed), fall limits (red dashed). |
| **Motor Output / PWM** | 10-second rolling chart: PID output (orange), actual motor PWM (purple). |
| **PID Controls** | Live sliders + number inputs for Kp, Ki, Kd, Setpoint, TrimRate. Sends to ESP32 with 300ms debounce. |
| **Live Status** | Real-time values: angle, error, output, integral, motor PWM, gyro rate, loop Hz, uptime, setpoint. |
| **Phase Portrait** | Scatter plot of angle (X) vs angular velocity (Y). A well-tuned robot draws a tight spiral inward. |
| **Kalman Filter** | Compares raw accelerometer angle (gray) vs Kalman-filtered angle (blue). Shows filter effectiveness. |
| **Event Log** | Timestamped log: connections, PID changes, arm/disarm events, errors. |

---

## 🎛️ PID Tuning Guide

### Understanding the parameters

| Parameter | Effect | Start value |
|-----------|--------|-------------|
| **Kp** (Proportional) | Main balancing force. Too low = falls. Too high = oscillates fast. | 10–15 |
| **Ki** (Integral) | Corrects steady-state lean. Too high = slow oscillation / wind-up. | 0 (add last) |
| **Kd** (Derivative) | Damps oscillation. Uses raw gyro rate. | 0.3–0.6 |

### Tuning steps (Ziegler-Nichols inspired):

1. **Start with Ki=0, Kd=0**
2. **Increase Kp** until robot oscillates rapidly (at the balance point)
3. **Back off Kp** by ~30% from oscillation point
4. **Increase Kd** to dampen the oscillation — robot should stand steady
5. **If steady-state lean** (robot drifts in one direction): add small **Ki** (0.05–0.2)
6. **Watch the Phase Portrait**: a well-tuned robot draws a tight inward spiral

### Reading the Phase Portrait
- **Wide spiral** → oscillating, increase Kd or decrease Kp
- **Drifting ellipse** → integrator needed, increase Ki  
- **Tight spiral inward** → perfectly tuned!
- **Explosion outward** → Kp too high

### Auto-trim (TrimRate)
If robot slowly drifts in one direction even with Ki, enable TrimRate (0.01–0.03). This slowly adjusts the setpoint based on the running output average.

---

## 📡 WebSocket Protocol

**Telemetry (ESP32 → Browser)** at 20Hz:
```json
{
  "t": 12345,       "angle": 1.23,     "accAngle": 1.85,
  "gyro": -0.5,     "output": 45.2,    "setpoint": 0.0,
  "error": 1.23,    "integral": 0.02,  "Kp": 12.0,
  "Ki": 0.0,        "Kd": 0.4,         "armed": true,
  "motorPWM": 45,   "trimRate": 0.0,   "loopHz": 198
}
```

**Commands (Browser → ESP32)**:
```json
{ "cmd": "set",           "Kp": 15.0, "Ki": 0.1, "Kd": 0.5 }
{ "cmd": "arm" }
{ "cmd": "disarm" }
{ "cmd": "setpoint",      "value": 1.5 }
{ "cmd": "trim",          "value": 0.02 }
{ "cmd": "reset_integral" }
{ "cmd": "calibrate" }
```

---

## 🔋 Battery Notes

- **5× 1.2V NiMH**: nominal 6.0V, full charge ~6.5V, cutoff ~5.5V
- TB6612FNG VM range: 4.5V–13.5V ✓
- ESP32 powered from onboard LDO via VIN (can take 6V) or 3.3V pin
- Expect ~30–60 min runtime depending on motor load

---

## ⚙️ Configuration (`balance_bot.ino` top section)

```cpp
#define GYRO_SIGN   1      // flip to -1 if angle direction is wrong
#define DIR         1      // flip to -1 if motors drive wrong way
#define MIN_PWM     20     // deadband (increase if motors stall at low PWM)
#define FALL_LIMIT  30.0   // degrees before auto-disarm
#define LOOP_US     5000   // 200 Hz control loop
```

