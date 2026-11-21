# LFR
# High-Speed Competition LFR v2

A high-speed **Line Following Robot (LFR)** designed for robotics competitions using a 6-channel analog sensor array, TB6612 motor driver, PID control, automatic polarity detection, junction handling, dashed-line recovery, and EEPROM-based dual-polarity calibration.

The robot supports both:

* ⚫ **NORMAL MODE** — Black line on white surface
* ⚪ **INVERSE MODE** — White line on black surface

The system can automatically detect a polarity change during operation and switch between modes without stopping the robot.

---

## 🚀 Features

* ⚡ High-speed line following
* 🎯 PID-based steering control
* 🔄 Automatic NORMAL ↔ INVERSE polarity detection
* 🔧 Manual polarity switching
* 💾 EEPROM calibration storage
* 🎛️ Separate calibration profile for each polarity
* ↩️ Left-priority junction handling
* ↪️ Right junction detection
* ⬆️ Straight-path priority
* 🛣️ Dashed-line / line-gap recovery
* 🏁 Finish-point detection
* 🔍 Sensor-based 90° pivot turns
* ⚙️ Fast ADC configuration
* 💡 LED status indication
* 🖲️ Single-button control system

---

## 🧠 Control Logic

The robot continuously reads all six sensors and calculates the position of the line using weighted sensor values.

### Sensor Weights

| Sensor | Position     | Weight |
| ------ | ------------ | -----: |
| S1     | Far Left     |  -2500 |
| S2     | Mid Left     |  -1500 |
| S3     | Center Left  |   -500 |
| S4     | Center Right |   +500 |
| S5     | Mid Right    |  +1500 |
| S6     | Far Right    |  +2500 |

The calculated position is used as the PID error.

```text
Left                         Right
S1     S2     S3     S4     S5     S6
-2500 -1500  -500   +500  +1500  +2500
```

---

# 🔌 Hardware

## Main Components

* Arduino-compatible microcontroller
* 6 × Analog IR/reflective sensors
* TB6612FNG motor driver
* 2 × DC geared motors
* Robot chassis
* Wheels
* Battery
* Push button
* LED
* Jumper wires

---

# 📌 Pin Configuration

## Sensor Pins

| Sensor | Arduino Pin | Function                     |
| ------ | ----------- | ---------------------------- |
| S1     | A7          | Far Left / Branch Detection  |
| S2     | A6          | Mid Left                     |
| S3     | A3          | Center Left                  |
| S4     | A2          | Center Right                 |
| S5     | A1          | Mid Right                    |
| S6     | A0          | Far Right / Branch Detection |

## TB6612 Motor Driver

| Pin  | Arduino Pin | Function              |
| ---- | ----------: | --------------------- |
| PWMA |          D3 | Left Motor PWM        |
| AIN2 |          D4 | Left Motor Direction  |
| AIN1 |          D5 | Left Motor Direction  |
| STBY |          D6 | Driver Standby        |
| BIN1 |          D7 | Right Motor Direction |
| BIN2 |          D8 | Right Motor Direction |
| PWMB |          D9 | Right Motor PWM       |

## Control

| Component | Pin |
| --------- | --: |
| Button    | D11 |
| LED       | D13 |

---

# ⚙️ Speed Configuration

The main speed parameters can be adjusted from the code:

```cpp
int BASE_SPEED = 135;
int MAX_SPEED  = 200;
int TURN_SPEED = 125;
int GAP_SPEED  = 130;
```

### Description

* `BASE_SPEED` → Normal cruising speed
* `MAX_SPEED` → Maximum motor speed
* `TURN_SPEED` → Pivot-turn speed
* `GAP_SPEED` → Speed used when crossing line gaps

For competition tuning, these values should be adjusted according to:

* Motor RPM
* Battery voltage
* Track surface
* Sensor height
* Robot weight
* Track complexity

---

# 🎯 PID Control

The robot uses PID-based steering.

Current parameters:

```cpp
float Kp = 0.058;
float Ki = 0.0000;
float Kd = 0.280;
```

### PID Components

```text
P = Current Error
I = Accumulated Error
D = Change in Error
```

The motor correction is calculated as:

```cpp
correction = Kp * P + Ki * I + Kd * D;
```

Then:

```cpp
leftMotor  = BASE_SPEED + correction;
rightMotor = BASE_SPEED - correction;
```

This allows the robot to smoothly follow curves and straight sections.

---

# 🔄 Dual-Polarity / Inverse Mode

The major feature of this version is the **dual-polarity track support**.

### NORMAL MODE

```text
White Surface
──────────────
     BLACK
      LINE
──────────────
```

### INVERSE MODE

```text
Black Surface
──────────────
     WHITE
      LINE
──────────────
```

The sensor values are normalized between `0–1000`.

In inverse mode, the value is flipped:

```cpp
if (inverseMode)
    v = 1000 - v;
```

This keeps the rest of the line-following algorithm unchanged.

---

# 🤖 Automatic Polarity Detection

Automatic inverse detection is enabled by default:

```cpp
const bool AUTO_INVERSE = true;
```

When most sensors detect a uniform surface, the controller checks the opposite polarity.

If the opposite polarity contains a coherent line near the center, the robot assumes that the track polarity has changed.

It then automatically switches:

```text
NORMAL
   ↓
Detect Inverse Zone
   ↓
Verify Center Line
   ↓
INVERSE
```

The system also uses a confirmation time and flip lockout to prevent accidental switching.

---

# 🏁 Finish Detection

The finish detector is designed to distinguish between:

### Polarity Change

```text
Current polarity → Uniform
Opposite polarity → Centered line
```

### Finish Box

```text
Current polarity → Uniform
Opposite polarity → No line
```

The robot performs a short forward verification before stopping.

This prevents an inverse zone from being incorrectly detected as the finish point.

---

# ↩️ Junction Detection

The robot supports 90-degree branches and T-junctions.

Far-left and far-right sensors are used for branch detection.

```cpp
TH_BRANCH = 600;
TH_CONFIRM = 550;
```

### Priority Rule

The junction decision follows:

```text
1. LEFT
2. RIGHT
3. STRAIGHT
```

So when both left and right branches are detected:

```text
LEFT branch  → Turn Left
RIGHT branch → Turn Right
Both         → Turn Left
None         → Continue Straight
```

A short forward nudge is performed before confirming the junction. This helps prevent false turns caused by curves touching the far sensors.

---

# ↪️ Pivot Turning

The robot uses responsive sensor-based pivot turns.

Instead of depending only on a fixed delay, the robot continuously reads the center sensors during the turn.

### Left Turn

```cpp
setMotors(-TURN_SPEED, TURN_SPEED);
```

### Right Turn

```cpp
setMotors(TURN_SPEED, -TURN_SPEED);
```

The turn ends when the center sensors detect the line again.

This allows the robot to adapt better to different track conditions.

---

# 🛣️ Dashed Line / Line Gap Recovery

If none of the sensors detect the line:

```text
activeCount == 0
```

the robot assumes it may be crossing a dashed-line gap.

It temporarily continues forward at:

```cpp
GAP_SPEED
```

If the line is not recovered within the timeout, the robot uses the previous error to determine which direction to search.

```text
Previous Error < 0
       ↓
   Search Left

Previous Error > 0
       ↓
   Search Right
```

---

# 🖲️ Button Controls

The robot uses a single push button.

| Button Action         | Function                        |
| --------------------- | ------------------------------- |
| Short press `< 1 sec` | Start / Stop                    |
| Hold `1–3 sec`        | Auto-calibrate current polarity |
| Hold `> 3 sec`        | Toggle NORMAL ↔ INVERSE         |

### LED Indication

* **LED ON** → Running / calibration indication
* **LED solid during hold** → Calibration mode
* **LED blinking during long hold** → Inverse toggle mode

---

# 🔧 Auto Calibration

The robot supports automatic sensor calibration.

During calibration, the robot rotates in both directions while recording minimum and maximum sensor readings.

```text
Sensor Min → Lowest detected value
Sensor Max → Highest detected value
```

Calibration is stored separately for:

```text
Slot 0 → NORMAL
Slot 1 → INVERSE
```

If an inverse calibration slot is unavailable, the robot falls back to the normal calibration.

---

# 💾 EEPROM Storage

Calibration data is permanently stored in EEPROM.

The memory structure contains:

```text
Magic Number
      ↓
NORMAL calibration
      ↓
INVERSE calibration
      ↓
Calibration validity flags
      ↓
Current polarity mode
```

Therefore, calibration data remains available after restarting the Arduino.

---

# 📊 Detection Thresholds

Current threshold configuration:

```cpp
const int TH_LINE    = 380;
const int TH_BRANCH  = 600;
const int TH_CONFIRM = 550;
const int TH_CENTER  = 550;
```

| Threshold    | Purpose                     |
| ------------ | --------------------------- |
| `TH_LINE`    | Normal line detection       |
| `TH_BRANCH`  | Far sensor branch detection |
| `TH_CONFIRM` | Junction confirmation       |
| `TH_CENTER`  | Pivot-turn exit detection   |

These values may need adjustment depending on sensor type, sensor height, lighting, and track material.

---

# 🛠️ Installation

## 1. Install Arduino IDE

Install the Arduino IDE and select the correct board.

## 2. Connect Hardware

Connect:

```text
6 IR Sensors
      ↓
Arduino
      ↓
TB6612FNG
   ↓       ↓
Left     Right
Motor    Motor
```

Connect the button to `D11` and LED to `D13`.

---

## 3. Upload Code

Open the `.ino` file in Arduino IDE.

Select:

```text
Tools → Board
Tools → Port
```

Then click:

```text
Upload
```

---

# 🔧 First-Time Setup

After assembling the robot:

### Step 1 — Place robot on the track

Make sure the sensors are at the correct height.

### Step 2 — Calibrate NORMAL mode

Hold the button for **1–3 seconds**.

The robot will rotate and collect sensor minimum/maximum values.

### Step 3 — Calibrate INVERSE mode

Switch to inverse mode by holding the button for **more than 3 seconds**.

Then perform calibration again.

### Step 4 — Start

Press the button briefly.

The robot will start following the line.

---

# 🎛️ Competition Tuning

For better competition performance, tune parameters in this order:

### 1. Sensor Height

Keep all sensors at a consistent distance from the track.

### 2. `TH_LINE`

Adjust line detection sensitivity.

### 3. `BASE_SPEED`

Increase gradually until the robot becomes unstable, then reduce slightly.

### 4. `Kp`

Controls how strongly the robot reacts to the current error.

### 5. `Kd`

Controls how strongly the robot reacts to rapid changes in error.

### 6. Junction Parameters

Tune:

```cpp
TH_BRANCH
TH_CONFIRM
TH_CENTER
```

for reliable branch detection.

---

# 🧩 Project Structure

```text
LFR/
│
├── LFR.ino
├── README.md
│
└── EEPROM
    └── Calibration Data
```

---

# 🧪 Serial Monitor

The robot communicates through:

```text
115200 baud
```

The Serial Monitor can be used to observe:

* Calibration status
* Current polarity
* EEPROM status
* Polarity switching
* Finish detection

Example:

```text
AI Calibration Loaded!
Normal slot : OK
Inverse slot: OK

Start polarity: NORMAL (black line on white)

>>> POLARITY SWITCHED -> INVERSE (white line on black)
```

---

# ⚠️ Important Notes

* Always calibrate the sensors before competition.
* Make sure the sensor values are stable.
* Keep the sensor array parallel to the track.
* Avoid excessive motor speed before PID tuning.
* Test inverse-mode transitions separately.
* Verify left/right motor direction before running.
* Check battery voltage before high-speed operation.
* Test junction detection at competition speed.

---

# 📈 Future Improvements

Possible future upgrades:

* [ ] Adaptive PID tuning
* [ ] Dynamic speed control based on curvature
* [ ] Sensor filtering
* [ ] Automatic speed reduction before junctions
* [ ] More advanced junction classification
* [ ] OLED/LCD debugging interface
* [ ] Bluetooth/Wi-Fi telemetry
* [ ] Competition-specific track learning
* [ ] Encoder-based speed balancing

---

# 👨‍💻 Project Type

**AI High-Speed Competition Line Following Robot**

### Technologies

* Arduino / Embedded C++
* Analog IR Sensors
* PID Control
* TB6612FNG Motor Driver
* EEPROM
* Real-Time Sensor Processing

---

# ⭐ Highlights

> **Fast • Adaptive • Dual-Polarity • PID Controlled • Competition Ready**

This project combines real-time sensor processing, PID control, automatic polarity detection, junction decision-making, and persistent EEPROM calibration into a single high-speed LFR control system.
