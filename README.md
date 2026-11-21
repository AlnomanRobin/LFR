# 🏎️ 6-Sensor High-Speed PID Line Follower Robot (LFR)

এই প্রজেক্টটি ন্যাশনাল বা কলেজ লেভেলের রোবটিক্স কম্পিটিশন (যেমন NRC, TechFest)-এর জন্য বিশেষভাবে ডিজাইন করা একটি ফুল-ফিচার্ড **PID Line Follower Robot** কোড।

---

## 📌 পিন কনফিগারেশন সামারি (Pinout Diagram)

### ১. IR Sensor Array (6 Sensors)
> ⚠️ **বিশেষ দ্রষ্টব্য:** Arduino Nano-তে **A6 এবং A7** কেবল অ্যানালগ পিন (এদের কোনো ডিজিটাল বাফার নেই)। তাই কোডে সম্পূর্ণ অ্যানালগ রিডিং ও সফটওয়্যার থ্রেশহোল্ড দিয়ে হ্যান্ডেল করা হয়েছে।

| Sensor | Arduino Pin | Function |
|---|---|---|
| **S1** | **A7** | Far Left / 90° Branch Detection |
| **S2** | **A6** | Mid Left |
| **S3** | **A3** | Center Left |
| **S4** | **A2** | Center Right |
| **S5** | **A1** | Mid Right |
| **S6** | **A0** | Far Right / 90° Branch Detection |

---

### ২. TB6612FNG Motor Driver
| Driver Pin | Arduino Pin | Function |
|---|---|---|
| **PWMA** | **D3** | Left Motor Speed (PWM) |
| **AIN2** | **D4** | Left Motor Direction |
| **AIN1** | **D5** | Left Motor Direction |
| **STBY** | **D6** | Standby Pin (Always HIGH) |
| **BIN1** | **D7** | Right Motor Direction |
| **BIN2** | **D8** | Right Motor Direction |
| **PWMB** | **D9** | Right Motor Speed (PWM) |
| **VCC**  | **5V** | Logic Power |
| **VM**   | **Battery (+)** | Motor Power (7.4V - 11.1V LiPo) |
| **GND**  | **GND** | Common Ground |

---

### ৩. Control Buttons (Internal Pull-Up)
বাটনগুলোর একটি পা Arduino পিনে এবং অপর পা গ্রাউন্ডে (GND) লাগাবেন।

| Button | Arduino Pin | Function |
|---|---|---|
| **UP / NEXT** | **D2** | মেনু উপরে নেওয়া / ভ্যালু বাড়ানো |
| **DOWN / PREV** | **D10** | মেনু নিচে নামানো / ভ্যালু কমানো |
| **OK / ENTER** | **D11** | সিলেক্ট করা / সেভ করা / স্টপ করা |

---

### ৪. 0.96" I2C OLED Display (SSD1306)
| OLED Pin | Arduino Pin |
|---|---|
| **SDA** | **A4** |
| **SCL** | **A5** |
| **VCC** | **5V** |
| **GND** | **GND** |

---

## 📦 প্রয়োজনীয় Arduino Libraries
Arduino IDE-এর **Library Manager** (`Ctrl + Shift + I`) থেকে নিচের লাইব্রেরি দুটি ইনস্টল করুন:
1. `Adafruit SSD1306` by Adafruit
2. `Adafruit GFX Library` by Adafruit

---

## 🚀 বিশেষ ফিচারসমূহ (Features)

1. **ইন্টারেক্টিভ OLED মেনু:** রোবট চালু থাকা অবস্থাতেই ডিসপ্লে এবং বাটনের মাধ্যমে PID টিউন এবং ক্যালিব্রেশন করা যায়। ল্যাপটপ কানেক্ট করার প্রয়োজন নেই!
2. **অটো ক্যালিব্রেশন (Auto Calibration):** সাদা ও কালো ট্র্যাকের লাইট কন্ডিশন অনুযায়ী রোবট নিজে নিজেই প্রতিটি সেন্সরের Min/Max ও পারফেক্ট থ্রেশহোল্ড নির্ধারণ করে।
3. **EEPROM সেভ সিস্টেম:** টিউন করা Kp, Ki, Kd এবং স্পিড সেটিংস মেমোরিতে সেভ থাকে; পাওয়ার অফ করলেও মুছে যায় না।
4. **স্মুথ ওয়েইটেড পজিশন PID:** রোবট আঁকাবাঁকা রাস্তায় জার্কিং ছাড়াই সুপার স্মুথ চলে।
5. **লাইন লস রিকভারি (Line-Loss Recovery):** ৯০ ডিগ্রি বা তীব্র বাঁকে লাইন হারিয়ে গেলে রোবট ছিটকে না গিয়ে লাস্ট পজিশন মনে রেখে স্পিন করে লাইনে ফিরে আসে।
6. **লাইভ সেন্সর মনিটর:** ডিসপ্লেতে ৬টি সেন্সরের রিয়েল-টাইম গ্রাফ দেখা যায়।

---

## 🛠️ টিউনিং গাইড (How to Tune PID)

1. **Sensor Calibration:**
   - মেনু থেকে `2. Calibrate Sensors` সিলেক্ট করুন।
   - ৬ সেকেন্ডের মধ্যে রোবটটিকে ম্যানুয়ালি ট্র্যাকের সাদা ও কালো লাইনের ওপর ডানে-বামে নাড়াচাড়া করুন।
   - প্রসেস শেষে `CALIBRATION SAVED!` মেসেজ আসবে।

2. **Sensor Monitor চেক:**
   - মেনু থেকে `3. Sensor Monitor` এ যান। সেন্সরগুলো লাইনের ওপর নিলে ডিসপ্লেতে গ্রাফ ও `1` উঠছে কিনা চেক করুন।

3. **PID টিউনিং স্টেপস:**
   - **Step 1:** প্রথমে `Ki = 0.000` রাখুন।
   - **Step 2:** `Kp` বাড়াতে থাকুন যতক্ষণ না রোবট হালকা দুলতে দুলতে হলেও লাইনে থাকতে পারছে (যেমন: 0.035 - 0.060)।
   - **Step 3:** এরপর `Kd` বাড়ান যাতে দুলুনি (oscillation) বন্ধ হয়ে রোবট একদম সোজা চলে (যেমন: 0.300 - 0.500)।
   - **Step 4:** রোবট পারফেক্ট চললে আস্তে আস্তে `Base Speed` (যেমন: 160 থেকে 200+) বাড়ান।
