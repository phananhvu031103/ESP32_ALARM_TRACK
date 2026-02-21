# ESP32 GPS Tracking & Alarm System

Hệ thống theo dõi và báo động GPS cho xe máy sử dụng ESP32, MPU6050, GPS ATGM336H và module SIM A7682S.

## 📋 Tính năng chính

### Báo động

- **4 chế độ hoạt động:**

  - `STAGE_NONE`: Chủ xe có mặt, hệ thống ở chế độ nghỉ
  - `STAGE_WARNING`: Chủ xe vắng mặt, giám sát chuyển động
  - `STAGE_ALERT`: Phát hiện chuyển động mạnh, kích hoạt còi & LED gửi SMS theo mốc khoảng cách
  - `STAGE_TRACKING`: Xe di chuyển > 10m, gửi vị trí định kỳ qua SMS/MQTT

- **Phát hiện chuyển động thông minh:**
  - Sử dụng MPU6050 với Kalman Filter để lọc nhiễu
  - Phân biệt chuyển động nhẹ và chuyển động mạnh
  - Tự động leo thang cảnh báo khi có chuyển động kéo dài

### 📡 Theo dõi vị trí

- GPS ATGM336H với Kalman Filter để tăng độ chính xác
- Tính toán khoảng cách di chuyển theo thời gian thực
- Gửi vị trí qua SMS với link Google Maps
- Gửi dữ liệu lên MQTT broker (HiveMQ)

### 📱 Gửi SMS tự động

- SMS cảnh báo khi phát hiện chuyển động
- SMS vị trí định kỳ khi ở chế độ TRACKING
- SMS cảnh báo pin yếu
- SMS cảnh báo mất tín hiệu GPS
- Hệ thống cooldown thông minh để tránh spam

### 🔋 Quản lý nguồn thông minh

- **Deep Sleep:** Khi chủ xe có mặt, tiết kiệm pin tối đa
- **Light Sleep:** Khi ở chế độ WARNING không có chuyển động
- Tự động đánh thức khi:
  - Phát hiện chuyển động (MPU interrupt)
  - Nhấn nút MODE
  - Định kỳ 20 phút để kiểm tra (đối với STAGE_NONE và STAGE_WARNING)

### 🎮 Điều khiển

- **Nút MODE:** Chuyển đổi trạng thái chủ xe (có mặt/vắng mặt)
- **SMS Commands:**
  - `GPS`: Lấy vị trí hiện tại
  - `STOP TRACKING`: Dừng chế độ theo dõi
  - `STOP SMS`: Tắt gửi SMS vị trí định kỳ

## 🔧 Phần cứng

### Linh kiện chính

| Linh kiện | Model                | Chức năng            |
| --------- | -------------------- | -------------------- |
| MCU       | ESP32 DOIT DevKit V1 | Vi điều khiển chính  |
| IMU       | MPU6050              | Cảm biến gia tốc/góc |
| GPS       | ATGM336H             | Định vị GPS          |
| SIM       | A7682S               | Gửi SMS & MQTT       |
| Buzzer    | Active Buzzer        | Còi báo động         |
| LED       | LED xanh             | Đèn cảnh báo         |
| Pin       | 2S Li-ion (7.4V)     | Nguồn cấp            |

### Sơ đồ chân kết nối

```
ESP32 Pin    →    Component
─────────────────────────────
GPIO 18      →    Buzzer
GPIO 34      →    Battery ADC
GPIO 32      →    MPU6050 INT
GPIO 14      →    LED Alert
GPIO 27      →    MODE Button
GPIO 35      →    RESET Button
GPIO 16      →    GPS RX
GPIO 17      →    GPS TX
GPIO 25      →    SIM RX
GPIO 26      →    SIM TX
GPIO 21      →    MPU6050 SDA
GPIO 22      →    MPU6050 SCL
```

## 📦 Cài đặt

### Yêu cầu phần mềm

- **PlatformIO** (khuyến nghị) hoặc Arduino IDE
- **ESP32 Board Support Package**

### Thư viện cần thiết

```ini
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.6
    adafruit/Adafruit Unified Sensor@^1.1.14
    mikalhart/TinyGPSPlus@^1.0.3
    bblanchon/ArduinoJson@^7.2.1
```

### Các bước cài đặt

1. **Clone repository:**

```bash
git clone <repository-url>
cd ESP_ALARM_TRACK
```

2. **Cấu hình:**

   - Mở file `include/config.h`
   - Chỉnh sửa số điện thoại:

   ```cpp
   #define PHONE_NUMBER "+84886966103"  // Số điện thoại nhận SMS
   ```

   - Chỉnh sửa APN của nhà mạng:

   ```cpp
   #define SIM_APN "v-internet"  // Viettel: "v-internet", Vinaphone: "m-wap", Mobifone: "m-i-vinaphone"
   ```

3. **Compile & Upload:**

   ```bash
   pio run -t upload
   ```

4. **Monitor Serial:**
   ```bash
   pio device monitor -b 115200
   ```

## 📁 Cấu trúc dự án

```
ESP_ALARM_TRACK/
├── include/                   # Header files
│   ├── config.h               # Cấu hình pins & thông số
│   ├── types.h                # Định nghĩa types & enums
│   ├── globals.h              # Khai báo biến toàn cục
│   ├── hardware.h             # Hardware functions
│   ├── tasks.h                # FreeRTOS tasks
│   ├── mqtt_handler.h         # MQTT functions
│   ├── sms_handler.h          # SMS functions
│   └── kalman_filter.h        # Kalman filter
├── src/                       # Source files
│   ├── main.cpp               # Entry point
│   ├── hardware.cpp           # Hardware implementation
│   ├── tasks.cpp              # FreeRTOS tasks implementation
│   ├── mqtt_handler.cpp       # MQTT implementation
│   └── sms_handler.cpp        # SMS implementation
├── platformio.ini             # PlatformIO config
└── README.md                  # This file
```

## 🎯 Cách sử dụng

### Khởi động lần đầu

1. Cấp nguồn cho hệ thống
2. Đợi hệ thống khởi tạo (~30s)
3. Nghe 2 tiếng beep ngắn → Hệ thống sẵn sàng

### Kích hoạt chế độ bảo vệ

1. **Nhấn nút MODE** để chuyển sang chế độ "Owner Absent"
2. LED xanh sáng liên tục → Hệ thống ở chế độ `STAGE_WARNING`
3. Nếu có chuyển động mạnh:
   - LED nhấp nháy
   - Còi kêu mỗi 2 giây
   - Chuyển sang `STAGE_ALERT`

### Theo dõi vị trí

- Khi xe di chuyển > 10m, hệ thống tự động chuyển sang `STAGE_TRACKING`
- Gửi SMS vị trí mỗi 30s
- Gửi dữ liệu lên MQTT broker mỗi 30s

### Tắt chế độ bảo vệ

1. **Nhấn nút MODE** để chuyển về "Owner Present"
2. LED tắt, còi tắt
3. Hệ thống chuyển về `STAGE_NONE`

### Lệnh SMS

Gửi SMS đến số SIM trong thiết bị:

- **`GPS`** - Lấy vị trí hiện tại
- **`STOP TRACKING`** - Dừng theo dõi và về chế độ bình thường
- **`STOP SMS`** - Tắt gửi SMS vị trí định kỳ (vẫn gửi MQTT)

## ⚙️ Tùy chỉnh

### Thay đổi ngưỡng phát hiện chuyển động

Trong `include/config.h`:

```cpp
#define STRONG_ACCEL_THRESHOLD 3.6f    // Ngưỡng chuyển động mạnh (m/s²)
#define ACCEL_MIN_DETECT 2.0f          // Ngưỡng chuyển động nhẹ (m/s²)
#define LIGHT_ACCEL_DEADBAND 1.5f      // Vùng chết không phát hiện
```

### Thay đổi khoảng cách cảnh báo

```cpp
#define DISTANCE_THRESHOLD_MAX 10.0f   // Di chuyển > 10m → TRACKING
#define DISTANCE_THRESHOLD_MIN 3.0f    // Di chuyển < 3m → WARNING
```

### Thay đổi thời gian ngủ

```cpp
#define SLEEP_WAKE_INTERVAL_MS (20ULL * 60 * 1000000)  // Deep sleep and light sleep: 20 phút
#define LIGHT_SLEEP_TIMEOUT 130000UL                   // Light sleep: 2 phút 10s
```

### Thay đổi cooldown SMS

Trong `src/sms_handler.cpp`:

```cpp
const unsigned long SMS_COOLDOWN[] = {
    20000UL,   // ALARM: 20 giây
    60000UL,  // MOVEMENT: 1 phút
    60000UL,   // POSITION: 1 phút
    180000UL,  // GPS_LOST: 3 phút
    900000UL,  // LOW_BATTERY: 15 phút
    600000UL  // SYSTEM_ERROR: 10 phút
    1000UL    // EMERGENCY 1s
};
```

## 📊 MQTT Data Format

Dữ liệu được gửi lên topic `gps/tracker/data` với format JSON:

```json
{
  "battery_percent": 85,
  "battery_voltage": 7.8,
  "owner_present": 0,
  "alarm_stage": 2,
  "motion_detected": 1,
  "mqtt_connected": 1,
  "gps_valid": 1,
  "latitude": 21.028511,
  "longitude": 105.804817,
  "strong_motion": 0,
  "low_battery": 0,
  "timestamp": 123456789
}
```

### Giá trị `alarm_stage`:

- `0` = STAGE_NONE
- `1` = STAGE_WARNING
- `2` = STAGE_ALERT
- `3` = STAGE_TRACKING

## 🔍 Debug & Troubleshooting

### Kiểm tra Serial Monitor

```
====ESP32 GPS Tracking System====
[SIM] Initializing...
[MPU] Initialized OK
[GPS] Warming up...
System started
```

### Các thông báo quan trọng

| Thông báo                     | Ý nghĩa                                   |
| ----------------------------- | ----------------------------------------- |
| `[MPU] I2C error`             | Lỗi kết nối MPU6050, kiểm tra dây I2C     |
| `[SIM] No response`           | SIM không phản hồi, kiểm tra nguồn & UART |
| `[GPS] signal lost`           | Mất tín hiệu GPS, di chuyển ra ngoài trời |
| `[MQTT] Connection failed`    | Lỗi kết nối MQTT, kiểm tra APN & mạng     |
| `[SLEEP] Going to deep sleep` | Vào chế độ ngủ sâu                        |



## 🔋 Tiêu thụ điện năng

| Chế độ          | Dòng tiêu thụ | Thời gian                |
| --------------- | ------------- | ------------------------ |
| Deep Sleep      | ~5mA          | Không giới hạn           |
| Light Sleep     | ~15mA         | Khi không có chuyển động |
| Active (GPS ON) | ~200mA        | Khi tracking             |
| Peak (SIM TX)   | ~400mA        | Khi gửi SMS/MQTT         |

## Kết quả thực hiện: 
https://www.youtube.com/watch?v=mN-s30zn16E


