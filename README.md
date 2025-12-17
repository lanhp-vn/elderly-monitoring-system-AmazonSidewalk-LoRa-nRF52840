# Advanced Elderly Care System using Amazon Sidewalk over LoRa

A wearable health monitoring system for elderly care that leverages **Amazon Sidewalk over LoRa** for long-range, low-power connectivity. The system provides continuous vital signs monitoring, automatic fall detection, GPS location tracking, and real-time caregiver alerts—all without requiring WiFi infrastructure or monthly subscription fees.

Demo Video: https://www.youtube.com/watch?v=Cs6WsbXar5s

> **MECPS 2025 Capstone Project** — UC Irvine  
> *Hoang Lan Pham, Royston Pinto, Rudrashis Gorai*  
> Advisor: Prof. Hung Cao | Graduate Mentor: Mohamed Benomar El Kati

---

## 🎯 Problem Statement

Falls are the **leading cause of injury-related death** among adults 65+, claiming approximately 38,000 lives annually in the US alone. Traditional wearable health devices rely on WiFi or Bluetooth, which offer limited range and create dangerous monitoring gaps when elderly users venture beyond their homes.

**This project solves these limitations by:**
- Extending monitoring coverage to **500-800m per Sidewalk bridge** (up to 10km line-of-sight)
- Eliminating infrastructure costs through Amazon's free community mesh network
- Providing **automatic emergency alerts** even when users are outside WiFi range

---

## 🏗️ System Architecture

```
┌─────────────────┐     LoRa 915MHz      ┌──────────────────┐     Internet      ┌─────────────────┐
│   Wearable      │ ─────────────────►   │  Echo/Ring       │ ───────────────►  │   AWS Cloud     │
│   Endpoint      │                      │  Sidewalk Bridge │                   │   (IoT Core)    │
│                 │                      │                  │                   │                 │
│  • Heart Rate   │                      │  • Routes data   │                   │  • Lambda       │
│  • SpO2         │◄──── BLE ────────►   │    to cloud      │                   │  • DynamoDB     │
│  • Fall Detect  │   (registration)     │  • No config     │                   │  • API Gateway  │
│  • GPS Location │                      │    needed        │                   │                 │
└─────────────────┘                      └──────────────────┘                   └────────┬────────┘
                                                                                        │
                                                                                        ▼
                                                                               ┌─────────────────┐
                                                                               │   Dashboard     │
                                                                               │   (React/Vite)  │
                                                                               │                 │
                                                                               │  • Real-time    │
                                                                               │    vitals       │
                                                                               │  • Fall alerts  │
                                                                               │  • Location map │
                                                                               └─────────────────┘
```

---

## ✨ Key Features

| Feature | Description |
|---------|-------------|
| **Vital Signs Monitoring** | Continuous heart rate (BPM) and SpO2 measurement via MAX30102 PPG sensor, sampled every 5 seconds |
| **3-Phase Fall Detection** | Detects free-fall → impact → post-fall stillness using MPU6050 IMU at 50Hz sampling |
| **GPS Location Tracking** | Real-time coordinates via NEO-6M module for locating wandering users |
| **Panic Button** | One-press help request with guaranteed delivery via retry logic |
| **Long-Range Connectivity** | Amazon Sidewalk over LoRa provides 500-800m urban range per bridge |
| **Zero Infrastructure Cost** | Uses existing Echo/Ring devices as gateways—no subscription fees |
| **Caregiver Dashboard** | Web-based real-time monitoring with alerts, charts, and interactive maps |

---

## 🔧 Hardware Components

### Bill of Materials

| Component | Model | Purpose | Interface |
|-----------|-------|---------|-----------|
| Microcontroller | Nordic nRF52840 DK | Main processor, BLE radio | — |
| LoRa Transceiver | Semtech SX1262 EVB | 915 MHz Sub-GHz communication | SPI |
| Heart Rate Sensor | DFRobot MAX30102 v2.0 | Heart rate, SpO2, temperature | I2C (0x57) |
| IMU | MPU6050 | 6-axis accelerometer/gyroscope | I2C (0x68) |
| GPS Module | NEO-6M | Location tracking | UART (9600 baud) |
| Sidewalk Gateway | Amazon Echo Dot 5th Gen | Bridge to AWS cloud | — |

### Pin Configuration

```
nRF52840 DK Pin Mapping
========================

I2C Bus (400kHz):
  P0.26 (SDA) ──── MAX30102 + MPU6050
  P0.27 (SCL) ──── MAX30102 + MPU6050

UART (GPS):
  P0.08 (RX)  ──── NEO-6M TX
  P0.06 (TX)  ──── NEO-6M RX

SPI (LoRa) - Via Arduino Headers:
  P1.13 (MOSI) ─── SX1262
  P1.14 (MISO) ─── SX1262
  P1.15 (SCK)  ─── SX1262

Buttons:
  P0.11 (BTN1) ─── Help/Panic Button

Status LEDs:
  P0.13 (LED1) ─── Sidewalk Connected
  P0.14 (LED2) ─── Time Synchronized
  P0.15 (LED3) ─── Device Registered
  P0.16 (LED4) ─── System Working
```

---

## 📁 Repository Structure

```
.
├── samples/
│   ├── sid_end_device/              # Main Sidewalk-enabled application
│   │   ├── src/sensors/
│   │   │   ├── app.c                # Main application entry
│   │   │   ├── sensor_manager.c     # Sensor coordination
│   │   │   ├── mpu6050_handler.c    # IMU + fall detection
│   │   │   ├── sen0344_handler.c    # Heart rate sensor
│   │   │   ├── gps_handler.c        # GPS/GNSS handler
│   │   │   └── critical_msg_buffer.c # Emergency message queue
│   │   ├── include/sensors/
│   │   ├── overlay-sensors.conf     # Sensor build configuration
│   │   └── boards/
│   │       └── nrf52840dk_nrf52840.overlay
│   │
│   └── sensors_combined/            # Standalone sensor test (no Sidewalk)
│
├── cloud/
│   ├── lambda/                      # AWS Lambda functions
│   │   ├── uplink/                  # Process incoming sensor data
│   │   ├── downlink/                # Send commands to device
│   │   └── db_handler/              # Database operations
│   ├── cloudformation/              # Infrastructure as Code
│   └── dashboard/                   # React frontend
│       ├── src/
│       ├── package.json
│       └── vite.config.js
│
├── docs/
│   ├── MECPS25_Capstone_Project_Report.pdf
│   └── schematics/
│
└── tools/
    └── provision.py                 # Device provisioning script
```

---

## 🚀 Getting Started

### Prerequisites

- **nRF Connect SDK v2.7.0** with Amazon Sidewalk SDK v2.7.0
- **VS Code** with nRF Connect Extension Pack
- **AWS Account** with CLI configured
- **Amazon Echo Dot** (5th Gen) or Ring device for Sidewalk bridge
- **Python 3.6+** for provisioning scripts

### Step 1: Clone and Setup SDK

```bash
# Install nRF Connect SDK (if not already)
# Follow: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.7.0/nrf/installation.html

# Navigate to Sidewalk samples
cd ncs/v2.7.0/sidewalk
```

### Step 2: AWS Device Provisioning

```bash
# Create device profile
aws iotwireless create-device-profile --name sidewalk_profile --sidewalk {}

# Create wireless device
aws iotwireless create-wireless-device \
  --type "Sidewalk" \
  --name "eldercare_device" \
  --destination-name "SidewalkDestination" \
  --sidewalk DeviceProfileId="<your-device-profile-id>"

# Export credentials
aws iotwireless get-device-profile --id "<device-profile-id>" > device_profile.json
aws iotwireless get-wireless-device \
  --identifier-type WirelessDeviceId \
  --identifier "<wireless-device-id>" > wireless_device.json

# Generate provisioning binary
python3 provision.py nordic aws \
  --output_bin mfg.bin \
  --wireless_device_json wireless_device.json \
  --device_profile_json device_profile.json \
  --addr 0xFF000
```

### Step 3: Build and Flash Firmware

```bash
# Build with sensors overlay
west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
  -DOVERLAY_CONFIG="overlay-sensors.conf"

# Flash both application and credentials
# Use nRF Connect Programmer to flash:
#   - nordic_aws_nrf52840.hex (credentials)
#   - build/zephyr/merged.hex (application)
```

### Step 4: Deploy Cloud Infrastructure

```bash
cd cloud/

# Deploy via CloudFormation
aws cloudformation deploy \
  --template-file cloudformation/template.yaml \
  --stack-name eldercare-stack \
  --capabilities CAPABILITY_IAM

# Build and deploy dashboard
cd dashboard
npm install
npm run build
aws s3 sync dist/ s3://<your-bucket-name>
```

### Step 5: View Debug Output

Since UART is reserved for GPS, use **Segger RTT** for debug logs:

```bash
# Option 1: VS Code nRF Connect extension RTT console
# Option 2: JLink RTT Viewer
JLinkRTTViewer
```

---

## 📊 Message Protocol

All sensor data is transmitted as compact JSON payloads (max 200 bytes):

```json
{
  "t": "R",        // Type: R=Regular, E=Emergency, H=Help
  "d": 1,          // Device ID
  "f": "N",        // Fall state: N=Normal, F=Free-fall, I=Impact
  "b": 72,         // Heart rate (BPM)
  "s": 98,         // SpO2 (%)
  "T": 3650,       // Temperature (centidegrees: 36.50°C)
  "la": 3364585,   // Latitude × 100000
  "lo": -11784294, // Longitude × 100000
  "B": 100,        // Battery (%)
  "ts": 18000      // Timestamp (seconds since boot)
}
```

### Message Types

| Type | Trigger | Delivery | Retry |
|------|---------|----------|-------|
| **Regular (R)** | Timer (every 5 min) | Fire-and-forget | None |
| **Emergency (E)** | Fall detection, abnormal vitals | Guaranteed | Up to 3× |
| **Help (H)** | Panic button press | Guaranteed | Up to 3× |

---

## 🧪 Testing

### Sensor Validation (Without Sidewalk)

Use the standalone sensor sample to verify hardware:

```bash
west build -b nrf52840dk/nrf52840 samples/sensors_combined
west flash
```

Expected output:
```
[00:00:05.000] <inf> main: IMU: accel=1.02g, gyro=0.05rad/s
[00:00:05.000] <inf> main: Health: HR=72 BPM, SpO2=98%, Temp=36.50 C
[00:00:05.000] <inf> main: GPS: 33.645850, -117.842940 (sats=8)
```

### Fall Detection Test

Drop the board to trigger the 3-phase algorithm:
1. **Free-fall** detected when acceleration < 0.35g
2. **Impact** detected when acceleration > 2.4g AND rotation > 240°/s
3. **Fall confirmed** after 3 seconds of stillness
