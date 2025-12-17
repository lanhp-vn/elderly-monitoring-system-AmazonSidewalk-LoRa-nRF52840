# Sidewalk End Device Sample

This sample demonstrates Amazon Sidewalk connectivity on Nordic nRF52840 DK with multiple application variants. The primary focus is the **Sensors variant** - a complete health monitoring system with fall detection, vital signs monitoring, and GPS tracking sent via Amazon Sidewalk to AWS IoT.

**Demo Video**: [https://www.youtube.com/watch?v=Cs6WsbXar5s](https://www.youtube.com/watch?v=Cs6WsbXar5s)

## Sample Variants

| Variant | Overlay Config | Purpose |
|---------|----------------|---------|
| **Sensors** | `overlay-sensors.conf` | Health monitoring with GPS, IMU, heart rate (production use case) |
| Hello | `overlay-hello.conf` | Basic Sidewalk connectivity test |
| Demo | `overlay-demo.conf` | Sensor monitoring demo |
| DUT | `overlay-dut.conf` | Device Under Test with CLI |

> **Note**: Hello and Demo variants are provided for initial Sidewalk connectivity testing. The Sensors variant is the complete implementation.

---

## Sensors Variant - Health Monitoring System

The sensors variant implements a wearable health monitoring system that:

- Detects falls using a 3-phase algorithm (free-fall → impact → stillness)
- Monitors vital signs (heart rate, SpO2, temperature)
- Tracks GPS location
- Sends emergency alerts via Amazon Sidewalk to AWS IoT
- Automatically switches from BLE to BLE+LoRa for extended range

### Hardware Components

| Component | Model | Interface | Function |
|-----------|-------|-----------|----------|
| Development Kit | nRF52840 DK (PCA10056) | - | Main MCU |
| IMU | DFRobot MPU6050 | I2C (0x68) | Fall detection |
| Heart Rate Sensor | DFRobot MAX30102 v2 (SEN0344) | I2C (0x57) | HR, SpO2, Temperature |
| GPS Module | NEO-6M | UART1 (9600 baud) | Location tracking |
| LoRa Transceiver | Semtech SX1262 | SPI2 | Sub-GHz communication |

### Wiring Diagram

```
nRF52840 DK                      External Hardware
============                     =================

I2C0 Bus (400kHz):
  P0.26 (SDA) ─────────────────── MPU6050 SDA ─── SEN0344 SDA
  P0.27 (SCL) ─────────────────── MPU6050 SCL ─── SEN0344 SCL

UART1 (9600 baud):
  P1.01 (RX) ──────────────────── NEO-6M TX
  P1.02 (TX) ──────────────────── NEO-6M RX

SPI2 (8MHz) - Semtech SX1262:
  P1.15 (SCK)  ────────────────── SX1262 SCK
  P1.13 (MOSI) ────────────────── SX1262 MOSI
  P1.14 (MISO) ────────────────── SX1262 MISO
  P1.08 (CS)   ────────────────── SX1262 NSS
  P0.03 (RST)  ────────────────── SX1262 RESET
  P1.04 (BUSY) ────────────────── SX1262 BUSY
  P1.06 (DIO1) ────────────────── SX1262 DIO1
  P1.10 (ANT)  ────────────────── SX1262 ANT_EN

Power:
  VDD (3.3V) ──────────────────── All sensors VCC
  GND ─────────────────────────── All sensors GND
```

### Message Types

The sensors variant sends three types of JSON messages:

#### Regular Message (Type: R)
Sent periodically (configurable, default 20 seconds for testing):
```json
{"t":"R","d":1,"f":"N","b":72,"s":98,"T":3650,"la":3364585,"lo":-11195730,"B":100,"ts":12345}
```

#### Emergency Message (Type: E)
Sent automatically on fall detection or abnormal vital signs:
```json
{"t":"E","d":1,"f":"I","b":85,"s":97,"T":3650,"la":3364585,"lo":-11195730,"B":100,"ts":12346}
```

#### Help Message (Type: H)
Sent when user presses the panic button (BTN1):
```json
{"t":"H","d":1,"f":"N","b":72,"s":98,"T":3650,"la":3364585,"lo":-11195730,"B":100,"ts":12347}
```

#### Payload Field Reference

| Field | Description | Example |
|-------|-------------|---------|
| `t` | Message type: R(egular), E(mergency), H(elp) | "E" |
| `d` | Device ID | 1 |
| `f` | Fall state: N(ormal), F(ree-fall), I(mpact) | "I" |
| `b` | Heart rate in BPM (0 = no finger) | 72 |
| `s` | SpO2 percentage (0 = no finger) | 98 |
| `T` | Temperature in centidegrees | 3650 |
| `la` | Latitude × 100000 (-1 = no GPS fix) | 3364585 |
| `lo` | Longitude × 100000 (-1 = no GPS fix) | -11195730 |
| `B` | Battery percentage | 100 |
| `ts` | Timestamp in seconds | 12345 |

---

## Building

### Prerequisites

1. **nRF Connect SDK v2.7.0** with West build tool
2. **AWS Account** with Sidewalk enabled
3. **Amazon Sidewalk Gateway** (Echo device with Sidewalk support)
4. **Device provisioned** with AWS IoT Wireless

### Build Commands

```bash
# Navigate to sidewalk repository
cd ncs/v2.7.0/sidewalk

# Build sensors variant (with SubGHz/LoRa support)
west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
    -DOVERLAY_CONFIG="overlay-sensors.conf"

# Flash to device
west flash
```

### Build Variants

```bash
# Hello variant (basic connectivity test)
west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
    -DOVERLAY_CONFIG="overlay-hello.conf"

# Demo variant
west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
    -DOVERLAY_CONFIG="overlay-demo.conf"

# Sensors variant with release optimization
west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
    -DOVERLAY_CONFIG="overlay-sensors.conf" \
    -DFILE_SUFFIX=release
```

---

## Device Provisioning

Each user must provision their own device with AWS credentials. The provisioning files contain private keys and are excluded from version control.

### Prerequisites

1. **AWS CLI** configured with appropriate permissions
2. **Python 3** with pip
3. **nRF Connect for Desktop** with Programmer app installed
4. Complete AWS Sidewalk setup: [AWS Sidewalk Getting Started Guide](https://docs.aws.amazon.com/iot-wireless/latest/developerguide/sidewalk-getting-started.html)

### Security Notice

**Never commit these files to version control**:
- `device_profile.json` - AWS device profile configuration
- `wireless_device.json` - Device certificates and **private keys**
- `mfg.bin` - Manufacturing provisioning binary
- `nordic_aws_*.hex` - Pre-provisioned device firmware

These files are excluded in `.gitignore`.

### Step 1: Create Device Profile

```bash
# Create a new Sidewalk device profile
aws iotwireless create-device-profile --name "my_sidewalk_profile" --sidewalk {}
```

Example output:
```json
{
    "Arn": "arn:aws:iotwireless:us-east-1:123456789012:DeviceProfile/436565f1-4f14-43e7-bd7c-8402bfce9245",
    "Id": "436565f1-4f14-43e7-bd7c-8402bfce9245"
}
```

**Save the `Id` value** - you'll need it in the next step.

### Step 2: Create Destination (if not already created)

Follow the [AWS Sidewalk destination setup](https://docs.aws.amazon.com/iot-wireless/latest/developerguide/sidewalk-getting-started.html) to create a destination with an IoT rule for processing messages.

### Step 3: Register Wireless Device

```bash
# Replace <DEVICE_PROFILE_ID> with the Id from Step 1
# Replace <DESTINATION_NAME> with your destination name
aws iotwireless create-wireless-device \
    --type "Sidewalk" \
    --name "my_device_1" \
    --destination-name "<DESTINATION_NAME>" \
    --sidewalk DeviceProfileId="<DEVICE_PROFILE_ID>"
```

Example output:
```json
{
    "Arn": "arn:aws:iotwireless:us-east-1:123456789012:WirelessDevice/3c9596c4-94ce-45fb-bca3-6cf5a26c95f1",
    "Id": "3c9596c4-94ce-45fb-bca3-6cf5a26c95f1"
}
```

**Save the device `Id` value**.

### Step 4: Export Configuration Files

```bash
cd tools/provision

# Export device profile (replace with your Device Profile ID)
aws iotwireless get-device-profile \
    --id "<DEVICE_PROFILE_ID>" > device_profile.json

# Export wireless device credentials (replace with your Wireless Device ID)
aws iotwireless get-wireless-device \
    --identifier-type WirelessDeviceId \
    --identifier "<WIRELESS_DEVICE_ID>" > wireless_device.json
```

### Step 5: Generate Manufacturing Binary

```bash
cd tools/provision

# Install dependencies
pip install -r requirements.txt

# Generate mfg.bin
python provision.py nordic aws \
    --output_bin mfg.bin \
    --wireless_device_json wireless_device.json \
    --device_profile_json device_profile.json \
    --addr 0xFF000
```

This generates `mfg.bin` containing the device credentials.

### Step 6: Build the Application

```bash
cd ../..  # Return to sidewalk root

# Build sensors variant
west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
    -DOVERLAY_CONFIG="overlay-sensors.conf"
```

The build output is located at `build/zephyr/merged.hex`.

### Step 7: Flash Device Using nRF Connect Programmer

1. Open **nRF Connect for Desktop**
2. Launch the **Programmer** app
3. Select your nRF52840 DK device
4. Add files to flash:
   - Click **Add file** → Select `tools/provision/mfg.bin` (set address to `0xFF000`)
   - Click **Add file** → Select `build/zephyr/merged.hex`
5. Click **Erase & Write** to flash both files

Alternatively, use command line:
```bash
# Flash manufacturing data
nrfjprog --program tools/provision/mfg.bin --sectorerase

# Flash application
west flash
```

### Step 8: Verify with MQTT Test Client

1. Go to [AWS IoT Console](https://console.aws.amazon.com/iot/)
2. Navigate to **Test** → **MQTT test client**
3. Subscribe to your destination's topic (e.g., `sidewalk/#` or your specific topic)
4. Power on your device and observe incoming messages

For detailed MQTT setup, see: [Nordic Sidewalk Product Setup](https://docs.nordicsemi.com/bundle/sidewalk_2.7.0/page/setting_up_sidewalk_environment/setting_up_sidewalk_product.html)

### Provisioning Multiple Devices

Repeat Steps 3-7 for each device, using a unique device name:

```bash
# Device 2
aws iotwireless create-wireless-device \
    --type "Sidewalk" \
    --name "my_device_2" \
    --destination-name "<DESTINATION_NAME>" \
    --sidewalk DeviceProfileId="<DEVICE_PROFILE_ID>"
```

### Reference Documentation

- [AWS Sidewalk Getting Started](https://docs.aws.amazon.com/iot-wireless/latest/developerguide/sidewalk-getting-started.html)
- [Nordic Sidewalk Provisioning](https://docs.nordicsemi.com/bundle/sidewalk_2.7.0/page/setting_up_sidewalk_environment/setting_up_sidewalk_product.html)
- [Amazon Sidewalk Documentation](https://docs.sidewalk.amazon)

---

## Configuration Options

### Sensors Variant Kconfig Options

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_SENSORS_DEVICE_ID` | 1 | Unique device identifier |
| `CONFIG_SENSORS_REGULAR_INTERVAL_MS` | 300000 | Regular report interval (5 min) |
| `CONFIG_SENSORS_AUTO_LORA_SWITCH` | y | Auto-switch to BLE+LoRa |
| `CONFIG_SENSORS_SPO2_THRESHOLD` | 90 | SpO2 emergency threshold (%) |
| `CONFIG_SENSORS_BPM_LOW_THRESHOLD` | 40 | Low heart rate threshold |
| `CONFIG_SENSORS_BPM_HIGH_THRESHOLD` | 180 | High heart rate threshold |
| `CONFIG_SENSORS_CRITICAL_BUFFER_SIZE` | 10 | Emergency message buffer size |
| `CONFIG_SENSORS_ENABLE_SIMULATED_GPS` | y | Use simulated GPS (for indoor testing) |

Override in `overlay-sensors.conf`:
```kconfig
# Faster reporting for testing (20 seconds)
CONFIG_SENSORS_REGULAR_INTERVAL_MS=20000

# Set device ID
CONFIG_SENSORS_DEVICE_ID=1

# Disable simulated GPS for real hardware
CONFIG_SENSORS_ENABLE_SIMULATED_GPS=n
```

---

## Button Functions

| Button | Short Press | Long Press |
|--------|-------------|------------|
| BTN1 | Send Help message | Enter Nordic DFU mode |
| BTN2 | Request connection | Factory reset |
| BTN3 | Toggle link (BLE ↔ BLE+LoRa) | - |
| BTN4 | - | - |

---

## LED Indicators

| LED | Meaning |
|-----|---------|
| LED1 | Sidewalk connected |
| LED2 | Time synchronized |
| LED3 | Device registered |
| LED4 | Application working |

---

## System Architecture

### Message Flow

```
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   Sensor Layer   │    │  Sidewalk Stack  │    │    AWS Cloud     │
├──────────────────┤    ├──────────────────┤    ├──────────────────┤
│ MPU6050 (50Hz)   │───▶│                  │    │                  │
│ ├─ Fall Detection│    │ BLE (Link 1)     │───▶│ AWS IoT Core     │
│ └─ Accel/Gyro    │    │        or        │    │      │           │
│                  │    │ LoRa (Link 3)    │    │      ▼           │
│ SEN0344 (5s)     │───▶│                  │    │ Lambda/IoT Rule  │
│ ├─ Heart Rate    │    │ Critical Buffer  │    │      │           │
│ ├─ SpO2          │    │ ├─ Emergency (E) │    │      ▼           │
│ └─ Temperature   │    │ ├─ Help (H)      │    │ Your Backend     │
│                  │    │ └─ Retries       │    │                  │
│ NEO-6M GPS (5s)  │───▶│                  │    │                  │
│ └─ Lat/Lon/Alt   │    │ Regular (R)      │    │                  │
└──────────────────┘    └──────────────────┘    └──────────────────┘
```

### Critical Message Buffer

Emergency and Help messages use a dedicated ring buffer with:
- **Guaranteed delivery**: Retries up to 3 times on failure
- **TTL**: Messages expire after 5 minutes
- **Priority**: Sent immediately when Sidewalk is ready
- **Overflow**: Oldest messages evicted when buffer is full

### Automatic LoRa Switch

The device automatically enables LoRa (in addition to BLE) when:
1. Device is registered with AWS
2. Time is synchronized
3. BLE link is up

This provides extended range (up to 1 mile) for emergency messages.

---

## Troubleshooting

### Sidewalk Not Connecting

```
[00:00:10.000] <inf> sidewalk: Device not registered
```

**Solutions**:
- Ensure device is provisioned with valid `mfg.hex`
- Verify Amazon Sidewalk gateway (Echo) is nearby and online
- Check AWS IoT Wireless device status in console

### Sensors Not Initializing

```
[00:00:01.000] <err> sensor_manager: MPU6050 handler init failed: -19
```

**Solutions**:
- Verify I2C wiring (SDA: P0.26, SCL: P0.27)
- Check sensor power supply (3.3V)
- Ensure sensors are on the same I2C bus

### Messages Not Reaching AWS

**Solutions**:
- Check Sidewalk link status LEDs
- Verify AWS IoT rule and destination are configured
- Monitor CloudWatch logs for the IoT rule

### Fall Detection Not Triggering

The 3-phase algorithm requires:
1. Free-fall (weightlessness <0.35g)
2. Impact (>2.4g acceleration + >240°/s rotation)
3. Post-fall stillness (3 seconds near 1g)

**Test procedure**: Drop the device from ~1m height onto a soft surface, then leave it still.

---

## Project Structure

```
samples/sid_end_device/
├── CMakeLists.txt
├── Kconfig                      # Sample-wide options
├── Kconfig.sensors              # Sensors variant options
├── prj.conf                     # Debug build config
├── prj_release.conf             # Release build config
├── overlay-sensors.conf         # Sensors variant config
├── overlay-hello.conf           # Hello variant config
├── overlay-demo.conf            # Demo variant config
├── boards/
│   └── nrf52840dk_nrf52840.overlay  # Pin mappings & sensors
├── include/
│   └── sensors/
│       ├── data_types.h         # Sensor data structures
│       ├── sensor_manager.h     # Unified sensor API
│       ├── gps_handler.h
│       ├── mpu6050_handler.h
│       ├── sen0344_handler.h
│       └── critical_msg_buffer.h
└── src/
    ├── main.c                   # Zephyr entry point
    ├── sidewalk.c               # Sidewalk state machine
    └── sensors/
        ├── app.c                # Sensors variant application
        ├── sensor_manager.c     # Sensor coordination
        ├── gps_handler.c        # GPS NMEA handling
        ├── mpu6050_handler.c    # IMU + fall detection
        ├── sen0344_handler.c    # Heart rate sensor
        └── critical_msg_buffer.c # Emergency message queue
```

---

## Testing Without Sensors

For initial Sidewalk connectivity testing without hardware sensors:

1. **Enable simulated GPS** (default):
   ```kconfig
   CONFIG_SENSORS_ENABLE_SIMULATED_GPS=y
   ```

2. **Use Hello variant** for basic message testing:
   ```bash
   west build -b nrf52840dk/nrf52840 samples/sid_end_device -- \
       -DOVERLAY_CONFIG="overlay-hello.conf"
   ```

3. **Test sensor hardware separately** with the standalone sample:
   ```bash
   west build -b nrf52840dk/nrf52840 samples/sensors_combined
   ```

---

## Related Resources

- [Standalone Sensors Sample](../sensors_combined/README.md) - Test sensors without Sidewalk
- [Amazon Sidewalk Documentation](https://docs.sidewalk.amazon)
- [Nordic Sidewalk Guide](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/amazon_sidewalk/index.html)
- [AWS IoT Wireless](https://docs.aws.amazon.com/iot-wireless/)

---

## License

Nordic code: LicenseRef-Nordic-5-Clause (see LICENSE.txt)
Amazon Sidewalk libraries: Amazon Sidewalk Program license (proprietary)
