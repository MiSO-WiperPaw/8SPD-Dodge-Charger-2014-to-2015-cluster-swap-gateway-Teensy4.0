# Dodge Charger 2015+ Cluster Retrofit Gateway

A Teensy 4.0-based CAN-C bus gateway/translator that retrofits 2015+ Dodge Charger instrument clusters into 2011–2014 Chargers equipped with the 8-speed automatic transmission and electronic shifter. The gateway sits between the vehicle's CAN-C bus and the newer cluster, translating and adding the features the cluster expects that the older platform doesn't natively provide.

Two firmware variants are included in this repository:

| Variant | File | Description |
|---|---|---|
| **NOGPS** | `dodge_2015_Cluster_gateway_T4__V1_NOGPS.ino` | Core gateway functionality only. No GPS, no compass, no time sync. |
| **GPS** | `dodge_2015_Cluster_gateway_T4_GPS.ino` | Everything in NOGPS, plus GPS-synced clock and compass heading display. |

Pick whichever matches your hardware. If you haven't installed a GPS/compass module, use NOGPS — it's simpler and has one less thing that can go wrong.

---

## Features

Both variants include:

- **Sport mode display** — shows sport mode indicator on the cluster when enabled via the shifter/paddle setup.
- **Autostick / manual mode display** — shows manual gear selection (1–8) when in autostick mode.
- **Actual gear display** — cluster shows the real selected/engaged gear.
- **Region/unit configuration** — sets pressure, speed, distance, fuel consumption, and temperature units on the cluster at boot and after every wake.
- **Steering wheel button remapping** — translates the older steering wheel's button CAN frames into the format the new cluster expects, including proper press/hold/release behavior for the OK button.
- **Low power deep sleep** — the gateway sleeps at ~9–14 mA when the vehicle is off and wakes instantly on CAN bus activity, so it does not drain the battery overnight.

The **GPS variant** additionally includes:

- **GPS-synced clock** — sets the cluster's displayed time from GPS, corrected to Asia/Dubai (UTC+4, no DST).
- **Compass heading display** — 8-point compass heading (N/NE/E/SE/S/SW/W/NW) from an onboard magnetometer, sent to the cluster.
- **GPS-based compass auto-calibration** — corrects for mounting angle and magnetic interference automatically while driving, using GPS course-over-ground as ground truth. No manual calibration procedure needed.
- **Ignition-aware transmission** — GPS/compass CAN frames are only sent while the ignition is genuinely on, so the gateway can't accidentally keep the cluster (and therefore the car) awake after the key is off.

---

## Hardware

### Required (both variants)

- Teensy 4.0
- 2x VP230 (or compatible 3.3V) CAN bus transceivers
- 12V → 5V DC-DC converter, automotive-grade, stable output
- TVS diodes on both CAN transceivers (strongly recommended for any permanent install)

### Additional for GPS variant

- GPS module with M10050 chipset (e.g. Beitian BE-220), NMEA-0183 capable, baud rate auto-detected at boot
- QMC5883L magnetometer/compass module (I²C, address `0x0D`)

### Connection Map

```
Teensy          VP230_Cluster (CAN1)
GPIO_0    <-->  TX
GPIO_1    <-->  RX

Teensy          VP230_Vehicle (CAN2)
GPIO_22   <-->  TX
GPIO_23   <-->  RX
GPIO_4    <-->  RX  (parallel jumper — deep sleep wake pin, see below)
```

**GPS variant only:**

```
Teensy          GPS Module
GPIO_28   <-->  RX  (Teensy TX -> GPS RX, config commands)
GPIO_29   <-->  RX  (Teensy RX <- GPS TX, NMEA stream)
3.3V/5V   <-->  VCC
GND       <-->  GND

Teensy          QMC5883L Compass
GPIO_18   <-->  SDA
GPIO_19   <-->  SCL
3.3V      <-->  VCC
GND       <-->  GND
```

### Hardware Notes

- **GPIO_4 wake pin**: wired in parallel with GPIO_23 (same node, short jumper wire). FlexCAN owns GPIO_23 via its pad mux, so it can't double as a wake-from-sleep GPIO interrupt pin. GPIO_4 is a dedicated free pin used purely to detect a CAN dominant bit and wake the Teensy from deep sleep.
- The vehicle bus (CAN2 / GPIO_22-23) is used as the sleep wake source because the vehicle ECU reliably produces CAN traffic the moment the ignition is turned on.
- QMC5883L modules typically have onboard I²C pull-ups — no external resistors generally needed.

---

## Software Requirements

Install via the Arduino Library Manager:

- **FlexCAN_T4** — CAN bus driver for Teensy 4.x
- **Watchdog_t4** — hardware watchdog support
- **Snooze** — deep sleep / low power support for Teensy 4.x

GPS variant additionally requires:

- **TinyGPS++** — NMEA sentence parsing

Make sure you're on a recent Teensyduino release (1.5x+) for full Snooze and `set_arm_clock` compatibility.

---

## Configuration

### Unit Settings (both variants)

Set near the top of the sketch:

```cpp
const uint8_t pressure_unit = 0x02;  // 0=kPa 1=bar 2=PSI
const uint8_t speed_unit    = 0x00;  // 0=km/h 1=MPH
const uint8_t range_unit    = 0x00;  // 0=km 1=miles
const uint8_t fuelCons_unit = 0x01;  // 0=km/L 1=L/100km 2=MPG
const uint8_t temp_unit     = 0x00;  // 0=C 1=F
```

### Timezone (GPS variant only)

```cpp
const int32_t UTC_OFFSET_SECONDS = 4 * 3600L;  // Asia/Dubai, UTC+4, no DST
```

Change this if installing outside the UAE. Note this is a fixed offset with no daylight saving logic — adjust if your region observes DST.

### GPS Baud Rate (GPS variant only)

The sketch auto-detects the GPS module's baud rate at boot by trying common rates in sequence — 115200 first, since that's the M10050 chip's documented default, followed by 9600, 38400, 4800, 19200, and 57600 as fallbacks. On some M10050 modules, periodic NMEA sentence output is disabled by default even though the protocol itself is correctly configured for NMEA — the sketch automatically sends the necessary UBX configuration commands (`CFG-PRT` to force NMEA mode, `CFG-MSG` to enable the RMC/GGA sentence output rates, and `CFG-CFG` to persist the change) on every boot to handle this. No manual GPS configuration should be needed.

---

## Required Vehicle-Side Configuration

- **AlfaOBD** is required to configure the donor vehicle for compatibility.
- Shifter type must be set to **"MS7S"** with sport mode enabled via AlfaOBD.
- Paddle shifters are not required but strongly recommended — install and enable via AlfaOBD if available.
- Older firmware and pursuit-spec clusters do **not** support sport mode or actual gear display, regardless of gateway configuration.
- The **OK** steering wheel button is currently mapped to the ACC cruise control button ID. If your vehicle has ACC equipped, consider remapping this CAN ID to avoid conflicts. This will be resolved by a future steering wheel swap mod.

---

## CAN Frame Reference

For reference, the GPS variant transmits the following gateway-originated frames to the cluster (only while ignition is on):

| CAN ID | Length | Purpose | Rate |
|---|---|---|---|
| `0x350` | 8 | Time (seconds, minutes, hours, year, month, day, bus ID `0x39`) | 1 Hz |
| `0x358` | 8 | Compass heading octet in byte 0 (`0x00`–`0x07`, or `0x0F` = offline) | 2 Hz |

The corresponding frames originating from the vehicle bus on these same IDs are intentionally blocked from reaching the cluster, since the gateway supplies its own.

---

## Low Power / Sleep Behavior

The gateway uses the Teensy 4.0's `Snooze` deep sleep mode to minimize parasitic battery draw while the vehicle is off:

- **Active**: ~100 mA @ 5V (600 MHz, full CAN gateway operation)
- **Asleep**: ~9–14 mA @ 5V (PLLs and most peripherals gated)
- **Wake source**: any CAN dominant bit on the vehicle bus (GPIO_4), restoring full operation within milliseconds
- **Sleep trigger**: both CAN buses idle for 20 seconds, with a 40-second startup grace period after every boot/wake to let the bus settle first

Deep sleep on the Teensy 4.0 wakes via a full chip reset — `setup()` reruns completely on every wake. This is by design; no state needs to persist across sleep except the compass calibration offset, which is stored separately (see below) in battery-backed registers that survive reset.

In the GPS variant, the gateway also tracks ignition state independently via CAN ID `0x170` (present only while the ignition is on). This prevents a feedback loop where the gateway's own periodic time/compass frames could otherwise keep the cluster — and therefore the car — from ever going to sleep.

---

## Compass Calibration (GPS variant only)

No manual calibration procedure is required. While driving in a straight line above 15 km/h, the gateway compares the GPS course-over-ground against the raw compass heading and gradually computes a correction offset for mounting angle and magnetic interference. This offset is:

- Updated continuously in small increments as you drive (16-sample rolling average per update)
- Stored in the iMXRT1062's battery-backed SNVS registers, surviving deep sleep, watchdog resets, and power cycles
- Applied automatically to every heading reading sent to the cluster

Expect the heading to settle into accuracy within the first few minutes of normal driving after install.

---

## Known Limitations

- Older firmware/pursuit clusters do not support sport mode or actual gear display.
- The OK button currently shares a CAN ID with ACC cruise control (see Vehicle-Side Configuration above).
- GPS time sync only requires valid time/date from the receiver, not a full position fix, so it typically syncs faster than position would suggest. In poor sky-visibility locations (indoors, parking garages, etc.) the clock will not update until at least one satellite's time signal is decoded, though it will continue keeping time from the Teensy's RTC in the meantime.
- A steering wheel and shifter swap mod is planned for a future release.

---
