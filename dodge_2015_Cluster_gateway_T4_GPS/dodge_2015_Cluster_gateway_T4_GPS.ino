// ===========================================================================================================
// 2011-2014 8SPD Dodge charger 2015+ Cluster retrofit Gateway project by MiSO WiperPaw
// This project aims to retrofit newer 2015+ instrument cluster panels into 2011-2014
// equipped with the automatic 8 speed transmission and electronic shifter/rande select 
// dodge chargers with full compatibility. The clusters are largely compatible, however
// some crucial and comfort features are missing/incompatible with the older vehicles.
// This code serves as the main backbone of a CAN-C bus gateway/translator to add and 
// translate the missing features.
// ===========================================================================================================
// Hardware:
// -Teensy T4
// -2x VP230 (or compatible) CAN Bus transcievers
// -Beitian BE-220 GPS module (M10050 chip, baud auto-detected, NMEA-0183)
// -QMC5883L compass (I2C address 0x0D, on Wire / SDA=18 SCL=19)
// -12->5v DC-DC converter (Automotive grade, ideally on-board and quite stable)
// -TVS Dioides (on the CAN transcievers. technically optional, but HIGHLY recommended for permanent installs)
// ===========================================================================================================
// Currently implemented features:
// -Sports mode display
// -Autostick display
// -Manual mode display
// -Actual gear display
// -Region/units change
// -Steering wheel button remapping
// -GPS time keeping (UTC+4 Dubai, periodic RTC correction for drift)
// -Compass heading display (8-point, OFFLINE when no fix or sensor absent)
// -GPS-based compass auto-calibration (COG vs heading offset, SNVS persistent)
// ===========================================================================================================
// Notes:
// -Older firmware and pursuit clusters do NOT support sport mode and actual gear display
// -AlfaOBD is required 
// -Shifter type must be set to "MS7S" and sport mode enabled via AlfaOBD
// -The mod will work without paddle shifters, but it is highly recommended to install and enable via AlfaOBD
// -2015+ Steering wheel and shifter swap mods coming soon! :3
// -The 'OK' button is mapped to the ACC cruise button, recommended to swap that ID if you have ACC equipped car
// but once the steering wheel swap mod is done, that will be solved
// ===========================================================================================================
//
// ==================================
// Connection map:
// ==================================
// ----------------------------------
// Teensy:     |    VP230_Cluster
//  GPIO_0     |     TX   (CAN1 TX)
//  GPIO_1     |     RX   (CAN1 RX)
// ----------------------------------
// Teensy:     |    VP230_Vehicle
//  GPIO_22    |     TX   (CAN2 TX)
//  GPIO_23    |     RX   (CAN2 RX)
//  GPIO_4     |     RX   (parallel — Snooze deepSleep wake pin)
// ----------------------------------
// Teensy:     |    BE-220 GPS:
//  GPIO_28    |     RX   (Serial7 TX → GPS RX, config only)
//  GPIO_29    |     TX   (Serial7 RX ← GPS TX, NMEA stream)
//  3.3V/5V    |     VCC
//  GND        |     GND
// ----------------------------------
// Teensy:     |    QMC5883L:
//  GPIO_18    |     SDA  (Wire)
//  GPIO_19    |     SCL  (Wire)
//  3.3V       |     VCC
//  GND        |     GND
// ----------------------------------
// Hardware note:
//  - GPIO_4 is wired in parallel with GPIO_23 (short jumper wire
//    on the same node). The vehicle bus is used as the sole wake
//    source — vehicle ECU frames are present immediately at
//    ignition providing a reliable dominant edge.
//  - FlexCAN owns GPIO_23 via pad mux so it cannot serve as a
//    Snooze GPIO wake pin. GPIO_4 is a dedicated free GPIO.
//  - QMC5883L module typically includes onboard pull-ups on
//    SDA/SCL — no external resistors needed.
// ----------------------------------
// Low power notes:
//  - deepSleep target: ~9-14 mA (measured, Teensyduino 1.52+)
//  - Wake source: FALLING edge on GPIO_4 (vehicle bus)
//  - deepSleep on T4.0 wakes via reset — setup() reruns on wake.
//    All state reinitialises cleanly each time.
//  - STARTUP_GRACE_MS > SLEEP_TIMEOUT_MS prevents re-sleep before
//    the bus has fully settled after wake.
//  - WDT_T4 is disabled before deepSleep to prevent Snooze from
//    triggering a spurious watchdog reset during sleep.
//    Watchdog re-arms at the top of setup() on every boot/wake.
// ----------------------------------

#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>
#include <Snooze.h>
#include <Wire.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
// IntervalTimer is part of the Teensyduino core — no extra #include needed

// ==================================================================================================================================
// CAN INTERFACES
// ==================================================================================================================================

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64> clusterCan;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> vehicleCan;

// ==================================================================================================================================
// WATCHDOG
// ==================================================================================================================================

WDT_T4<WDT1> wdt;

// ==================================================================================================================================
// SNOOZE CONFIG
//
// deepSleep on T4.0 achieves ~9-14 mA by shutting down all PLLs and
// gating peripheral clocks — far below anything WFI alone can reach.
//
// Wake sources:
//   SnoozeDigital   — GPIO_4 (VP230_Vehicle RXD) FALLING = dominant bus bit
//   SnoozeUSBSerial — required on T4.0 to prevent USB lockup after wake
//
// deepSleep wakes via reset on T4.0 so setup() reruns completely.
// No state needs to survive sleep — everything reinitialises cleanly.
// ==================================================================================================================================

SnoozeDigital   snoozeDigital;
SnoozeUSBSerial snoozeUSB;

SnoozeBlock snoozeConfig(snoozeDigital, snoozeUSB);

// ==================================================================================================================================
// GPS
//
// Confirmed via M10050 chip datasheet: default baud is 115200 bps
// (not the commonly assumed 9600/38400 of older Beitian variants),
// default update rate 10 Hz, NMEA sentences RMC/VTG/GGA/GSA/GSV/GLL.
// GPS_BAUD_CANDIDATES lists 115200 first since it's the documented
// default; remaining entries are kept as fallbacks in case the unit's
// flash-saved config was changed from default at some point (the
// M10050 has persistent flash config that survives power loss).
//
// TinyGPS++ parses $GNRMC / $GPRMC sentences for UTC time.
// Time is shifted to Asia/Dubai (UTC+4, no DST — fixed offset).
//
// GPS time sync strategy:
//   - Opportunistic: applied the moment TinyGPS++ has valid time/date.
//   - Thereafter: resync RTC every GPS_SYNC_INTERVAL_MS to correct
//     for T4.0 RTC crystal drift (~20 ppm = ~1.7 s/day).
//   - Between syncs: RTC keeps time autonomously — no GPS dependency.
//   - If fix is lost: RTC continues, CAN time frame keeps transmitting.
// ==================================================================================================================================

TinyGPSPlus     gps;

#define GPS_SERIAL              Serial7

const uint32_t  GPS_BAUD_CANDIDATES[] = { 115200, 9600, 38400, 4800, 19200, 57600 };
const uint8_t   GPS_BAUD_CANDIDATE_COUNT = sizeof(GPS_BAUD_CANDIDATES) / sizeof(GPS_BAUD_CANDIDATES[0]);
const uint32_t  GPS_BAUD_DETECT_MS    = 2000;   // Listen window per candidate baud rate
uint32_t        gpsBaudActive         = 0;      // Detected baud rate, 0 = not yet detected

const int32_t   UTC_OFFSET_SECONDS   = 4 * 3600L;  // Asia/Dubai UTC+4, no DST
const uint32_t  GPS_SYNC_INTERVAL_MS = 300000;      // Resync RTC every 5 minutes

uint32_t        lastGpsSyncMs        = 0;
bool            gpsSynced            = false;

// ==================================================================================================================================
// QMC5883L COMPASS
//
// I2C address 0x0D. Accessed via Wire (SDA=18, SCL=19).
// Continuous measurement mode, 200 Hz ODR, 8 Gauss range, 512x oversampling.
// Raw X/Y used to compute heading angle, mapped to 8-point compass rose.
// Heading byte sent on CAN ID 0x358 every COMPASS_INTERVAL_MS.
// If sensor is absent or read fails: sends 0x0F (OFFLINE).
//
// Declination for Dubai is approximately +1.5° — negligible for 8-point
// compass (each point = 45°) so no correction applied.
//
// AUTO-CALIBRATION via GPS COG:
// While driving, GPS course-over-ground is compared against the raw
// compass heading to compute a mounting/magnetic offset. Valid samples
// are collected when speed > CAL_MIN_SPEED_KPH and consecutive COG
// readings are stable (straight-line filter). The offset is averaged
// over CAL_SAMPLE_COUNT samples and stored in the iMXRT1062 SNVS
// battery-backed registers so it survives deepSleep and hard resets.
// The corrected heading = rawHeading + headingOffset.
// ==================================================================================================================================

#define QMC5883L_ADDR       0x0D
#define QMC5883L_REG_DATA   0x00    // X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
#define QMC5883L_REG_CTRL1  0x09
#define QMC5883L_REG_RESET  0x0B

// CTRL1: Mode=Continuous(01), ODR=200Hz(11), RNG=8G(01), OSR=512x(00)
// = 0b 00 01 11 01 = 0x1D
#define QMC5883L_CTRL1_VAL  0x1D

bool            compassPresent       = false;
const uint32_t  COMPASS_INTERVAL_MS  = 500;      // Send heading frame every 500 ms
const uint32_t  TIME_INTERVAL_MS     = 1000;     // Send time frame every 1 s

// ==================================================================================================================================
// COMPASS CALIBRATION
//
// GPS COG vs raw compass heading difference is accumulated into a
// rolling average offset that corrects for mounting angle and hard
// iron bias combined.
//
// Sampling conditions (all must be true):
//   - GPS fix valid
//   - Speed >= CAL_MIN_SPEED_KPH (filters stationary/slow GPS noise)
//   - |COG_now - COG_prev| <= CAL_MAX_COG_DELTA (straight-line filter)
//
// Offset storage:
//   iMXRT1062 SNVS_LPGPR0 (battery-backed general purpose register).
//   Survives deepSleep, watchdog reset, and power cycle as long as
//   the 3V coin cell / supercap on VBAT is present.
//   A magic word in SNVS_LPGPR1 validates the stored value —
//   if absent (first boot, flat battery) offset defaults to 0.0°.
// ==================================================================================================================================

const float    CAL_MIN_SPEED_KPH  = 15.0f;   // Minimum speed for valid COG sample
const float    CAL_MAX_COG_DELTA  = 10.0f;   // Max COG change between samples (straight-line)
const uint8_t  CAL_SAMPLE_COUNT   = 16;      // Samples to average per calibration update

// SNVS battery-backed general purpose registers — already defined in imxrt.h
// as SNVS_LPGPR0 (IMXRT_SNVS.offset100) and SNVS_LPGPR1 (IMXRT_SNVS.offset104)
// No redefinition needed — use the Teensyduino names directly.
#define CAL_MAGIC    0xCAFE5883UL

float    headingOffset     = 0.0f;
float    calAccumulator    = 0.0f;
uint8_t  calSampleCount    = 0;
float    prevCog           = -1.0f;

// Forward declaration needed — readCompassHeading is defined later in the file
bool readCompassHeading(float &headingDeg);

// Load offset from SNVS — call once at boot
void loadCalibration() {

    if (SNVS_LPGPR1 == CAL_MAGIC) {

        uint32_t raw;
        raw = SNVS_LPGPR0;
        memcpy(&headingOffset, &raw, sizeof(float));

        // Sanity check — reject obviously corrupt values
        if (headingOffset < -180.0f || headingOffset > 180.0f) {
            headingOffset = 0.0f;
        }

    } else {

        headingOffset = 0.0f;
    }
}

// Save offset to SNVS
void saveCalibration() {

    uint32_t raw;
    memcpy(&raw, &headingOffset, sizeof(float));
    SNVS_LPGPR0 = raw;
    SNVS_LPGPR1 = CAL_MAGIC;
}

// Normalise an angle difference to the range -180..+180
inline float normaliseDelta(float d) {

    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Call from loop() whenever GPS data is fresh.
// Accumulates COG vs compass samples and updates headingOffset
// once CAL_SAMPLE_COUNT valid samples have been collected.
void updateCalibration() {

    if (!compassPresent)            return;
    if (!gps.course.isValid())      return;
    if (!gps.speed.isValid())       return;
    if (!gps.location.isValid())    return;

    float speedKph = gps.speed.kmph();
    if (speedKph < CAL_MIN_SPEED_KPH) {
        prevCog = -1.0f;  // Reset straight-line filter on slow/stop
        return;
    }

    float cog = gps.course.deg();

    // Straight-line filter — reject if COG changed too much since last sample
    if (prevCog >= 0.0f && fabsf(normaliseDelta(cog - prevCog)) > CAL_MAX_COG_DELTA) {
        prevCog = cog;
        return;
    }
    prevCog = cog;

    // Read raw compass heading
    float rawHeading = 0.0f;
    if (!readCompassHeading(rawHeading)) return;

    // Compute offset sample: COG - rawHeading, normalised to ±180°
    float sample = normaliseDelta(cog - rawHeading);

    calAccumulator += sample;
    calSampleCount++;

    if (calSampleCount >= CAL_SAMPLE_COUNT) {

        // Update offset as rolling blend: 75% old + 25% new batch average
        // This gives slow stable convergence rather than jumping on each batch
        float batchAvg  = calAccumulator / (float)calSampleCount;
        headingOffset   = headingOffset * 0.75f + batchAvg * 0.25f;

        calAccumulator  = 0.0f;
        calSampleCount  = 0;

        saveCalibration();
    }
}

// ==================================================================================================================================
// GATEWAY STATES
// ==================================================================================================================================

enum GatewayState : uint8_t {
    ACTIVE,
    STANDBY,
    RECOVERING
};

volatile GatewayState gatewayState = ACTIVE;

// ==================================================================================================================================
// MODIFICATION STATES
// ==================================================================================================================================

// Marked volatile — written inside ISR-driven RX callbacks, read in loop()
volatile bool sportMode  = false;
volatile bool manualMode = false;

// ==================================================================================================================================
// CLUSTER UNIT SETTINGS
// Pressure: 0=Kpa 1=bar 2=PSI 
// Speed: 00=Km/h 01=MPH
// Range: 00=Km 01=Miles
// Fuel Consumption: 00=Km/L 01=L/100Km 02=MPG
// Temp: 00=C 01=F
// ==================================================================================================================================

const uint8_t pressure_unit = 0x02;
const uint8_t speed_unit    = 0x00;
const uint8_t range_unit    = 0x00;
const uint8_t fuelCons_unit = 0x01;
const uint8_t temp_unit     = 0x00;

// ==================================================================================================================================
// STEERING WHEEL CONTROLS
//
// Each CAN source owns its own independent byte pair so they can never
// clobber each other. sendClusterSteeringFrame() OR-merges them at
// transmit time, which means a held OK (0x23A) stays asserted in the
// outgoing frame regardless of how many idle 0x318 frames arrive.
//
// prev* sentinels are 0xFF so the very first real frame always triggers
// a send regardless of its value.
//
// These are written inside RX callbacks — marked volatile accordingly.
// ==================================================================================================================================

static volatile uint8_t btn318_byte4 = 0x00;
static volatile uint8_t btn318_byte5 = 0x00;
static volatile uint8_t btn23A_byte4 = 0x00;
static volatile uint8_t btn23A_byte5 = 0x00;

static volatile uint8_t prev318Byte4 = 0xFF;
static volatile uint8_t prev23AByte0 = 0xFF;

// ==================================================================================================================================
// CONFIG
// ==================================================================================================================================

const uint32_t SLEEP_TIMEOUT_MS   = 20000;
const uint32_t RECOVERY_PERIOD_MS = 1000;

// STARTUP_GRACE_MS must be longer than SLEEP_TIMEOUT_MS.
// On wake from deepSleep, setup() reruns and all idle timers
// start from 0. If grace <= sleep timeout, the housekeeping ISR
// can push idle counters past the sleep threshold before enough
// CAN frames have arrived to reset them, causing an immediate
// re-entry into deepSleep. 10 s gives the bus ample time to
// become fully active before the standby timeout can fire.
const uint32_t STARTUP_GRACE_MS   = 40000;

// ==================================================================================================================================
// IGNITION STATE TRACKING
//
// CAN ID 0x170 (sport/manual mode status) is only present on the bus
// while the ignition is on — it stops the instant the key is turned
// off, well before other bus traffic necessarily stops. This makes it
// a much more reliable "ignition on" signal than a generic bus-idle
// timer, because it isn't affected by any frames the gateway itself
// might be transmitting (which would otherwise create a feedback loop
// keeping the bus looking "active" indefinitely).
//
// ignitionOn is set true the instant 0x170 is seen, and reset to false
// after IGNITION_TIMEOUT_MS without seeing another 0x170 frame.
// All gateway-originated periodic frames (time, compass) are gated on
// this flag so they stop immediately when the key is turned off,
// rather than continuing to keep the cluster — and therefore the
// car — awake.
// ==================================================================================================================================

volatile bool     ignitionOn          = false;
volatile uint32_t ignitionIdleMs      = 0xFFFFFFFF; // starts "timed out"
const uint32_t    IGNITION_TIMEOUT_MS = 2000;       // 0x170 typically repeats every <500 ms when on

// ==================================================================================================================================
// CAN STATISTICS
// ==================================================================================================================================

volatile uint32_t vehicleTxFailures = 0;
volatile uint32_t clusterTxFailures = 0;

// ==================================================================================================================================
// STATUS LED
// ==================================================================================================================================

const int STATUS_LED = LED_BUILTIN;

// ==================================================================================================================================
// INTERVAL TIMER — housekeeping
//
// A single hardware PIT fires every 500 ms, completely independent of
// SysTick or CAN activity. Handles:
//   1. Standby timeout  — idle time accumulation per bus
//   2. Recovery check   — TX failure inspection
//   3. Heartbeat LED    — 500 ms toggle while ACTIVE
//   4. Unit config      — dispatch ~2 s after boot/wake
//   5. Time CAN frame   — flag to send 0x350 every 1 s
//   6. Compass frame    — flag to send 0x358 every 500 ms
//
// ISR only sets flags and increments counters.
// ==================================================================================================================================

IntervalTimer housekeepingTimer;

volatile uint32_t vehicleIdleMs  = 0;
volatile uint32_t clusterIdleMs  = 0;
volatile uint32_t recoveryAccMs  = 0;
volatile uint32_t tickCount      = 0;
volatile uint32_t uptimeMs       = 0;
volatile uint32_t timeAccMs      = 0;
volatile uint32_t compassAccMs   = 0;

volatile bool doHeartbeat        = false;
volatile bool doRecovery         = false;
volatile bool doUnitConfig       = false;
volatile bool pendingUnitConfig  = false;
volatile bool doSendTime         = false;
volatile bool doSendCompass      = false;
//volatile bool doSerialDebug      = false;    // DEBUG: remove before final install

//volatile uint32_t debugAccMs     = 0;        // DEBUG: remove before final install

const uint32_t HOUSEKEEPING_INTERVAL_MS = 500;

void housekeepingISR() {

    uptimeMs      += HOUSEKEEPING_INTERVAL_MS;
    timeAccMs     += HOUSEKEEPING_INTERVAL_MS;
    compassAccMs  += HOUSEKEEPING_INTERVAL_MS;
 //   debugAccMs    += HOUSEKEEPING_INTERVAL_MS;  // DEBUG: remove before final install

    if (vehicleIdleMs < 0xFFFFFFFF) vehicleIdleMs += HOUSEKEEPING_INTERVAL_MS;
    if (clusterIdleMs < 0xFFFFFFFF) clusterIdleMs += HOUSEKEEPING_INTERVAL_MS;

    // Ignition tracking — ignitionIdleMs is reset to 0 by the 0x170
    // handler in processVehicleFrame() every time it's seen. If it
    // accumulates past IGNITION_TIMEOUT_MS, the key is off.
    if (ignitionIdleMs < 0xFFFFFFFF) ignitionIdleMs += HOUSEKEEPING_INTERVAL_MS;
    if (ignitionIdleMs >= IGNITION_TIMEOUT_MS) {
        ignitionOn = false;
    }

    recoveryAccMs += HOUSEKEEPING_INTERVAL_MS;
    if (recoveryAccMs >= RECOVERY_PERIOD_MS) {
        recoveryAccMs = 0;
        doRecovery    = true;
    }

    doHeartbeat = true;

    tickCount++;
    if (pendingUnitConfig && tickCount >= 4) {
        doUnitConfig      = true;
        pendingUnitConfig = false;
    }

    // Time frame: every 1 s (2 ticks)
    if (timeAccMs >= TIME_INTERVAL_MS) {
        timeAccMs   = 0;
        doSendTime  = true;
    }

    // Compass frame: every 500 ms (every tick)
    if (compassAccMs >= COMPASS_INTERVAL_MS) {
        compassAccMs  = 0;
        doSendCompass = true;
    }

    // DEBUG: serial output every 2 s — remove before final install
    // if (debugAccMs >= 2000) {
    //     debugAccMs   = 0;
    //     doSerialDebug = true;
    // }
}

// ==================================================================================================================================
// WATCHDOG CALLBACK
// ==================================================================================================================================

void watchdogCallback() {

    // Optional: save diagnostics here
}

// ==================================================================================================================================
// FORWARD DECLARATIONS
// ==================================================================================================================================

void processVehicleFrame(const CAN_message_t &msg);
void processClusterFrame(const CAN_message_t &msg);
void processSteeringButtons(const CAN_message_t &msg);
void sendClusterSteeringFrame();
void enterDeepSleep();
void initCompass();
// readCompassHeading forward declared earlier (before updateCalibration)
uint8_t headingToOctet(float deg);
void sendTimeFrame();
void sendCompassFrame();
void detectGpsBaud();
void sendUbxEnableNmea(uint32_t baud);
void sendUbxSaveConfig();
void enableNmeaMessageRates();
void tryApplyGpsFix();
void feedGpsParser();
void loadCalibration();
void saveCalibration();
void updateCalibration();
// doSerialDebug output is inline in loop() — no forward declaration needed

// ==================================================================================================================================
// CAN RX CALLBACKS
// ==================================================================================================================================

void onVehicleFrame(const CAN_message_t &msg) {

    vehicleIdleMs = 0;
    processVehicleFrame(msg);
}

void onClusterFrame(const CAN_message_t &msg) {

    clusterIdleMs = 0;
    processClusterFrame(msg);
}

// ==================================================================================================================================
// INITIALIZE CAN
// ==================================================================================================================================

void initCAN() {

    clusterCan.begin();
    clusterCan.setBaudRate(500000);
    clusterCan.setMaxMB(16);
    clusterCan.enableFIFO();
    clusterCan.enableFIFOInterrupt();
    clusterCan.onReceive(onClusterFrame);

    vehicleCan.begin();
    vehicleCan.setBaudRate(500000);
    vehicleCan.setMaxMB(16);
    vehicleCan.enableFIFO();
    vehicleCan.enableFIFOInterrupt();
    vehicleCan.onReceive(onVehicleFrame);
}

// ==================================================================================================================================
// RECOVER CAN
// ==================================================================================================================================

void recoverCAN() {

    gatewayState = RECOVERING;

    clusterCan.reset();
    vehicleCan.reset();

    delay(10);

    initCAN();

    vehicleTxFailures = 0;
    clusterTxFailures = 0;

    gatewayState = ACTIVE;
}

// ==================================================================================================================================
// ENTER DEEP SLEEP
//
// Called from loop() only — never from IRQ context.
//
// Sequence:
//   1. Feed watchdog one last time to clear any pending trigger window
//   2. Disable watchdog — Snooze gates its clock during sleep which
//      would trigger a spurious WDT reset without this step.
//      Proper iMXRT1062 unlock sequence required on every call.
//   3. Turn off status LED
//   4. Call Snooze.deepSleep() — gates PLLs and peripheral clocks,
//      drops to ~9-14 mA, waits for FALLING edge on GPIO_4 (vehicle bus)
//   5. T4.0 wakes via reset — execution never returns past this call.
//      setup() reruns from the top on wake. No cleanup needed before
//      sleep since reset destroys all peripheral state anyway.
// ==================================================================================================================================

void enterDeepSleep() {

    wdt.feed();

    // Disable watchdog using proper iMXRT1062 unlock sequence
    WDOG1_WMCR = 0;
    WDOG1_WSR  = 0x5555;
    WDOG1_WSR  = 0xAAAA;
    WDOG1_WCR  &= ~(1 << 2);

    digitalWrite(STATUS_LED, LOW);

    Snooze.deepSleep(snoozeConfig);
}

// ==================================================================================================================================
// QMC5883L COMPASS
// ==================================================================================================================================

void initCompass() {

    Wire.begin();

    // Reset
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(QMC5883L_REG_RESET);
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) {
        compassPresent = false;
        return;
    }

    delay(10);

    // Configure: continuous mode, 200 Hz, 8 Gauss, 512x OSR
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(QMC5883L_REG_CTRL1);
    Wire.write(QMC5883L_CTRL1_VAL);
    Wire.endTransmission();

    compassPresent = true;
}

// Read raw X/Y and compute heading in degrees (0–360)
// Returns false if read fails
bool readCompassHeading(float &headingDeg) {

    if (!compassPresent) return false;

    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(QMC5883L_REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    int16_t x = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t y = (int16_t)(Wire.read() | (Wire.read() << 8));
    /* int16_t z = */ Wire.read(); Wire.read(); // Z not needed for 2D heading

    headingDeg = atan2f((float)y, (float)x) * 180.0f / (float)M_PI;

    if (headingDeg < 0.0f)   headingDeg += 360.0f;
    if (headingDeg >= 360.0f) headingDeg -= 360.0f;

    return true;
}

// Map heading degrees to 8-point compass octet
// Each point spans 45°, centred on its cardinal direction
// N=0x00 NE=0x01 E=0x02 SE=0x03 S=0x04 SW=0x05 W=0x06 NW=0x07 OFFLINE=0x0F
uint8_t headingToOctet(float deg) {

    // Normalise to 0–360
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    // Each sector is 45° wide, offset by 22.5° so N is centred on 0°/360°
    uint8_t sector = (uint8_t)((deg + 22.5f) / 45.0f) % 8;

    return sector; // 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW
}

// ==================================================================================================================================
// SEND COMPASS FRAME — ID 0x358, length 1
//
// Byte 0: heading octet (0x00–0x07) or 0x0F if offline
// ==================================================================================================================================

void sendCompassFrame() {

    float headingDeg = 0.0f;
    uint8_t headingByte;

    if (readCompassHeading(headingDeg)) {

        // Apply calibration offset — corrects mounting angle and hard iron bias
        headingDeg += headingOffset;

        // Renormalise to 0–360 after offset
        while (headingDeg <    0.0f) headingDeg += 360.0f;
        while (headingDeg >= 360.0f) headingDeg -= 360.0f;

        headingByte = headingToOctet(headingDeg);

    } else {
        headingByte = 0x0F; // OFFLINE
    }

    CAN_message_t msg;
    msg.id     = 0x358;
    msg.len    = 8;
    msg.buf[0] = headingByte;

    clusterCan.write(msg);
}

// ==================================================================================================================================
// UBX: FORCE NMEA OUTPUT
//
// The M10050 supports both NMEA and UBX binary protocols and can be
// configured (by factory default on some batches, or by a previous
// owner/seller) to output UBX binary only. TinyGPS++ only understands
// NMEA text, so if the module is in UBX mode it will have a perfectly
// good fix but never produce a single parseable sentence — exactly
// matching a "red LED on, no fix detected" symptom.
//
// This sends the standard UBX CFG-PRT (0x06 0x00) message, which sets
// the UART1 port's output protocol mask to NMEA-only (bit 1 = 0x02)
// and input protocol mask to accept both UBX and NMEA (0x03), at the
// currently active baud rate. The message is structured as raw bytes
// per the u-blox protocol spec and does not require parsing any
// response — the module applies it and begins NMEA output within
// a few sentence cycles.
//
// Sent once per detected baud rate that produces a recognisable byte
// stream at all (even all-binary), since we cannot otherwise tell the
// module to switch out of UBX mode before it ever sends an NMEA '$'.
// ==================================================================================================================================

void sendUbxEnableNmea(uint32_t baud) {

    // UBX-CFG-PRT for UART1, poll-and-set variant (full 20-byte payload)
    // Class 0x06 (CFG), ID 0x00 (PRT), length 20
    uint8_t payload[20] = {
        0x01,             // portID = 1 (UART1)
        0x00,             // reserved
        0x00, 0x00,       // txReady (disabled)
        0xD0, 0x08, 0x00, 0x00, // mode: 8N1, no parity (0x000008D0)
        0x00, 0x00, 0x00, 0x00, // baudRate placeholder, filled below
        0x03, 0x00,       // inProtoMask: UBX + NMEA (0x0001 | 0x0002)
        0x02, 0x00,       // outProtoMask: NMEA only (0x0002)
        0x00, 0x00,       // flags
        0x00, 0x00        // reserved
    };

    // Fill in current baud rate (little-endian uint32)
    payload[8]  = (uint8_t)(baud & 0xFF);
    payload[9]  = (uint8_t)((baud >> 8) & 0xFF);
    payload[10] = (uint8_t)((baud >> 16) & 0xFF);
    payload[11] = (uint8_t)((baud >> 24) & 0xFF);

    uint8_t header[6] = { 0xB5, 0x62, 0x06, 0x00, 20, 0x00 };

    // UBX checksum (8-bit Fletcher) over class/id/length/payload
    uint8_t ckA = 0, ckB = 0;
    for (uint8_t i = 2; i < 6; i++) { ckA += header[i]; ckB += ckA; }
    for (uint8_t i = 0; i < 20; i++) { ckA += payload[i]; ckB += ckA; }

    GPS_SERIAL.write(header, 6);
    GPS_SERIAL.write(payload, 20);
    GPS_SERIAL.write(ckA);
    GPS_SERIAL.write(ckB);
    GPS_SERIAL.flush();
}

// ==================================================================================================================================
// UBX: SAVE CONFIGURATION (CFG-CFG)
//
// CFG-PRT was ACKed by the module but live NMEA output did not start —
// the module appears to apply CFG-PRT to its stored/next-session
// config without immediately switching the currently active UART
// stream. UBX-CFG-CFG (class 0x06, ID 0x09) tells the receiver to
// save the current configuration to its battery-backed/flash memory
// and is the standard mechanism many u-blox-protocol receivers use to
// make a CFG-PRT change "stick" and take effect on the live port.
//
// This is non-destructive — it does not clear ephemeris, almanac, or
// fix data, only persists communication/config settings.
// ==================================================================================================================================

void sendUbxSaveConfig() {

    // UBX-CFG-CFG payload: clearMask, saveMask, loadMask, deviceMask
    // saveMask = 0x0000061F saves ioPort, msgConf, infMsg, navConf, rxmConf
    uint8_t payload[13] = {
        0x00, 0x00, 0x00, 0x00,   // clearMask = 0 (clear nothing)
        0x1F, 0x06, 0x00, 0x00,   // saveMask  = 0x0000061F
        0x00, 0x00, 0x00, 0x00,   // loadMask  = 0 (load nothing)
        0x01                      // deviceMask = 0x01 (BBR — battery-backed RAM)
    };

    uint8_t header[6] = { 0xB5, 0x62, 0x06, 0x09, 13, 0x00 };

    uint8_t ckA = 0, ckB = 0;
    for (uint8_t i = 2; i < 6; i++) { ckA += header[i]; ckB += ckA; }
    for (uint8_t i = 0; i < 13; i++) { ckA += payload[i]; ckB += ckA; }

    GPS_SERIAL.write(header, 6);
    GPS_SERIAL.write(payload, 13);
    GPS_SERIAL.write(ckA);
    GPS_SERIAL.write(ckB);
    GPS_SERIAL.flush();
}

// ==================================================================================================================================
// UBX: POLL NAV-PVT (one-shot request, class 0x01 ID 0x07)
//
// Sends a zero-length poll request for the navigation position/
// velocity/time solution. If the module responds with a UBX-NAV-PVT
// message (class 0x01, ID 0x07) it proves the receiver has a live
// navigation solution and will respond to direct requests, even if
// its periodic/automatic output messages are currently disabled.
// This isolates "module has no fix" from "module has a fix but
// periodic output is turned off" — two very different problems.
// ==================================================================================================================================

void pollUbxNavPvt() {

    uint8_t header[6] = { 0xB5, 0x62, 0x01, 0x07, 0x00, 0x00 }; // length 0 = poll

    uint8_t ckA = 0, ckB = 0;
    for (uint8_t i = 2; i < 6; i++) { ckA += header[i]; ckB += ckA; }

    GPS_SERIAL.write(header, 6);
    GPS_SERIAL.write(ckA);
    GPS_SERIAL.write(ckB);
    GPS_SERIAL.flush();
}

// ==================================================================================================================================
// UBX: ENABLE PERIODIC NMEA MESSAGES (CFG-MSG, class 0x06 ID 0x01)
//
// CFG-PRT controls which protocols a port accepts/emits, but each
// individual NMEA sentence type also has its own independent output
// rate controlled separately via CFG-MSG. If a previous configuration
// disabled all per-message rates, the port can be correctly set to
// NMEA-capable yet still emit nothing, because every message's "send
// every N navigation solutions" counter is set to 0.
//
// This explicitly sets the output rate to 1 (every navigation epoch)
// for RMC and GGA on the currently active port (UART1, target index 1
// in the rate array — applies to all I/O targets in this profile).
// ==================================================================================================================================

void enableNmeaMessageRates() {

    // NMEA message class is 0xF0; RMC = 0x04, GGA = 0x00
    const uint8_t nmeaMsgIds[] = { 0x04, 0x00 }; // RMC, GGA

    for (uint8_t m = 0; m < 2; m++) {

        uint8_t payload[8] = {
            0xF0, nmeaMsgIds[m],   // msgClass, msgID
            0x01,                  // rate on I2C target
            0x01,                  // rate on UART1 target — every epoch
            0x01,                  // rate on UART2 target
            0x01,                  // rate on USB target
            0x01,                  // rate on SPI target
            0x00                   // reserved
        };

        uint8_t header[6] = { 0xB5, 0x62, 0x06, 0x01, 8, 0x00 };

        uint8_t ckA = 0, ckB = 0;
        for (uint8_t i = 2; i < 6; i++) { ckA += header[i]; ckB += ckA; }
        for (uint8_t i = 0; i < 8; i++) { ckA += payload[i]; ckB += ckA; }

        GPS_SERIAL.write(header, 6);
        GPS_SERIAL.write(payload, 8);
        GPS_SERIAL.write(ckA);
        GPS_SERIAL.write(ckB);
        GPS_SERIAL.flush();

        delay(100);
    }
}

// ==================================================================================================================================
// GPS BAUD AUTO-DETECTION
//
// Tries each candidate baud rate in turn. At each candidate, first
// sends the UBX CFG-PRT command to force NMEA output (harmless no-op
// if the module is already in NMEA mode, since output simply continues
// as NMEA). Then listens for at least two '$' sentence-start characters
// — the universal NMEA sentence delimiter. A genuine match (correct
// baud AND now-NMEA mode) shows a clean, repeating stream of '$'
// characters; a wrong baud rate produces no recognisable '$' at all,
// since the byte boundaries themselves are misaligned.
//
// Called once from setup(). Blocking by design — this only runs once
// at boot/wake and the watchdog is fed throughout.
// ==================================================================================================================================

void detectGpsBaud() {

    for (uint8_t i = 0; i < GPS_BAUD_CANDIDATE_COUNT; i++) {

        uint32_t candidate = GPS_BAUD_CANDIDATES[i];

        GPS_SERIAL.end();
        GPS_SERIAL.begin(candidate);
        delay(50); // let UART settle after baud change

        // Force NMEA output in case module is currently in UBX binary
        // mode. Safe to send even if baud is wrong — the module simply
        // won't understand it and we fall through to the next candidate.
        sendUbxEnableNmea(candidate);
        delay(100); // give module time to apply and start NMEA output

        uint32_t start    = millis();
        uint16_t dollarCount = 0;

        while (millis() - start < GPS_BAUD_DETECT_MS) {

            wdt.feed();

            while (GPS_SERIAL.available()) {

                if (GPS_SERIAL.read() == '$') {
                    dollarCount++;
                }
            }

            // Two or more '$' in the window is a confident match —
            // a misaligned baud rate essentially never produces this
            if (dollarCount >= 2) {
                gpsBaudActive = candidate;
                return;
            }
        }
    }

    // Nothing matched — fall back to the most common default and
    // let normal operation retry; better than leaving the port closed
    gpsBaudActive = GPS_BAUD_CANDIDATES[0];
    GPS_SERIAL.end();
    GPS_SERIAL.begin(gpsBaudActive);
    sendUbxEnableNmea(gpsBaudActive);
}

// ==================================================================================================================================
// GPS TIME SYNC
//
// Non-blocking opportunistic approach — no waiting, no timeout.
// tryApplyGpsFix() checks whether TinyGPS++ has fresh valid time and
// date fields from bytes fed by feedGpsParser() and applies immediately.
//
// Deliberately does NOT require gps.location.isValid() — a position
// fix needs multiple satellites, but time is available from a single
// satellite as soon as the receiver decodes the broadcast almanac.
// For timekeeping purposes this is more than sufficient, and means
// the RTC gets set much faster (often within seconds of power-on
// in a known location with good sky view).
//
// RTC resync every GPS_SYNC_INTERVAL_MS corrects T4.0 crystal drift.
// ==================================================================================================================================

void tryApplyGpsFix() {

    if (!gps.time.isValid()   ||
        !gps.date.isValid()   ||
        !gps.time.isUpdated()) {
        return;
    }

    // Throttle: don't re-apply more often than GPS_SYNC_INTERVAL_MS
    if (gpsSynced &&
        (millis() - lastGpsSyncMs) < GPS_SYNC_INTERVAL_MS) {
        return;
    }

    tmElements_t tm;
    tm.Second = gps.time.second();
    tm.Minute = gps.time.minute();
    tm.Hour   = gps.time.hour();
    tm.Day    = gps.date.day();
    tm.Month  = gps.date.month();
    tm.Year   = gps.date.year() - 1970;

    time_t utcTime   = makeTime(tm);
    time_t localTime = utcTime + UTC_OFFSET_SECONDS;

    setTime(localTime);

    lastGpsSyncMs = millis();
    gpsSynced     = true;
}

// Feed GPS parser continuously — call from loop() every iteration
// so we don't miss bytes between sync intervals
void feedGpsParser() {

    while (GPS_SERIAL.available()) {
        gps.encode(GPS_SERIAL.read());
    }
}

// ==================================================================================================================================
// SEND TIME FRAME — ID 0x350, length 8
//
// Uses Teensy RTC (TimeLib) for current local time.
// RTC is set and periodically corrected by GPS. If GPS has never
// synced, the RTC will show the Teensy's default time (Jan 1 2000)
// until the first fix — this is visible on the cluster as a cue
// that GPS has not yet locked.
//
// Byte layout (all decimal BCD values):
//   buf[0] = seconds
//   buf[1] = minutes
//   buf[2] = hours
//   buf[3] = year high byte  (e.g. 0x07 for 2024)
//   buf[4] = year low byte   (e.g. 0xE8 for 2024)
//   buf[5] = month
//   buf[6] = day
//   buf[7] = bus identifier  0x39
// ==================================================================================================================================

void sendTimeFrame() {

    time_t now = ::now();

    uint16_t yr  = (uint16_t)year(now);
    uint8_t  mo  = (uint8_t)month(now);
    uint8_t  dy  = (uint8_t)day(now);
    uint8_t  hr  = (uint8_t)hour(now);
    uint8_t  mn  = (uint8_t)minute(now);
    uint8_t  sc  = (uint8_t)second(now);

    CAN_message_t msg;
    msg.id     = 0x350;
    msg.len    = 8;
    msg.buf[0] = sc;
    msg.buf[1] = mn;
    msg.buf[2] = hr;
    msg.buf[3] = (uint8_t)(yr >> 8);
    msg.buf[4] = (uint8_t)(yr & 0xFF);
    msg.buf[5] = mo;
    msg.buf[6] = dy;
    msg.buf[7] = 0x39;  // Bus identifier

    clusterCan.write(msg);
}

// ==================================================================================================================================
// VEHICLE -> CLUSTER PROCESSING
// ==================================================================================================================================

void processVehicleFrame(const CAN_message_t &msg) {

    processSteeringButtons(msg);

    CAN_message_t out = msg;

    switch (msg.id) {

        // ----------------------------------------------------
        // 0x350 — Block vehicle time frame
        // Gateway supplies its own 0x350 from GPS/RTC.
        // ----------------------------------------------------

        case 0x350:

            return;

        // ----------------------------------------------------
        // 0x358 — Block vehicle compass frame
        // Gateway supplies its own 0x358 from the QMC5883L.
        // Returning here prevents the vehicle frame from being
        // forwarded to the cluster.
        // ----------------------------------------------------

        case 0x358:

            return;

        // ----------------------------------------------------
        // 0x3E8 — Set shifter type
        // ----------------------------------------------------

        case 0x3E8:

            if (msg.len >= 6) {
                out.buf[5] = 0x9E;
            }
            break;

        // ----------------------------------------------------
        // 0x3F3 — Enable gear display
        // ----------------------------------------------------

        case 0x3F3:

            if (msg.len >= 2) {
                out.buf[0] = 0x81;
                out.buf[1] = 0x02;
            }
            break;

        // ----------------------------------------------------
        // 0x170 — Sport / manual mode tracking
        // ----------------------------------------------------

        case 0x170:

            // 0x170 presence is the ignition-on signal — reset the
            // idle counter and assert ignitionOn every time it's seen.
            ignitionIdleMs = 0;
            ignitionOn     = true;

            if (msg.len >= 3) {

                if (msg.buf[1] == 0xFB) {

                    sportMode = true;

                    // Keep cluster readout in 'D' otherwise gear display goes full white
                    if (msg.buf[2] == 0x53) {
                        out.buf[2] = 0x44;
                    }

                    // Manual Gears '1' to '8'
                    manualMode = (msg.buf[2] >= 0x31 && msg.buf[2] <= 0x38);

                } else {

                    sportMode  = false;
                    manualMode = false;
                }
            }
            break;

        // ----------------------------------------------------
        // 0x330 — Sport mode display
        // ----------------------------------------------------

        case 0x330:

            if (sportMode && msg.len >= 8) {
                out.buf[7] = 0x04;
            }
            break;

        // ----------------------------------------------------
        // 0x144 — Manual mode display
        // ----------------------------------------------------

        case 0x144:

            if (manualMode && msg.len >= 8) {
                out.buf[7] = 0x80;
            }
            break;
    }

    if (!clusterCan.write(out)) {
        clusterTxFailures++;
    }
}

// ==================================================================================================================================
// CLUSTER -> VEHICLE
// ==================================================================================================================================

void processClusterFrame(const CAN_message_t &msg) {

    if (!vehicleCan.write(msg)) {
        vehicleTxFailures++;
    }
}

// ==================================================================================================================================
// SEND UNIT SETTING
// ==================================================================================================================================

void sendUnitSetting(uint8_t setting, uint8_t value) {

    CAN_message_t unitMsg;

    unitMsg.id     = 0x314;
    unitMsg.len    = 3;
    unitMsg.buf[0] = setting;
    unitMsg.buf[1] = 0x03;
    unitMsg.buf[2] = value;

    clusterCan.write(unitMsg);

    delay(10);
}

// ==================================================================================================================================
// SEND ALL UNIT SETTINGS
// ==================================================================================================================================

void sendAllUnitSettings() {

    sendUnitSetting(0x60, pressure_unit);   // Pressure
    sendUnitSetting(0x61, speed_unit);      // Speed
    sendUnitSetting(0x62, range_unit);      // Range
    sendUnitSetting(0x63, fuelCons_unit);   // Fuel Consumption
    sendUnitSetting(0x65, temp_unit);       // Temperature
}

// ==================================================================================================================================
// SWC BUTTONS — merge all sources and transmit
// ==================================================================================================================================

void sendClusterSteeringFrame() {

    CAN_message_t steerMsg;

    steerMsg.id  = 0x22D;
    steerMsg.len = 8;

    memset(steerMsg.buf, 0, 8);

    steerMsg.buf[4] = btn318_byte4 | btn23A_byte4;
    steerMsg.buf[5] = btn318_byte5 | btn23A_byte5;

    clusterCan.write(steerMsg);
}

// ==================================================================================================================================
// DECODE OLD & SEND NEW SWC BUTTONS
// ==================================================================================================================================

void processSteeringButtons(const CAN_message_t &msg) {

    bool changed = false;

    // --------------------------------------------------------
    // ID 0x318 — directional pad
    // Only ever writes btn318_* — never touches btn23A_*
    // --------------------------------------------------------

    if (msg.id == 0x318 && msg.len >= 5) {

        uint8_t raw = msg.buf[4];

        if (raw != prev318Byte4) {

            prev318Byte4 = raw;

            btn318_byte4 = 0x00;
            btn318_byte5 = 0x00;

            switch (raw) {

                case 0x01: btn318_byte4 = 0x10; break; // LEFT
                case 0x04: btn318_byte4 = 0x40; break; // DOWN
                case 0x10: btn318_byte5 = 0x04; break; // UP
                case 0x40: btn318_byte5 = 0x01; break; // RIGHT
            }

            changed = true;
        }
    }

    // --------------------------------------------------------
    // ID 0x23A — OK button
    // Only ever writes btn23A_* — never touches btn318_*
    // --------------------------------------------------------

    if (msg.id == 0x23A && msg.len >= 1) {

        uint8_t raw = msg.buf[0];

        if (raw != prev23AByte0) {

            prev23AByte0 = raw;

            btn23A_byte4 = 0x00;
            btn23A_byte5 = 0x00;

            if (raw == 0x80) {
                btn23A_byte5 = 0x10;
            }

            changed = true;
        }
    }

    if (changed) {
        sendClusterSteeringFrame();
    }
}

// ==================================================================================================================================
// SETUP
//
// Runs on every boot AND on every wake from deepSleep (T4.0 wakes
// via reset). All state is reinitialised here cleanly each time.
// ==================================================================================================================================

void setup() {

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    Serial.begin(115200);

    // --------------------------------------------------------
    // WATCHDOG
    // --------------------------------------------------------

    WDT_timings_t config;

    config.trigger  = 5;
    config.timeout  = 6;
    config.callback = watchdogCallback;

    wdt.begin(config);

    // --------------------------------------------------------
    // CAN INIT
    // --------------------------------------------------------

    initCAN();

    // --------------------------------------------------------
    // GPS SERIAL — AUTO-DETECT BAUD RATE
    // Serial7: GPIO_28 (TX→GPS RX), GPIO_29 (RX←GPS TX)
    // M10050 default baud is 115200; detectGpsBaud() tries that
    // first and falls back through other common rates if needed.
    // Blocking for up to ~12 s worst case across all candidates —
    // watchdog is fed throughout. Runs once per boot/wake.
    // --------------------------------------------------------

    detectGpsBaud();

    // --------------------------------------------------------
    // GPS PROTOCOL CONFIG
    // CFG-PRT alone is not sufficient on this module — it switches
    // the port's accepted/emitted protocol but each individual NMEA
    // sentence type has its own independent output-rate counter
    // (CFG-MSG) which must be explicitly enabled, otherwise the port
    // is correctly set to NMEA yet still emits nothing. Confirmed via
    // direct UBX ACK/NAK inspection during bench testing — see commit
    // history / chat log for the diagnostic raw-dump trace if this
    // module is ever swapped and the issue resurfaces.
    // --------------------------------------------------------

    sendUbxEnableNmea(gpsBaudActive);
    delay(100);

    enableNmeaMessageRates();
    delay(100);

    sendUbxSaveConfig();
    delay(100);

    // --------------------------------------------------------
    // COMPASS INIT
    // --------------------------------------------------------

    initCompass();

    // --------------------------------------------------------
    // LOAD COMPASS CALIBRATION FROM SNVS
    // Restores heading offset from battery-backed register.
    // Defaults to 0.0° if never saved or battery flat.
    // --------------------------------------------------------

    loadCalibration();

    Serial.println("Retrofit Cluster Gateway started");

    // --------------------------------------------------------
    // SNOOZE WAKE PIN CONFIG
    // GPIO_4 = VP230_Vehicle RXD (parallel wire from GPIO_23 node)
    // --------------------------------------------------------

    snoozeDigital.pinMode(4, INPUT_PULLUP, FALLING);

    // --------------------------------------------------------
    // HOUSEKEEPING TIMER
    // --------------------------------------------------------

    housekeepingTimer.begin(housekeepingISR, 500000); // 500 000 µs = 500 ms

    // --------------------------------------------------------
    // Queue initial unit settings — dispatched ~2 s after boot/wake
    // --------------------------------------------------------

    pendingUnitConfig = true;
    tickCount         = 0;
}

// ==================================================================================================================================
// LOOP
// ==================================================================================================================================

void loop() {

    // --------------------------------------------------------
    // FEED WATCHDOG
    // --------------------------------------------------------

    wdt.feed();

    // --------------------------------------------------------
    // FEED GPS PARSER
    // Continuously consumes Serial7 bytes so TinyGPS++ always
    // has fresh data. Cheap — just drains the UART FIFO.
    // --------------------------------------------------------

    feedGpsParser();

    // --------------------------------------------------------
    // OPPORTUNISTIC GPS TIME SYNC
    // Applies fix to RTC the moment TinyGPS++ has fresh valid data.
    // Non-blocking — returns immediately if no fix available yet.
    // Also handles periodic resync every GPS_SYNC_INTERVAL_MS.
    // --------------------------------------------------------

    tryApplyGpsFix();

    // --------------------------------------------------------
    // COMPASS CALIBRATION UPDATE
    // --------------------------------------------------------

    updateCalibration();

    // --------------------------------------------------------
    // STANDBY ENTRY
    // --------------------------------------------------------

    if (uptimeMs      >= STARTUP_GRACE_MS &&
        vehicleIdleMs >= SLEEP_TIMEOUT_MS &&
        clusterIdleMs >= SLEEP_TIMEOUT_MS &&
        gatewayState  != STANDBY) {

        enterDeepSleep();
    }

    // --------------------------------------------------------
    // PERIODIC RECOVERY CHECK
    // --------------------------------------------------------

    if (doRecovery) {

        doRecovery = false;

        if (vehicleTxFailures > 100 ||
            clusterTxFailures > 100) {

            recoverCAN();
        }
    }

    // --------------------------------------------------------
    // HEARTBEAT LED
    // --------------------------------------------------------

    if (doHeartbeat) {

        doHeartbeat = false;

        if (gatewayState == ACTIVE) {
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        }
    }

    // --------------------------------------------------------
    // UNIT CONFIG DISPATCH
    // --------------------------------------------------------

    if (doUnitConfig) {

        doUnitConfig = false;

        sendAllUnitSettings();
    }

    // --------------------------------------------------------
    // SEND TIME FRAME — 0x350 every 1 s
    // Gated on ignitionOn (derived from 0x170 presence) rather than
    // generic bus idle time, since the gateway's own transmissions
    // would otherwise keep the bus looking "active" indefinitely.
    // --------------------------------------------------------

    if (doSendTime) {

        doSendTime = false;

        if (ignitionOn) {
            sendTimeFrame();
        }
    }

    // --------------------------------------------------------
    // SEND COMPASS FRAME — 0x358 every 500 ms
    // Same ignitionOn gate as the time frame above.
    // --------------------------------------------------------

    if (doSendCompass) {

        doSendCompass = false;

        if (ignitionOn) {
            sendCompassFrame();
        }
    }

    // --------------------------------------------------------
    // DEBUG SERIAL OUTPUT — remove before final install
    // Prints every 2 s: GPS baud/fix status, UTC time, local time,
    // raw compass heading degrees, and heading octet sent to cluster
    // --------------------------------------------------------

    // if (doSerialDebug) {

    //     doSerialDebug = false;

    //     // --- GPS ---
    //     Serial.println(F("--- GPS ---"));
    //     Serial.print(F("  Baud rate : "));
    //     Serial.println(gpsBaudActive);
    //     Serial.print(F("  Chars seen: "));
    //     Serial.println(gps.charsProcessed());
    //     Serial.print(F("  Sentences : "));
    //     Serial.println(gps.sentencesWithFix());
    //     Serial.print(F("  Fix valid : "));
    //     Serial.println(gps.location.isValid() ? F("YES") : F("NO"));
    //     Serial.print(F("  Satellites: "));
    //     Serial.println(gps.satellites.isValid() ? gps.satellites.value() : 0);
    //     Serial.print(F("  GPS synced: "));
    //     Serial.println(gpsSynced ? F("YES") : F("NO"));
    //     Serial.print(F("  Ignition  : "));
    //     Serial.println(ignitionOn ? F("ON") : F("OFF"));

    //     if (gpsSynced) {

    //         // Print RTC local time (UTC+4)
    //         time_t now = ::now();
    //         char buf[48];
    //         snprintf(buf, sizeof(buf),
    //                  "  Local time : %04d-%02d-%02d %02d:%02d:%02d",
    //                  year(now), month(now), day(now),
    //                  hour(now), minute(now), second(now));
    //         Serial.println(buf);

    //         snprintf(buf, sizeof(buf),
    //                  "  Last sync  : %lu ms ago",
    //                  millis() - lastGpsSyncMs);
    //         Serial.println(buf);

    //     } else {

    //         Serial.println(F("  Local time : not set (awaiting fix)"));
    //     }

    //     // --- COMPASS ---
    //     Serial.println(F("--- Compass ---"));
    //     Serial.print(F("  Present    : "));
    //     Serial.println(compassPresent ? F("YES") : F("NO"));

    //     if (compassPresent) {

    //         float headingDeg = 0.0f;
    //         if (readCompassHeading(headingDeg)) {

    //             float corrected = headingDeg + headingOffset;
    //             while (corrected <    0.0f) corrected += 360.0f;
    //             while (corrected >= 360.0f) corrected -= 360.0f;

    //             uint8_t octet = headingToOctet(corrected);

    //             const char* const dirNames[] = {
    //                 "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    //             };

    //             char buf[64];
    //             snprintf(buf, sizeof(buf),
    //                      "  Raw        : %.1f deg",
    //                      headingDeg);
    //             Serial.println(buf);

    //             snprintf(buf, sizeof(buf),
    //                      "  Offset     : %.1f deg (%u samples pending)",
    //                      headingOffset, calSampleCount);
    //             Serial.println(buf);

    //             snprintf(buf, sizeof(buf),
    //                      "  Corrected  : %.1f deg -> %s (0x%02X)",
    //                      corrected, dirNames[octet], octet);
    //             Serial.println(buf);

    //         } else {

    //             Serial.println(F("  Heading    : read failed (OFFLINE)"));
    //         }

    //     } else {

    //         Serial.println(F("  Heading    : OFFLINE (sensor not found)"));
    //     }

    //     Serial.println();
    // }

    // --------------------------------------------------------
    // END OF MAIN LOOP :3
    // --------------------------------------------------------
}
