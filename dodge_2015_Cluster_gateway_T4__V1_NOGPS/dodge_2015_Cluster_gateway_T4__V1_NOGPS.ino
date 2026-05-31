// ===========================================================================================================
// 2011-2014 8SPD Dodge charger 2015+ Cluster retrofit Gateway project by MiSO WiperPaw
// This project aims to retrofit newer 2015+ instrument cluster panels into 2011-2014
//  dodge chargers equipped with the automatic 8 speed transmission and electronic shifter/rande select 
// with full compatibility. The clusters are largely compatible, however
// some crucial and comfort features are missing/incompatible with the older vehicles.
// This project serves as a CAN-C bus gateway/translator to add and translate the missing features.
// ===========================================================================================================
// Hardware:
// -Teensy T4
// -2x VP230 (or compatible) CAN Bus transcievers
// -Meshnology BE-220 GPS+Compass module (to add proper timekeeping and compass Heading info)
// -12->5v DC-DC converter (Automotive grade, ideally on-board and quite stable)
// -TVS Dioides (on the CAN transcievers. technically optional, but HIGHLY recommended for permanent installs) 
// ===========================================================================================================
// Currently implemented features:
// -sports mode display
// -autostick display
// -manual mode display
// -actual gear display
// -region/units change
// -steering wheel button remapping
// -*GPS time keeping
// -*Compass heading display
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
// Hardware note:
//  - GPIO_4 is wired in parallel with GPIO_23 (short jumper wire
//    on the same node). The vehicle bus is used as the sole wake
//    source — vehicle ECU frames are present immediately at
//    ignition providing a reliable dominant edge.
//  - FlexCAN owns GPIO_23 via pad mux so it cannot serve as a
//    Snooze GPIO wake pin. GPIO_4 is a dedicated free GPIO.
// ----------------------------------
// Teensy:     |    BN-220:
//  NA         |     NA
//  NA         |     NA
//  NA         |     NA
//  NA         |     NA
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

SnoozeDigital  snoozeDigital;
SnoozeUSBSerial snoozeUSB;

SnoozeBlock snoozeConfig(snoozeDigital, snoozeUSB);

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

const uint32_t SLEEP_TIMEOUT_MS   = 5000;
const uint32_t RECOVERY_PERIOD_MS = 1000;

// STARTUP_GRACE_MS must be longer than SLEEP_TIMEOUT_MS.
// On wake from deepSleep, setup() reruns and all idle timers
// start from 0. If grace <= sleep timeout, the housekeeping ISR
// can push idle counters past the sleep threshold before enough
// CAN frames have arrived to reset them, causing an immediate
// re-entry into deepSleep. 10 s gives the bus ample time to
// become fully active before the standby timeout can fire.
const uint32_t STARTUP_GRACE_MS   = 10000;

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
//
// ISR only sets flags and increments counters. CAN writes, delay(),
// and Snooze.deepSleep() all happen in loop() only.
// ==================================================================================================================================

IntervalTimer housekeepingTimer;

volatile uint32_t vehicleIdleMs = 0;
volatile uint32_t clusterIdleMs = 0;
volatile uint32_t recoveryAccMs = 0;
volatile uint32_t tickCount     = 0;
volatile uint32_t uptimeMs      = 0;

volatile bool doHeartbeat       = false;
volatile bool doRecovery        = false;
volatile bool doUnitConfig      = false;
volatile bool pendingUnitConfig = false;

const uint32_t HOUSEKEEPING_INTERVAL_MS = 500;

void housekeepingISR() {

    uptimeMs += HOUSEKEEPING_INTERVAL_MS;

    if (vehicleIdleMs < 0xFFFFFFFF) vehicleIdleMs += HOUSEKEEPING_INTERVAL_MS;
    if (clusterIdleMs < 0xFFFFFFFF) clusterIdleMs += HOUSEKEEPING_INTERVAL_MS;

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

// ==================================================================================================================================
// CAN RX CALLBACKS
//
// FlexCAN_T4 calls these directly from its IRQ handler the moment a
// frame lands in the mailbox FIFO. All gateway processing happens here.
// Keep lean: no delay(), no Serial, no Snooze calls.
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

    vehicleCan.begin();
    vehicleCan.setBaudRate(500000);
    vehicleCan.setMaxMB(16);
    vehicleCan.enableFIFO();
    vehicleCan.enableFIFOInterrupt();
    vehicleCan.onReceive(onVehicleFrame);

    clusterCan.begin();
    clusterCan.setBaudRate(500000);
    clusterCan.setMaxMB(16);
    clusterCan.enableFIFO();
    clusterCan.enableFIFOInterrupt();
    clusterCan.onReceive(onClusterFrame);
}

// ==================================================================================================================================
// RECOVER CAN
// ==================================================================================================================================

void recoverCAN() {

    gatewayState = RECOVERING;

    vehicleCan.reset();
    clusterCan.reset();

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

    // Feed watchdog one final time to clear any pending trigger
    // window before we disable it
    wdt.feed();

    // Disable watchdog using proper iMXRT1062 unlock sequence.
    // Required before Snooze deepSleep on every sleep entry —
    // Snooze gates the watchdog clock during sleep which causes
    // a spurious reset if the watchdog is still armed.
    // Direct register writes without unlocking are silently
    // ignored on subsequent calls after WDT_T4 has armed it.
    WDOG1_WMCR = 0;
    // Unlock sequence — must write both values back-to-back
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
    // Disable watchdog — WDE bit cleared after unlock
    WDOG1_WCR &= ~(1 << 2);

    digitalWrite(STATUS_LED, LOW);

    // Enter deep sleep — wakes via reset on T4.0
    // Execution does not return past this point
    Snooze.deepSleep(snoozeConfig);
}

// ==================================================================================================================================
// VEHICLE -> CLUSTER PROCESSING
// ==================================================================================================================================

void processVehicleFrame(const CAN_message_t &msg) {

    processSteeringButtons(msg);

    CAN_message_t out = msg;

    switch (msg.id) {

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
    // Re-armed on every boot/wake. Must be done before deepSleep
    // can disable it again on the next standby entry.
    // --------------------------------------------------------

    WDT_timings_t config;

    config.trigger  = 5;
    config.timeout  = 6;
    config.callback = watchdogCallback;

    wdt.begin(config);

    // --------------------------------------------------------
    // CAN INIT
    // Must happen before Snooze pin config — initCAN() sets the
    // FlexCAN pad mux on GPIO_1 (CAN1/cluster) and GPIO_23
    // (CAN2/vehicle). GPIO_4 is a separate dedicated pin wired
    // in parallel to the VP230_Vehicle RXD line (GPIO_23 node).
    // --------------------------------------------------------

    initCAN();

    Serial.println("Retrofit Cluster Gateway started");

    // --------------------------------------------------------
    // SNOOZE WAKE PIN CONFIG
    // GPIO_4 = VP230_Vehicle RXD (parallel wire from GPIO_23 node)
    //
    // The vehicle bus is used as the sole wake source — vehicle
    // ECU activity is present immediately at ignition, providing
    // a reliable dominant edge. The cluster bus was found to be
    // slower to start sending frames after power-up.
    // FALLING = dominant bus bit → wake from deepSleep.
    // INPUT_PULLUP holds line high (recessive) when bus is idle.
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
//
// CAN frame processing happens in RX callbacks (IRQ context).
// loop() handles housekeeping and standby entry only.
//
// STANDBY SEQUENCE:
//   1. housekeepingISR accumulates vehicleIdleMs / clusterIdleMs
//   2. loop() detects both buses idle beyond SLEEP_TIMEOUT_MS
//   3. loop() calls enterDeepSleep()
//   4. Snooze gates PLLs, drops to ~9-14 mA, waits for wake edge
//   5. Wake edge (CAN dominant bit) → T4.0 resets → setup() reruns
// ==================================================================================================================================

void loop() {

    // --------------------------------------------------------
    // FEED WATCHDOG
    // --------------------------------------------------------

    wdt.feed();

    // --------------------------------------------------------
    // STANDBY ENTRY
    // Both buses must be idle for SLEEP_TIMEOUT_MS.
    // uptimeMs guard prevents sleep at boot before bus has settled.
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
    // sendUnitSetting() calls delay() so must run in loop()
    // --------------------------------------------------------

    if (doUnitConfig) {

        doUnitConfig = false;

        sendAllUnitSettings();
    }

    // --------------------------------------------------------
    // END OF MAIN LOOP :3
    // --------------------------------------------------------
}
