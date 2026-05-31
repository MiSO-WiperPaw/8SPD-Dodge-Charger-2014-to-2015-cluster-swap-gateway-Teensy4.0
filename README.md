# 8SPD Dodge Charger 2014 - 2015 cluster swap CANBus gateway Teensy4.0

This project aims to retrofit newer 2015+ 
instrument cluster panels into 2011-2014
dodge chargers equipped with the automatic 8
speed transmission and electronic shifter/range select 
with full compatibility. The clusters are largely compatible, however
some crucial and comfort features are missing/incompatible with the older vehicles.
This project serves as a CAN-C bus gateway/translator to add and translate the missing features.

# Hardware:

- Teensy T4

- 2x VP230 (or compatible) CAN Bus transcievers

- Meshnology BE-220 GPS+Compass module (to add proper timekeeping and compass Heading info)

- 12->5v DC-DC converter (Automotive grade, ideally on-board and quite stable)

- TVS Dioides (on the CAN transcievers. technically optional, but HIGHLY recommended for permanent installs) 

# Currently implemented features:

- sports mode display

- autostick display

- manual mode display

- actual gear display

- region/units change

- steering wheel button remapping

> **Coming Soon!:**
> 
> GPS time keeping
> 
> Compass heading display


# Notes:
> Older firmware and pursuit clusters do NOT support sport mode and actual gear display

> -AlfaOBD is required

> -Shifter type must be set to "MS7S" and sport mode enabled via AlfaOBD

>-The mod will work without paddle shifters, but it is highly recommended to install and enable via AlfaOBD

>-2015+ Steering wheel and shifter swap mods coming soon! :3

>-The 'OK' button is mapped to the ACC cruise button, recommended to swap that ID if you have ACC equipped car
 >or remove the 23A decoder and map the OK button to be triggered on a button combination (ex. up+down arrows),
 >but once the steering wheel swap mod is also done, that issue will be solved and implemented that way by default.

# Connection map:

| Teensy | VP230_Cluster |
|---|---|
| GPIO_0     |     TX   (CAN2 TX)  |
| GPIO_1     |     RX   (CAN2 RX)  |

| Teensy | VP230_Vehicle |
|---|---|
| GPIO_22     |     TX   (CAN1 TX)  |
| GPIO_23     |     RX   (CAN1 RX)  |
| GPIO_4      |     RX   (parallel — Snooze deepSleep wake pin) |

> **Hardware note:**
>
> - GPIO_4 is wired in parallel with GPIO_23 (short jumper wire
>   on the same node). The vehicle bus is used as the sole wake
>   source — vehicle ECU frames are present immediately at
>   ignition providing a reliable dominant edge.
>   
> - FlexCAN owns GPIO_23 via pad mux so it cannot serve as a
>   Snooze GPIO wake pin. GPIO_4 is a dedicated free GPIO.

> GPS timekeeping & compass heading (Soon)
>
> | Teensy:     |    BN-220: |
> |---|---|
> | NA         |     NA |
> | NA         |     NA |
> | NA         |     NA |
> | NA         |     NA |
 
# Low power notes:

 - deepSleep target: ~9-14 mA (measured, Teensyduino 1.52+)
  
 - Wake source: FALLING edge on GPIO_4 (vehicle bus)
   
 - deepSleep on T4.0 wakes via reset — setup() reruns on wake.
   All state reinitialises cleanly each time.
   
 - STARTUP_GRACE_MS > SLEEP_TIMEOUT_MS prevents re-sleep before
   the bus has fully settled after wake.
   
 - WDT_T4 is disabled before deepSleep to prevent Snooze from
   triggering a spurious watchdog reset during sleep.
   Watchdog re-arms at the top of setup() on every boot/wake.
----------------------------------
