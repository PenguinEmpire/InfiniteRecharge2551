_Large portions of this document are based on the [2019 Control System Reference Sheet](https://blanchardproductions-my.sharepoint.com/:w:/g/personal/ryan_blanchard_productions/EbjsomFdO1lPs1G2WJyoRdkBPiQ-xWn0CZFPdU5zBWIp4Q?e=pJdnJ7), which was mostly written by Ryan Blanchard (@n00she)._

---

# Penguin Empire Control System 2020 Reference Sheet

## NI roboRIO Port Definitions

### PWM

### DIO

### Expected CAN IDs

### I²C

## OpenMesh OM5P-AC Radio and D-Link Switch Ethernet

- From roboRIO to Radio 18-24V POE (Closest to DC Power)

## Voltage Regulator Module (VRM)

### 12V/2A

- OpenMesh OM5P-AC Radio

## PDP Port Definitions

### <40 AMP Ports 0-3 and 12-15

Port | Usage
---- | -----
0 | Unused
1 | Unused
2 | Unused
3 | Unused
12 | Unused
13 | Unused
14 | Unused
15 | Unused

### <20 AMP Ports 4-11:

Port | Usage
---- | -----
4 | Unused
5 | Unused
6 | Unused
7 | Unused
8 | Unused
9 | Unused
10 | Unused
11 | Unused

## IPv4 Addressing

Mostly TODO

Device | Address
------ | -------
OpenMesh OM5P-AC Radio | `10.25.51.1` (Static) (TODO: Check)
NI roboRIO | `10.25.51.2` (DHCP) (TODO: Why not static?)
Limelight | `10.25.51.11` (Static)
Windows 10 Drivers Station | `10.25.51.`**`5`**; Netmask `255.0.0.0` (Static) (Must be 5 because of TX1 gstreamer scripts)

### Further detail on Limelight

`10.25.51.11:5801` for the config dashboard.

`10.25.51.11:5800` for just the camera stream.