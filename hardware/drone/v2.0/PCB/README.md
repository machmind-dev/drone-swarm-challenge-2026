# Swarm Stack PCB

Schematics and board design: **Autodesk Eagle** (`.sch` / `.brd`)

<table><tr>
<td><img src="pcb-1.jpg" width="400"></td>
<td><img src="pcb-2.jpg" width="400"></td>
</tr></table>

## ToF Sensors (VL53L1X)

| Sensor | XSHUT GPIO | Direction |
|--------|------------|-----------|
| S0 | GPIO4 | Left Hand (LH) |
| S1 | GPIO20 | LH 45° |
| S2 | GPIO21 | Forward |
| S3 | GPIO26 | 45° RH |
| S4 | GPIO32 | Right Hand (RH) |
| S5 | GPIO27 | Upwards |

## I2C (ESP32-P4)

| Signal | GPIO |
|--------|------|
| SDA | GPIO2 |
| SCL | GPIO3 |

**Note 1:** The 1K Ohm limiting resistor for LED strip control is not populated on the PCB — add it externally on the signal line.

**Note 2:** BEC must share a common ground with the Swarm Stack.

**Note 3:** Pinout was modified in the latest version — Waveshare devboard ESP32-P4 with and without WiFi module have different pinouts.
