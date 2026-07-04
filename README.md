
# RPOD Target Spacecraft: Primary Flight Software

## Overview

This repository contains the primary flight software for the independent Target Spacecraft within the Rendezvous, Proximity Operations, and Docking (RPOD) mission architecture. Operating on an ESP32 micro-architecture, this codebase governs the standalone target vehicle, acting as the dynamic entity that the chaser spacecraft must autonomously track and dock with.

The software streams real-time 6-DoF inertial telemetry and odometry to a centralized mission server via secure WebSockets, while synchronously executing telecommands to emulate target spin, stabilization, or tumbling scenarios.

## Flight System Features

* **Standalone Telemetry Downlink:** Streams MPU6050 (Accel/Gyro) inertial data and hardware-interrupt-driven encoder counts at 20Hz (50ms intervals) to Mission Control.
* **Remote Telecommand Execution:** Parses incoming JSON payloads over WebSockets to actively drive the target's rotational/mobility mechanisms via PWM.
* **Anti-Collision Failsafes:** Implements a strict connection watchdog. If the WebSocket link to the mission server drops, the firmware automatically triggers an emergency halt to all motor actuation. This prevents uncommanded target rotation, safeguarding the approaching chaser's docking mechanisms from catastrophic collision.
* **Deterministic I2C Polling:** Utilizes manual I2C registry access for the IMU to bypass heavy external libraries, minimizing loop latency and ensuring deterministic sensor reads.

---

## Hardware Architecture

### Avionics Pin Mapping

| Component | Pin Function | ESP32 Pin | Notes |
| --- | --- | --- | --- |
| **MPU6050 (IMU)** | SDA | `GPIO 21` | I2C Data (400kHz Fast Mode) |
|  | SCL | `GPIO 22` | I2C Clock |
| **Actuation Driver** | IN1 (Dir 1) | `GPIO 16` | Digital Output |
|  | IN2 (Dir 2) | `GPIO 17` | Digital Output |
|  | EN / PWM | `GPIO 18` | 8-bit PWM @ 5000Hz |
| **Quadrature Encoder** | Phase A | `GPIO 14` | Hardware Interrupt (RISING) |
|  | Phase B | `GPIO 15` | Digital Input |

### Dependencies

This project requires the Arduino framework for ESP32 and the following core libraries:

* `WiFi.h` & `WiFiMulti.h` (Built-in ESP32)
* `Wire.h` (Built-in I2C)
* `WebSocketsClient.h` (Requires [WebSockets by Markus Sattler](https://github.com/Links2004/arduinoWebSockets))

---

## Network & Ground Segment Architecture

The Target Spacecraft connects to the local operational network and establishes an encrypted WebSocket (WSS) uplink/downlink to the cloud-hosted mission dashboard.

* **WiFi SSID:** `AL AML D2`
* **WebSocket Host:** `targetdata.onrender.com`
* **Port:** `443` (SSL/TLS)
* **Endpoint:** `/esp32`

*Note: Update the `WIFI_PASSWORD` macro in the source code prior to deployment.*

---

## Telemetry & Telecommand Protocol (JSON)

Data exchange between the Target Spacecraft and the mission server utilizes lightweight JSON payloads, ensuring low overhead and immediate parsing compatibility with the broader ROS2 ecosystem.

### Downlink: Telemetry Payload (Target → Server)

Emitted at 20Hz when the WebSocket is established.

```json
{
  "ax": 0.012, 
  "ay": -0.981, 
  "az": 0.055,
  "gx": 1.230, 
  "gy": -0.450, 
  "gz": 0.110,
  "enc": 10452,
  "motorOn": true,
  "motorSpeed": 160
}

```

### Uplink: Remote Commands (Server → Target)

The flight software listens for specific command structures to manipulate the target's physical state or reset its odometry prior to the final rendezvous phase.

**1. Actuation Control**
Dictate the target's physical spin/movement state by toggling the motor and setting the PWM speed (0-255).

```json
{
  "type": "motor",
  "on": true,
  "speed": 200
}

```

**2. Odometry Reset**
Zero out the accumulated encoder count to establish a fresh zero-datum before the chaser initiates terminal docking procedures.

```json
{
  "type": "encoder_reset"
}

```

---

## Critical Safety & Fault Handling

* **Interrupt Concurrency:** To prevent memory tearing during simultaneous read/writes, the encoder count is marked `volatile` and read exclusively inside a `noInterrupts()` block within the main control loop.
* **Loss of Signal (LOS) Protocol:** If the `WStype_DISCONNECTED` event is triggered, the `motorStop()` function is immediately invoked. This enforces a cooperative, static state for the target vehicle during a communication blackout, ensuring it does not unpredictably alter its pose while the chaser is docking to prevent breaking the (very very) expensive chaser arms(wouldn't wanna be killed by our GP's supervisor).
