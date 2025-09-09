# Gesture Controlled Rover

A gesture-controlled robotic rover designed using an Arduino Nano, MPU6050 accelerometer sensor, and NRF24L01 wireless transceiver modules. The rover’s movement is intuitively controlled through hand gestures, with the accelerometer capturing motion data and transmitting it wirelessly to the rover for real-time navigation.

## Features

- Wireless communication using NRF24L01
- Hand gesture detection using MPU6050 accelerometer
- Visual feedback via onboard LEDs for both transmitter and receiver
- Timeout-based auto-stop for safety
- Simple motor control using L298N driver
- Custom command encoding and transmission protocol

## Components Used

### Transmitter

- Arduino Nano
- MPU6050 accelerometer
- NRF24L01 + PA/LNA (optional)
- 3x LEDs for status indicators
- Power source (Battery/USB)

### Receiver

- Arduino Nano
- NRF24L01
- L298N Motor Driver
- 2x DC Motors
- 2x LEDs for communication indicators
- Power source (Battery)

## Code

- `txmitter.ino`: Code for the transmitter (gesture detection and RF transmission)
- `rx.ino`: Code for the receiver (motor control based on commands)
