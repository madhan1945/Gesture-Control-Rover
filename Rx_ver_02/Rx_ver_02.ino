#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN  10

// Motor control pins (L298N) - Ensure these are PWM-capable pins
#define IN1  2
#define IN2  3
#define IN3  4
#define IN4  5
#define ENA  6  // Enable pin for motor 1
#define ENB  7  // Enable pin for motor 2

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
unsigned char receivedCommand = 0;

// Speed values for different actions
const int speedForwardBackward = 255 * 60 / 100;  // 75% of 255
const int speedTurning = 255 * 50 / 100;          // 50% of 255

// Variable to track the last time a command was received
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 500; // 500 ms timeout

void setup() {
  Serial.begin(9600);

  if (!radio.begin()) {
    Serial.println("nRF24L01 failed to initialize!");
    while (1);
  }

  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(A0, OUTPUT);  // LED on A0
  pinMode(A1, OUTPUT);  // LED on A1

  stopMotors();
  Serial.println("Receiver ready, waiting for commands...");
}

void loop() {
  if (radio.available()) {
    digitalWrite(A1, HIGH);  // Turn on A1 LED when receiving data
    digitalWrite(A0, LOW);
    radio.read(&receivedCommand, sizeof(receivedCommand));

    Serial.print("Received Command: ");
    Serial.println(receivedCommand);

    lastCommandTime = millis();  // Reset the timer on receiving a command

    // Act based on the received command
    switch (receivedCommand) {
      case 1:
        moveForward();
        Serial.println("Moving Forward");
        break;
      case 2:
        moveBackward();
        Serial.println("Moving Backward");
        break;
      case 3:
        turnRight();
        Serial.println("Turning Right");
        break;
      case 4:
        turnLeft();
        Serial.println("Turning Left");
        break;
      default:
        stopMotors();
        Serial.println("Stopped");
        break;
    }
  } 

  // Check if 500 ms have passed since the last command
  if (millis() - lastCommandTime > commandTimeout) {
    receivedCommand = -1;
    stopMotors();  // Stop the motors if no command received in time
    digitalWrite(A1, LOW);   // Turn off A1 LED when no data is being received
    digitalWrite(A0, HIGH);  // Turn on A0 LED when no transmission is received
  }

  // If no command received for a while, turn off A0 LED
  if (receivedCommand == -1) {
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);   // Turn off A1 LED when no data is being received
    digitalWrite(A0, HIGH);  // Turn on A0 LED when no transmission is received
  }
}

void moveForward() {
  analogWrite(ENA, speedForwardBackward);
  analogWrite(ENB, speedForwardBackward);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, speedForwardBackward);
  analogWrite(ENB, speedForwardBackward);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  analogWrite(ENA, speedTurning);
  analogWrite(ENB, speedTurning);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, speedTurning);
  analogWrite(ENB, speedTurning);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
