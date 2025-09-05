#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>

#define CE_PIN   9
#define CSN_PIN  10

RF24 radio(CE_PIN, CSN_PIN);
MPU6050 mpu;

const byte address[6] = "00001";

int16_t ax, ay, az;
int16_t init_ax, init_ay, init_az;
int16_t last_ax, last_ay, last_az;
unsigned long lastMovementTime = 0;
const unsigned long stopThreshold = 1500;  // 5 seconds

unsigned char command = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(50000);

  // Set up LED pins
  pinMode(4, OUTPUT); // D4 (MPU failure LED)
  pinMode(5, OUTPUT); // D5 (Transmitter failure LED)
  pinMode(6, OUTPUT); // D6 (Data sent successfully LED)

  // Turn off all LEDs initially
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

  mpu.initialize();

  // Check if MPU is initialized
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    digitalWrite(4, HIGH);  // Turn on D4 when MPU initialization fails
    while (1);  // Stop further execution
  }

  // Read initial accelerometer values
  init_ax = mpu.getAccelerationX();
  init_ay = mpu.getAccelerationY();
  init_az = mpu.getAccelerationZ();

  // Set the last known accelerometer values to the initial values
  last_ax = init_ax;
  last_ay = init_ay;
  last_az = init_az;
  lastMovementTime = millis();  // Record the current time

  // Initialize the radio
  if (!radio.begin()) {
    Serial.println("Radio initialization failed!");
    digitalWrite(5, HIGH);  // Turn on D5 when transmitter initialization fails
    while (1);  // Stop further execution
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.stopListening();

  Serial.println("Transmitter ready");
}

void loop() {
  ax = mpu.getAccelerationX();
  ay = mpu.getAccelerationY();
  az = mpu.getAccelerationZ();

  Serial.print("ax: "); Serial.print(ax);
  Serial.print(" | ay: "); Serial.print(ay);
  Serial.print(" | az: "); Serial.println(az);

  // Check if there has been a change in acceleration values
  if (ax != last_ax || ay != last_ay || az != last_az) {
    last_ax = ax;
    last_ay = ay;
    last_az = az;
    lastMovementTime = millis();  // Reset the movement timer
  }

  // If no movement detected for more than 5 seconds, stop everything
  if (millis() - lastMovementTime > stopThreshold) {
    command = 0;  // Stop
    Serial.println("Command: Stopped due to inactivity");
  } else if (ay >= init_ay + 8000 && init_az - az < 4000) {
    command = 1;  // Move forward
    Serial.println("Command: Moving forward");
  } else if (ay <= init_ay - 6000 && init_az - az < 6000) {
    command = 2;  // Move backward
    Serial.println("Command: Moving backward");
  } else if (az <= init_az - 6000 && ax < init_ax + 1500) {
    command = 3;  // Turn left
    Serial.println("Command: Turning left");
  } else if (az <= init_az - 3000 && ax >= init_ax + 6000) {
    command = 4;  // Turn right
    Serial.println("Command: Turning right");
  } else {
    command = 0;  // Stop
    Serial.println("Command: Stopped");
  }

  // Try to send data and control LED based on success
  bool success = radio.write(&command, sizeof(command));

  if (success) {
    Serial.println("Message sent successfully");
    digitalWrite(6, HIGH);  // Turn on D6 when data is successfully transmitted
  } else {
    Serial.println("Message failed to send");
    digitalWrite(6, LOW);  // Turn off D6 if message fails
  }

  delay(250);
}
