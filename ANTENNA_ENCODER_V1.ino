#include <Arduino.h>

const int enablePin = 9;  // PWM pin for motor speed control
const int int1Pin = 8;    // Motor driver input 1
const int int2Pin = 7;    // Motor driver input 2

const int encoderPinA = 2;
const int encoderPinB = 3;
const int encoderPinZ = 4; // Z pulse pin

volatile long encoderValue = 0;
volatile long zPulseCount = 0;

int speed = 0;  // Global variable to hold speed
int targetPulses = 0; // Target pulses for motor to move

const int motorToEncoderRatio = 2; // 1:2 motor to encoder ratio
const int antennaToEncoderReduction = 5; // 5:1 antenna to encoder reduction
const int pulsesPerRotation = 10000; // Encoder pulses per full motor rotation

void setup() {
  Serial.begin(9600);

  pinMode(enablePin, OUTPUT);
  pinMode(int1Pin, OUTPUT);
  pinMode(int2Pin, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP); 
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinZ, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinZ), zPulseDetected, RISING);
}

void loop() {
  if (Serial.available() > 0) {  
    int angle = Serial.parseInt();  // Read the incoming angle as integer
    Serial.print("Received angle: ");
    Serial.println(angle);

    targetPulses = angleToPulses(angle);  // Convert angle to pulses
    Serial.print("Pulses required: ");
    Serial.println(targetPulses);

    // Wait for the next input
    while (Serial.available() == 0) {}

    speed = Serial.parseInt();  // Read the incoming speed as integer
    Serial.print("Received speed: ");
    Serial.println(speed);
  }

  if (Serial.available() > 0) {
    char command = Serial.read(); // Read command from Serial
    switch (command) {
      case 'F':  // Forward
        forward(speed, targetPulses);  // Call forward function with current 'speed'
        break;
      case 'B':  // Backward
        backward(speed, targetPulses);  // Call backward function with current 'speed'
        break;
      case 'S':  // Stop
        stopMotor();
        Serial.println("Stop motor");
        break;
      case 'D':
        directionCheck();
        break;
    }
  }

  Serial.print("Encoder Value: ");
  Serial.println(encoderValue);
  Serial.print("Z Pulse Count: ");
  Serial.println(zPulseCount);
  Serial.print("Calculated Angle: ");
  Serial.println(calculateAngle());
  delay(1000);
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  static int lastEncoded = 0;
  int sum  = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; // store this value for next time
}

void zPulseDetected() {
  zPulseCount++;
}

void forward(int speed, int pulses) {
  encoderValue = 0; // Reset encoder value
  while (encoderValue <= pulses) {
    analogWrite(enablePin, speed);  // Set motor speed
    digitalWrite(int1Pin, HIGH);    // Set direction
    digitalWrite(int2Pin, LOW);
  }
  stopMotor();
}

void backward(int speed, int pulses) {
  encoderValue = 0; // Reset encoder value
  while (encoderValue >= -pulses) {
    analogWrite(enablePin, speed);  // Set motor speed
    digitalWrite(int1Pin, LOW);     // Set direction
    digitalWrite(int2Pin, HIGH);
  }
  stopMotor();
}

void stopMotor() {
  digitalWrite(int1Pin, LOW);
  digitalWrite(int2Pin, LOW);
  analogWrite(enablePin, 0);  // Stop motor
}

void directionCheck() {
  static int previousEncoderValue = 0;

  if (encoderValue > previousEncoderValue) {
    Serial.println("Clockwise Direction");
  } else if (encoderValue < previousEncoderValue) {
    Serial.println("Anti-Clockwise Direction");
  } else {
    Serial.println("No Change in Direction");
  }
  previousEncoderValue = encoderValue;
}

int angleToPulses(int angle) {
  // Calculate the antenna rotation in degrees
  int antennaRotation = angle;

  // Calculate the encoder rotation in degrees
  int encoderRotation = antennaRotation / antennaToEncoderReduction;

  // Calculate the motor rotation in degrees
  int motorRotation = encoderRotation * motorToEncoderRatio;

  // Calculate the pulses required (assuming 10000 pulses per full motor rotation)
  int pulses = (motorRotation / 360.0) * pulsesPerRotation;

  return pulses;
}

int calculateAngle() {
  // Calculate the angle of rotation based on Z pulses
  int totalPulses = zPulseCount * pulsesPerRotation + encoderValue;
  int motorRotation = (totalPulses / pulsesPerRotation) * 360;
  int encoderRotation = motorRotation / motorToEncoderRatio;
  int antennaRotation = encoderRotation * antennaToEncoderReduction;

  return antennaRotation;
}
