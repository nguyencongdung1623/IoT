#include <ESP32Servo.h>

// Define servo pin
#define SERVO_PIN 12

// Create Servo object
Servo myServo;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize
  }
  Serial.println("ESP32 Servo Control - Enter angle (0-180 degrees):");

  // Allow allocation of timers for ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set servo frequency to 50 Hz (standard for most servos)
  myServo.setPeriodHertz(50);

  // Attach servo to pin
  myServo.attach(SERVO_PIN, 500, 2400); // 500us min pulse, 2400us max pulse
  Serial.println("Servo initialized on pin " + String(SERVO_PIN));

  // Set initial position to 90 degrees
  myServo.write(90);
  Serial.println("Servo set to 90 degrees");
}

void loop() {
  // Check for Serial input
  if (Serial.available() > 0) {
    // Read input as string
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace

    // Convert input to integer
    int angle;
    try {
      angle = input.toInt();
      // Validate angle
      if (angle >= 0 && angle <= 180) {
        myServo.write(angle);
        Serial.println("Servo set to " + String(angle) + " degrees");
      } else {
        Serial.println("Error: Angle must be between 0 and 180 degrees");
      }
    } catch (...) {
      Serial.println("Error: Invalid input. Enter a number (0-180)");
    }
  }
}