#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 pin assignments for each finger
// Order: Index, Middle, Ring, Pinky, Thumb
int fingerPins[] = {0, 1, 2, 3, 4}; 

// Safe pulse limits for servos (Standard range: 150-600 for most 180° servos)
int initialPulse = 300; 

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos typically run at 60Hz
  
  // --- SOFT START SEQUENCE ---
  // Gradually moves all fingers to the starting position.
  // This prevents high current draw and potential power dips when powered on.
  for (int p = 0; p < 5; p++) {
    for (int i = 0; i < 50; i++) { // Iterative steps for smooth motion
      pwm.setPWM(fingerPins[p], 0, initialPulse);
    }
    delay(100); // Short delay between each motor initialization
  }
  
  Serial.println("READY"); // Notify Python script that the system is ready
}

void loop() {
  // Wait for a full 10-byte data packet from Python
  if (Serial.available() >= 10) {
    for (int i = 0; i < 5; i++) {
      int highByte = Serial.read();
      int lowByte = Serial.read();
      
      // Reconstruct 16-bit pulse value from two 8-bit bytes
      int pulse = (highByte << 8) | lowByte; 
      
      // Data validation (PCA9685 supports a maximum resolution of 4095)
      if (pulse >= 0 && pulse <= 4095) {
        pwm.setPWM(fingerPins[i], 0, pulse);
      }
    }
    
    // Buffer clearing: Discard extra bytes if more than 10 are available.
    // This is critical to minimize latency and prevent lag.
    while(Serial.available() > 10) { Serial.read(); }
  }
}
