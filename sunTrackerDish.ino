/*
  Solar Tracker
*/
#include <Arduino.h>
#include "DRV8834.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// All the wires needed for full functionality
// X motor
#define DIRX 2
#define STEPX 3
// Y motor
#define DIRY 4
#define STEPY 5

// microstep control for DRV8834
#define M0 10
#define M1 11

// Joystick control
const int joyH = 1;
const int joyV = 2;
const int joySW = 8;

// variable to read the value from the analog pin
int joyVal;

// Joystick button
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

DRV8834 stepperX(MOTOR_STEPS, DIRX, STEPX, M0, M1);
DRV8834 stepperY(MOTOR_STEPS, DIRY, STEPY, M0, M1);


void setup() {
  pinMode(joySW, INPUT);
  digitalWrite(joySW, HIGH);

  // Set target motor RPM.
  stepperX.setRPM(120);
  stepperY.setRPM(120);


  // Inizialize Serial
  Serial.begin(9600);
}

void loop() {
  // Display Joystick values using the serial monitor
  // outputJoystick();

  // Switch button
  buttonState = digitalRead(joySW);
  if (buttonPushCounter % 2) { // Auto Tracker is on
    Serial.println("on");
    // X axis
    // Read the horizontal joystick value  (value between 0 and 1023)
    joyVal = analogRead(joyH);
    joyVal = map(joyVal, 0, 1023, -360, 360);     // scale it to use it with the servo (result  between 0 and 180)

    // The easy way is just tell the motor to rotate 360 degrees at 1rpm
    Serial.println(joyVal);
    if (abs(joyVal) < 10) {
      // disable driver

    } else {
      // move
      stepperX.rotate(joyVal / 10);
    }
    // Y axis
    // Read the horizontal joystick value  (value between 0 and 1023)
    joyVal = analogRead(joyV);
    joyVal = map(joyVal, 0, 1023, -360, 360);     // scale it to use it with the servo (result  between 0 and 180)

    // The easy way is just tell the motor to rotate 360 degrees at 1rpm
    //  Serial.println(joyVal);
    if (abs(joyVal) < 10) {
      // disable driver

    } else {
      // move
      stepperY.rotate(joyVal / 10);
    }
  }
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == LOW) {
      // if the current state is HIGH then the button is pressed
      buttonPushCounter++;
      Serial.println(buttonPushCounter);
    } else {
      // normaly open button
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;
}

/**
  Display joystick values
*/
void outputJoystick() {
  Serial.print(analogRead(joyH));
  Serial.print ("---");
  Serial.print(analogRead(joyV));
  Serial.println ("----------------");
}
