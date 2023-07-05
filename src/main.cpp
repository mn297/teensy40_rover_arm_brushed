#include <Arduino.h>
#include <SPI.h>
#include "rover_arm.h"
#include "AMT22.h"
// No additional #include statements are needed
const int moistureSensorPin = A0;  // Change to your actual analog pin number

// The setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);
  SPI.begin(); // initiate SPI bus
}

// the loop function runs over and over again forever
void loop()
{

  // toggle LED
  // delayMicroseconds(1000000);
  // digitalWrite(LED_BUILTIN, 0);
  // Serial.printf("LED is %d\n", digitalRead(LED_BUILTIN));

  // delayMicroseconds(1000000);
  // Serial.printf("LED is %d\n", digitalRead(LED_BUILTIN));
  // digitalWrite(LED_BUILTIN, 1);

  // // Delay for a second so the text doesn't print too quickly
  // delayMicroseconds(1000000);
    // Read the value from the moisture sensor

  // Print the sensor value to the Serial Monitor
  // Wait for 1 second before the next reading
  delay(500);
}
