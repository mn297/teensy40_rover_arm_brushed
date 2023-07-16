#include <Arduino.h>
#include <SPI.h>
#include "rover_arm.h"
#include "RoverArmMotor.h"
#include "Teensy_PWM.h"
#include "AMT22.h"
#include <limits>
#include <IntervalTimer.h>
#include "TeensyDebug.h"

#if DEBUG_GDB_STUB == 1
// #pragma GCC optimize("O0")
#endif

IntervalTimer rover_arm_timer;

// No additional #include statements are needed
#define TICK 1

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()

extern RoverArmMotor Wrist_Roll;
extern RoverArmMotor Wrist_Pitch;

uint32_t sp_counter = 0;

void rover_arm_timer_routine()
{
#if TICK == 1
#if TEST_WRIST_ROLL_CYTRON == 1
  Wrist_Roll.tick();
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
  Wrist_Pitch.tick();
#else

#endif
#endif
}
// The setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
#if DEBUG_GDB_STUB == 1
  debug.begin(SerialUSB1);
  halt_cpu(); // stop on startup; if not, Teensy keeps running and you
#endif

  Serial.println("Starting up");
  SPI.begin(); // initiate SPI bus
  rover_arm_setup();
  delay(500);
  rover_arm_timer.begin(rover_arm_timer_routine, 50000); // blinkLED will be called every 100ms
  Serial.println("Setup done");
}

// the loop function runs over and over again forever
void loop()
{
  rover_arm_loop();
}

void serialEvent()
{
  while (Serial.available())
  {
    // Read the incoming string
    String incomingString = Serial.readStringUntil('\n');

    // Check if the incoming string starts with "setpoint"
    // Initialize parameters
    double param1, param2, param3;

    // Use sscanf to extract the parameters
    sscanf(incomingString.c_str(), "%lf %lf %lf", &param1, &param2, &param3);

    // Now you can use param1, param2, param3
    Serial.printf("Received angles: %f, %f, %f\r\n", param1, param2, param3);

    // Call newSetpoint() with the received angle
    // bool result = Wrist_Roll.newSetpoint(angle);
#if TEST_WRIST_ROLL_CYTRON == 1
    bool result1 = Wrist_Roll.newSetpoint(param1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    bool result2 = Wrist_Pitch.newSetpoint(param1);
#endif

    // Print status.
#if TEST_WRIST_ROLL_CYTRON == 1
    Serial.printf("Wrist_Roll newSetpoint at %lf result: %d\r\n", param1, result1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    Serial.printf("Wrist_Pitch newSetpoint at %lf result: %d\r\n", param2, result2);
#endif
  }
}