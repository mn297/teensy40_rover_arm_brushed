#include <Arduino.h>
#include <SPI.h>
#include "rover_arm.h"
#include "RoverArmMotor.h"
#include "Teensy_PWM.h"
#include "AMT22.h"
#include <limits>
#include <IntervalTimer.h>
IntervalTimer rover_arm_timer;

// No additional #include statements are needed
Teensy_PWM *PWM1_instance;
Teensy_PWM *PWM2_instance;
Teensy_PWM *PWM3_instance;
#define WRIST_ROLL_TESTBENCH 1

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()

extern RoverArmMotor Wrist_Roll;

void rover_arm_timer_routine()
{
#if WRIST_ROLL_TESTBENCH == 1
  Wrist_Roll.tick();
#else
  int16_t turnCounter[2] = {0, 0};
  getTurnCounterSPI(turnCounter, CS1, 12);
  Serial.printf("encoder 1 gives %d %d\r\n", turnCounter[0], turnCounter[1]);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif

}
// The setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  SPI.begin(); // initiate SPI bus

  Serial.println("Starting up");

#if WRIST_ROLL_TESTBENCH == 1
  rover_arm_setup();
#endif
  rover_arm_timer.begin(rover_arm_timer_routine, 50000); // blinkLED will be called every 100ms
}

// the loop function runs over and over again forever
void loop()
{
  rover_arm_loop();

  if (Serial.available())
  {
    // Read the incoming string
    String incomingString = Serial.readStringUntil('\n');

    // Check if the incoming string starts with "setpoint"
    if (incomingString.startsWith("sp"))
    {
      // Extract the angle from the incoming string
      double angle = atof(incomingString.substring(9).c_str()); // 9 is the length of "setpoint"

      // Call newSetpoint() with the received angle
      bool result = Wrist_Roll.newSetpoint(angle);

      // Echo the result back to the serial port
      if (result) {
        Serial.printf("New setpoint at %f\r\n", Wrist_Roll.getSetpoint());
      } else {
        Serial.printf("FAILED, setpoint still at %f\r\n", Wrist_Roll.getSetpoint());
      }
    }
  }
}
