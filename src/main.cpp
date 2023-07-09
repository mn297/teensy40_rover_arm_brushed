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
#define WRIST_ROLL_TESTBENCH 0

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()

extern RoverArmMotor Wrist_Roll;
extern RoverArmMotor Wrist_Pitch;

uint32_t sp_counter = 0;

void rover_arm_timer_routine()
{
#if WRIST_ROLL_TESTBENCH == 1
  int16_t turnCounter[2] = {0, 0};
  getTurnCounterSPI(turnCounter, CS1, 12);
  Serial.printf("encoder 1 gives %d %d\r\n", turnCounter[0], turnCounter[1]);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  double current_angle_sw = 0;
  Wrist_Roll.get_current_angle_sw(&current_angle_sw);
  Serial.printf("Wrist_Roll setpoint %.2f, angle_sw %.2f, output %.2f\r\n",
                Wrist_Roll.setpoint / Wrist_Roll.gearRatio,
                current_angle_sw / Wrist_Roll.gearRatio,
                Wrist_Roll.output);
#else
  Wrist_Roll.tick();
#if TEST_WRIST_PITCH_CYTRON == 1
  Wrist_Pitch.tick();
#endif
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

  rover_arm_setup();
  delay(500);
  rover_arm_timer.begin(rover_arm_timer_routine, 50000); // blinkLED will be called every 100ms
  Serial.println("Setup done");
}

// the loop function runs over and over again forever
void loop()
{
#if WRIST_ROLL_TESTBENCH == 1
  Wrist_Roll.forward();
  print_motor("Wrist_Roll", &Wrist_Roll);
  // Wrist_Pitch.forward();
  delay(2500);
  Wrist_Roll.reverse();
  // Wrist_Pitch.reverse();
  delay(1000);
#else
  rover_arm_loop();
#endif
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
    // Print status each one
#if TEST_WRIST_ROLL_CYTRON == 1
    Serial.printf("Wrist_Roll newSetpoint at %lf result: %d\r\n", param1, result1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    Serial.printf("Wrist_Pitch newSetpoint at %lf result: %d\r\n", param2, result2);
#endif
  }
}