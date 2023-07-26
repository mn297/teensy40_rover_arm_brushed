#include <Arduino.h>
#include <SPI.h>
#include "rover_arm.h"
#include "RoverArmMotor.h"
#include "Teensy_PWM.h"
#include "AMT22.h"
#include <limits>
#include <IntervalTimer.h>
#include "TeensyDebug.h"
#include <EEPROM.h>
#include "DMAChannel.h"

#if DEBUG_GDB_STUB == 1
// #pragma GCC optimize("O0")
#endif

// No additional #include statements are needed

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()

extern RoverArmMotor Wrist_Roll;
extern RoverArmMotor Wrist_Pitch;
extern RoverArmMotor End_Effector;
extern RoverArmMotor Elbow;
extern RoverArmMotor Shoulder;

#if USE_DMA == 1
DMAChannel dmachannel1;
unsigned char DMA_TxBuf[50]; // transfer buffer 50
unsigned char DMA_RxBuf[50]; // receive buffer  50

void INT_DMA1(void)
{
  int i;
  dmachannel1.clearInterrupt();

  for (i = 0; i < 8; i++)
  {
    Serial.printf("%d\n\r", DMA_RxBuf[i]);
  }
}

void DMA_Init(void)
{
  //*****************DMA1****************************************************
  dmachannel1.begin();
  dmachannel1.source((uint8_t &)LPUART5_DATA);
  dmachannel1.destinationBuffer(DMA_RxBuf, 8);
  dmachannel1.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART5_RX);
  dmachannel1.interruptAtCompletion();
  dmachannel1.attachInterrupt(INT_DMA1);

  dmachannel1.enable();
}
#endif
double mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  double result = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

  return result;
}
void setup()
{
  // pinMode(ELBOW_BRAKE, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    ;
#if DEBUG_GDB_STUB == 1
  debug.begin(SerialUSB1);
  halt_cpu(); // stop on startup; if not, Teensy keeps running and you
#endif
#if MASTERING_TEST == 0
  rover_arm_setup();
#endif

#if MASTERING_TEST == 1
  // Mastering test.
  pinMode(CS1, OUTPUT);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  int error = -1;
  int16_t result_arr[2];
  while (error == -1)
  {
    error = getTurnCounterSPI(result_arr, CS1, 12); // timer not used, so nullptr
    if (error)
    {
      Serial.printf("setup() ERROR: %d\r\n", error);
      continue;
    }
    double angle_raw = mapFloat((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); // mn297 potentiometer encoder
    Serial.printf("setup() BEFORE angle: %.2f, turns: %d\r\n", angle_raw, result_arr[1]);
  }
  setZeroSPI(CS1);
  delay(1000);
  resetAMT22(CS1);
  delay(1000);
  error = -1;
  while (error == -1)
  {
    error = getTurnCounterSPI(result_arr, CS1, 12); // timer not used, so nullptr
    if (error == -1)
    {
      Serial.printf("setup() ERROR: %d\r\n", error);
      continue;
    }
    double angle_raw = mapFloat((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); // mn297 potentiometer encoder
    Serial.printf("setup() AFTER angle: %.2f, turns: %d\r\n", angle_raw, result_arr[1]);
  }
#endif
}

void loop()
{
#if MASTERING_TEST == 1
  // Mastering test.
  int16_t result_arr[2];
  delay(500);
  int error = getTurnCounterSPI(result_arr, CS1, 12); // timer not used, so nullptr
  if (error == -1)
  {
    Serial.printf("loop() ERROR: %d\r\n", error);
    return;
  }
  double angle_raw = mapFloat((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); // mn297 potentiometer encoder
  int turns = result_arr[1];
  double angle_multi = angle_raw + 360 * turns;
  Serial.printf("AFTER angle: %.2f, turns: %d, angle_multi %.2f\r\n", angle_raw, turns, angle_multi);
#endif

#if TICK == 0
  // Elbow.disengage_brake();
  // Serial.printf("Elbow disengage_brake\r\n");
  // delay(1000);
  // Elbow.engage_brake();
  // Serial.printf("Elbow engage_brake\r\n");
  // delay(1000);
  digitalWrite(DIR1, HIGH);
  delay(1000);
  digitalWrite(DIR1, LOW);
  delay(1000);
  Wrist_Roll.forward(25);
  Serial.printf("Wrist_Roll forward\r\n");
// Wrist_Roll.reverse(25);
// Serial.printf("Wrist_Roll reverse\r\n");
// delay(1000);
#endif

#if MASTERING_TEST == 0
#if TEST_LOOP == 1
  rover_arm_loop();
#endif
#endif
#if TEST_LIMIT_SWITCH == 1
  test_limit_switches();
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

    // EEPROM.
    byte val = EEPROM.read(0);
    Serial.printf("Read %p from EEPROM\r\n", val);
    EEPROM.write(0, param1);
    Serial.printf("Wrote %f to EEPROM\r\n", param1);

    // Call new_setpoint() with the received angle
    // bool result = Wrist_Roll.new_setpoint(angle);
#if TEST_WRIST_ROLL_CYTRON == 1
    bool result1 = Wrist_Roll.new_setpoint(param1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    bool result2 = Wrist_Pitch.new_setpoint(param1);
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
    bool result3 = End_Effector.new_setpoint(param1);
#endif
#if TEST_ELBOW_SERVO == 1
    bool result4 = Elbow.new_setpoint(param1);
#endif
#if TEST_SHOULDER_SERVO == 1
    bool result5 = Shoulder.new_setpoint(param1);
#endif

    // Print status.
#if TEST_WRIST_ROLL_CYTRON == 1
    Serial.printf("Wrist_Roll new_setpoint at %lf result: %d\r\n", param1, result1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    Serial.printf("Wrist_Pitch new_setpoint at %lf result: %d\r\n", param2, result2);
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
    Serial.printf("End_Effector new_setpoint at %lf result: %d\r\n", param3, result3);
#endif
#if TEST_ELBOW_SERVO == 1
    Serial.printf("Elbow new_setpoint at %lf result: %d\r\n", param1, result4);
#endif
#if TEST_SHOULDER_SERVO == 1
    Serial.printf("Shoulder new_setpoint at %lf result: %d\r\n", param1, result5);
#endif
  }
}