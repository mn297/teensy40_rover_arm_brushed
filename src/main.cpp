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
#define TICK 1

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()

extern RoverArmMotor Wrist_Roll;
extern RoverArmMotor Wrist_Pitch;
extern RoverArmMotor End_Effector;
extern RoverArmMotor Elbow;

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

  rover_arm_setup();
}

// the loop function runs over and over again forever
void loop()
{
#if TEST_LOOP == 1
  rover_arm_loop();
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

    // Call newSetpoint() with the received angle
    // bool result = Wrist_Roll.newSetpoint(angle);
#if TEST_WRIST_ROLL_CYTRON == 1
    bool result1 = Wrist_Roll.newSetpoint(param1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    bool result2 = Wrist_Pitch.newSetpoint(param1);
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
    bool result3 = End_Effector.newSetpoint(param1);
#endif
#if TEST_ELBOW_SERVO == 1
    bool result4 = Elbow.newSetpoint(param1);
#endif

    // Print status.
#if TEST_WRIST_ROLL_CYTRON == 1
    Serial.printf("Wrist_Roll newSetpoint at %lf result: %d\r\n", param1, result1);
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    Serial.printf("Wrist_Pitch newSetpoint at %lf result: %d\r\n", param2, result2);
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
    Serial.printf("End_Effector newSetpoint at %lf result: %d\r\n", param3, result3);
#endif
#if TEST_ELBOW_SERVO == 1
    Serial.printf("Elbow newSetpoint at %lf result: %d\r\n", param1 , result4);
#endif
  }
}