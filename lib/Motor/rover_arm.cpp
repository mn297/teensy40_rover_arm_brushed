#include "rover_arm.h"
// Drivers.
#include "RoverArmMotor.h"
#include "AMT22.h"
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

// Standard includes.
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <bitset>
#include <limits>

// Comms includes.
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

/*---------------------WRIST_ROLL_CYTRON---------------------*/
#if TEST_WRIST_ROLL_CYTRON == 1
RoverArmMotor Wrist_Roll(PWM1, DIR1, CS1, CYTRON, WRIST_ROLL_MIN_ANGLE, WRIST_ROLL_MAX_ANGLE);
#endif

/*---------------------WRIST_PITCH_CYTRON---------------------*/
#if TEST_WRIST_PITCH_CYTRON == 1
RoverArmMotor Wrist_Pitch(PWM2, DIR2, CS2, CYTRON, WRIST_PITCH_MIN_ANGLE, WRIST_PITCH_MAX_ANGLE);
#endif

/*---------------------END_EFFECTOR_CYTRON---------------------*/
#if TEST_END_EFFECTOR_CYTRON == 1
RoverArmMotor End_Effector(&hspi1, CYTRON_PWM_1, CYTRON_DIR_1, AMT22_1, CYTRON, 0, 359.99f);
#endif

/*---------------------ELBOW_SERVO DECLARATIONS---------------------*/
#if TEST_ELBOW_SERVO == 1
RoverArmMotor Elbow(PWM2, -1, CS1, BLUE_ROBOTICS, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
#endif

/*---------------------SHOULDER_SERVO DECLARATIONS---------------------*/
#if TEST_SHOULDER_SERVO == 1
RoverArmMotor Shoulder(PWM1, -1, CS2, BLUE_ROBOTICS, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
#endif

/*---------------------WAIST_SERVO DECLARATIONS---------------------*/
#if TEST_WAIST_SERVO == 1
RoverArmMotor Waist(PWM3, -1, CS3, BLUE_ROBOTICS, WAIST_MIN_ANGLE, WAIST_MAX_ANGLE);
#endif

/*--------------------- ROS --------------------*/
ros::NodeHandle nodeHandler;
std_msgs::Float32MultiArray feedback;
std_msgs::String motor_msg;
void commandCallback(const std_msgs::Float32MultiArray &command);

#if BRUSHLESS_ARM == 1
ros::Publisher ArmFeedback("arm12FB", &motor_msg);
ros::Publisher ArmFeedback_Debug("arm12FB_debug", &motor_msg);
ros::Subscriber<std_msgs::Float32MultiArray> ArmCommand("arm12Cmd", commandCallback);
#elif BRUSHED_ARM == 1
ros::Publisher ArmFeedback("arm24FB", &feedback);
ros::Publisher ArmFeedback_Debug("arm24FB_debug", &motor_msg);
ros::Subscriber<std_msgs::Float32MultiArray> ArmCommand("arm24Cmd", commandCallback);
#endif

// Source: https://mcgill.sharepoint.com/sites/McGillRobotics_Group/Shared%20Documents/Forms/AllItems.aspx?id=%2Fsites%2FMcGillRobotics%5FGroup%2FShared%20Documents%2FRover%20Project%2FSoftware%2FROS%20Message%20Instructions%2Epdf&parent=%2Fsites%2FMcGillRobotics%5FGroup%2FShared%20Documents%2FRover%20Project%2FSoftware&p=true&ct=1690126169268&or=Teams%2DHL&ga=1
void commandCallback(const std_msgs::Float32MultiArray &command)
{
    char feedback_buffer[256];
#if BRUSHLESS_ARM == 1
    Elbow.new_setpoint((double)command.data[0]);
    Shoulder.new_setpoint((double)command.data[1]);
    Waist.new_setpoint((double)command.data[2]);

    feedback.data_length = 3;

    noInterrupts();
    feedback.data[0] = (float)Elbow.currentAngle;
    feedback.data[1] = (float)Shoulder.currentAngle;
    feedback.data[2] = (float)Waist.currentAngle;
    interrupts();

#elif BRUSHED_ARM == 1
    // End_Effector.new_setpoint((double)command.data[0]);
    // Wrist_Roll.new_setpoint((double)command.data[1]);
    Wrist_Pitch.new_setpoint((double)command.data[2]);

    feedback.data_length = 3;

    noInterrupts();
    // feedback.data[0] = (float)End_Effector.currentAngle;
    // feedback.data[1] = (float)Wrist_Roll.currentAngle;
    // feedback.data[2] = (float)Wrist_Pitch.currentAngle;
    feedback.data[0] = 0;
    feedback.data[1] = 0;
    feedback.data[2] = 0;
    interrupts();
#endif

    ArmFeedback.publish(&feedback);
}

#define MIN_FLOAT -std::numeric_limits<float>::infinity()
#define MAX_FLOAT std::numeric_limits<float>::infinity()
static void attach_all_interrupts();

double setpoint = 0;
int turn = 0;
int limit_set = 0;
int button_counter = 0;
int is_turning = 0;

// LOOP control variables.
IntervalTimer rover_arm_timer;
volatile bool spiLock = false;     // Lock for SPI
volatile bool tickRequest = false; // Indicates if tick() wants to use SPI

void print_motor(char *msg, void *pMotor)
{
    char buffer[512]; // Ensure this is large enough to hold the entire formatted string

#if TICK == 0
    double current_angle_sw;
    ((RoverArmMotor *)pMotor)->get_current_angle_sw(&current_angle_sw);
#endif

    sprintf(buffer, "%s sp %.2f, angle_sw %.2f, angle_multi %.2f, angle_raw %.2f, turns %d, output %.2f, zero_angle_sw %.2f, gear_ratio %.2f",
            msg,
            ((RoverArmMotor *)pMotor)->setpoint,
#if TICK
            ((RoverArmMotor *)pMotor)->currentAngle,
#else
            current_angle_sw,
#endif
            ((RoverArmMotor *)pMotor)->current_angle_multi,
            ((RoverArmMotor *)pMotor)->_angle_raw,
            ((RoverArmMotor *)pMotor)->_turns,
            ((RoverArmMotor *)pMotor)->output,
            ((RoverArmMotor *)pMotor)->zero_angle_sw,
            ((RoverArmMotor *)pMotor)->gear_ratio);

    if (((RoverArmMotor *)pMotor)->encoder_error)
    {
        strcat(buffer, " (ERROR)\r\n");
    }
    else
    {
        strcat(buffer, "\r\n");
    }

    motor_msg.data = buffer;
    ArmFeedback_Debug.publish(&motor_msg);
}

void rover_arm_timer_routine()
{
    noInterrupts();
    if (spiLock)
    {
        // If SPI is currently in use, set the tickRequest flag.
        tickRequest = true;
        interrupts();
        return;
    }
    spiLock = true;
    interrupts();
#if TICK == 1
#if TEST_WRIST_ROLL_CYTRON == 1
    Wrist_Roll.tick();
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
    Wrist_Pitch.tick();
#endif
#if TEST_END_EFFECTOR_CYTRON == 1
    End_Effector.tick();
#endif
#if TEST_ELBOW_SERVO == 1
    Elbow.tick();
#endif
#if TEST_SHOULDER_SERVO == 1
    Shoulder.tick();
#endif
#if TEST_WAIST_SERVO == 1
    Waist.tick();
#endif
#endif
    spiLock = false;
    tickRequest = false;
}

void rover_arm_setup(void)
{
    // Serial.println("Starting up");

    SPI.begin(); // initiate SPI bus
    SPI.setClockDivider(SPI_CLOCK_DIV64);

    /*---------------------ROS---------------------*/
    // Initialize ROS Node and advertise the feedback publisher.
    nodeHandler.initNode();
    nodeHandler.advertise(ArmFeedback);
    nodeHandler.advertise(ArmFeedback_Debug);
    nodeHandler.subscribe(ArmCommand);

    /*---WRIST_ROLL_CYTRON setup---*/
#if TEST_WRIST_ROLL_CYTRON == 1
    Wrist_Roll.wrist_waist = 1;
    Wrist_Roll.set_gear_ratio(WRIST_ROLL_GEAR_RATIO);
    Wrist_Roll.setAngleLimits(WRIST_ROLL_MIN_ANGLE, WRIST_ROLL_MAX_ANGLE);
    // Wrist_Roll.stop_tick = 1;
    Wrist_Roll.begin(REG_KP_WRIST_ROLL, REG_KI_WRIST_ROLL, REG_KD_WRIST_ROLL,
                     REG_KP_WRIST_ROLL_AGG, REG_KI_WRIST_ROLL_AGG, REG_KD_WRIST_ROLL_AGG);
    // Assume at zero angle at startup.
    Wrist_Roll.set_current_as_zero_angle_sw();
    Wrist_Roll.new_setpoint(0.0);
#endif

    /*---WRIST_PITCH_CYTRON setup---*/
#if TEST_WRIST_PITCH_CYTRON == 1
    Wrist_Pitch.wrist_waist = 0;
    Wrist_Pitch.set_gear_ratio(WRIST_PITCH_GEAR_RATIO);
    Wrist_Pitch.setAngleLimits(WRIST_PITCH_MIN_ANGLE, WRIST_PITCH_MAX_ANGLE);
    // Wrist_Pitch.reset_encoder();
    // Wrist_Pitch.stop_tick = 1;
    Wrist_Pitch.set_safety_pins(-1, LIMIT_WRIST_PITCH_MAX, LIMIT_WRIST_PITCH_MIN);
    Wrist_Pitch.begin(REG_KP_WRIST_PITCH, REG_KI_WRIST_PITCH, REG_KD_WRIST_PITCH,
                      REG_KP_WRIST_PITCH_AGG, REG_KI_WRIST_PITCH_AGG, REG_KD_WRIST_PITCH_AGG);
    Wrist_Pitch.set_current_as_zero_angle_sw(WRIST_PITCH_ZERO_ANGLE);
    Wrist_Pitch.new_setpoint(0.0);
#endif

    /*---END_EFFECTOR_CYTRON setup---*/
#if TEST_END_EFFECTOR_CYTRON == 1
    End_Effector.wrist_waist = 0;
    End_Effector.setAngleLimits(MIN_FLOAT, MAX_FLOAT);
    // End_Effector.reset_encoder();
    End_Effector.begin(regKp_end_effector, regKi_end_effector, regKd_end_effector);
    End_Effector.reverse(100);
#if SKIP_MASTERING == 1
    End_Effector.stop();
    End_Effector.set_current_as_zero_angle_sw();
    End_Effector.new_setpoint(0.0);
#endif
#endif

    /* ELBOW_SERVO setup */
#if TEST_ELBOW_SERVO == 1
    Elbow.setAngleLimits(ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
    // Elbow.reset_encoder();
    Elbow.stop_tick = 0;
    Elbow.fight_gravity = 1;
    Elbow.set_safety_pins(ELBOW_BRAKE, LIMIT_ELBOW_MAX, LIMIT_ELBOW_MIN);
    Elbow.begin(REG_KP_ELBOW, REG_KI_ELBOW, REG_KD_ELBOW, REG_KP_ELBOW_AGG, REG_KI_ELBOW_AGG, REG_KD_ELBOW_AGG);
    Elbow.reset_encoder();
    Elbow.set_zero_angle();
    Elbow.set_current_as_zero_angle_sw(ELBOW_ZERO_ANGLE);
    Elbow.new_setpoint(0.0);
#endif

    /* SHOULDER_SERVO setup */
#if TEST_SHOULDER_SERVO == 1
    Shoulder.setAngleLimits(SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
    // Shoulder.reset_encoder();
    Shoulder.set_safety_pins(SHOULDER_BRAKE, -1, -1);
    Shoulder.begin(REG_KP_SHOULDER, REG_KI_SHOULDER, REG_KD_SHOULDER, REG_KP_SHOULDER_AGG, REG_KI_SHOULDER_AGG, REG_KD_SHOULDER_AGG);
    Shoulder.set_current_as_zero_angle_sw(SHOULDER_ZERO_ANGLE);
    Shoulder.new_setpoint(0.0);
#endif

    /*---WAIST_SERVO setup---*/
#if TEST_WAIST_SERVO == 1
    Waist.wrist_waist = 1;
    Waist.setAngleLimits(WAIST_MIN_ANGLE, WAIST_MAX_ANGLE);
    // Waist.reset_encoder();
    Waist.begin(regKp_waist, regKi_waist, regKd_waist);
#endif

    attach_all_interrupts();
    delay(250);
    rover_arm_timer.begin(rover_arm_timer_routine, PID_PERIOD_US);
    // Serial.println("Setup done, tick() timer started");
}

void rover_arm_loop()
{
    noInterrupts();
    if (tickRequest)
    {
        // If tick() wants to access SPI, defer the SPI access
        // from the main loop and handle it in tick().
        interrupts();
        return;
    }
    spiLock = true;
    interrupts();

    static unsigned long ros_loop = 0;      // Initialize lastPrint variable
    static unsigned long print_loop = 0;    // Initialize lastPrint variable
    unsigned long currentMillis = millis(); // get the current "time"

    if (currentMillis - ros_loop >= ROS_LOOP_PERIOD_MS)
    {
        nodeHandler.spinOnce();
        if (currentMillis - print_loop >= PRINT_LOOP_PERIOD_MS)
        {
#if DEBUG_PRINT_MOTOR == 1
#if TEST_WRIST_ROLL_CYTRON == 1
            print_motor("SP WRIST_ROLL_CYTRON", &Wrist_Roll);
#endif

#if TEST_WRIST_PITCH_CYTRON == 1
            print_motor("SP WRIST_PITCH_CYTRON", &Wrist_Pitch);
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
            print_motor("SP END_EFFECTOR_CYTRON", &End_Effector);
#endif

#if TEST_ELBOW_SERVO == 1
            print_motor("SP Elbow", &Elbow);
#endif

#if TEST_SHOULDER_SERVO == 1
            print_motor("SP Shoulder", &Shoulder);
#endif

#if TEST_WAIST_SERVO == 1
            print_motor("SP Waist", &Waist);
#endif

#endif
            print_loop = currentMillis;
        }
        ros_loop = currentMillis;
    }
    spiLock = false;
}

void test_limit_switches()
{
    static unsigned long lastPrint = 0;     // Initialize lastPrint variable
    unsigned long currentMillis = millis(); // get the current "time"

    if (currentMillis - lastPrint >= 200)
    {
#if TEST_WRIST_ROLL_CYTRON == 1
        Serial.printf("low: %d\r\n", digitalRead(LIMIT_WRIST_PITCH_MIN));
#endif
#if TEST_WRIST_PITCH_CYTRON == 1
        // Serial.printf("high: %d\r\n", digitalRead(LIMIT_WRIST_PITCH_MAX));
#endif
        lastPrint = currentMillis; // Update the lastPrint time
    }
}

#define DEBOUNCE_DELAY 250 // Delay for 500 ms. Adjust as needed.

volatile unsigned long last_trigger_time_pitch_max = 0;
volatile unsigned long last_trigger_time_pitch_min = 0;
volatile unsigned long last_trigger_time_end_effector_max = 0;
volatile unsigned long last_trigger_time_end_effector_min = 0;

volatile int limit_wrist_pitch_max_activated = 0;
volatile int limit_wrist_pitch_min_activated = 0;
volatile int limit_end_effector_max_activated = 0;
volatile int limit_end_effector_min_activated = 0;

#if TEST_WRIST_PITCH_CYTRON == 1
void limit_wrist_pitch_max_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_pitch_max > DEBOUNCE_DELAY)
    {
        last_trigger_time_pitch_max = now;
        if (digitalRead(LIMIT_WRIST_PITCH_MAX) == LOW)
        {
            limit_wrist_pitch_max_activated = 1;
            // Serial.println("Wrist pitch max limit reached");
            Wrist_Pitch.stop();
            // Wrist_Pitch.set_current_as_angle_sw(Wrist_Pitch.max_angle);
            Wrist_Pitch.new_setpoint(Wrist_Pitch.setpoint - 5.0f);
        }
        else
        {
            limit_wrist_pitch_max_activated = 0;
        }
    }
}

void limit_wrist_pitch_min_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_pitch_min > DEBOUNCE_DELAY)
    {
        last_trigger_time_pitch_min = now;
        bool is_low = digitalRead(LIMIT_WRIST_PITCH_MIN) == LOW;
        limit_wrist_pitch_min_activated = is_low;
        if (is_low)
        {
            limit_wrist_pitch_min_activated = 1;
            // Serial.println("Wrist pitch min limit reached");
            Wrist_Pitch.stop();
            // Wrist_Pitch.set_current_as_angle_sw(Wrist_Pitch.min_angle);
            Wrist_Pitch.new_setpoint(Wrist_Pitch.setpoint + 5.0f);
        }
        else
        {
            limit_wrist_pitch_min_activated = 0;
        }
    }
}
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
void limit_end_effector_max_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_max > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_max = now;
        if (digitalRead(LIMIT_END_EFFECTOR_MAX) == LOW)
        {
            limit_end_effector_max_activated = 1;
            // Serial.println("End effector max limit reached");
            End_Effector.stop();
        }
        else
        {
            limit_end_effector_max_activated = 0;
        }
    }
}

void limit_end_effector_min_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_min > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_min = now;
        if (digitalRead(LIMIT_END_EFFECTOR_MIN) == LOW)
        {
            limit_end_effector_min_activated = 1;
            // Serial.println("End effector min limit reached");
            End_Effector.stop();
        }
        else
        {
            limit_end_effector_min_activated = 0;
        }
    }
}
#endif

#if TEST_ELBOW_SERVO == 1
void limit_elbow_max_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_max > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_max = now;
        if (digitalRead(LIMIT_ELBOW_MAX) == LOW)
        {
            limit_end_effector_max_activated = 1;
            // Serial.println("Elbow max limit reached");
            Elbow.stop();
        }
        else
        {
            limit_end_effector_max_activated = 0;
        }
    }
}

void limit_elbow_min_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_min > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_min = now;
        if (digitalRead(LIMIT_ELBOW_MIN) == LOW)
        {
            limit_end_effector_min_activated = 1;
            // Serial.println("Elbow min limit reached");
            Elbow.stop();
        }
        else
        {
            limit_end_effector_min_activated = 0;
        }
    }
}
#endif

#if TEST_WAIST_SERVO == 1
void limit_waist_max_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_max > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_max = now;
        if (digitalRead(LIMIT_WAIST_MAX) == LOW)
        {
            limit_end_effector_max_activated = 1;
            // Serial.println("Waist max limit reached");
            Waist.stop();
        }
        else
        {
            limit_end_effector_max_activated = 0;
        }
    }
}

void limit_waist_min_int()
{
    unsigned long now = millis();
    if (now - last_trigger_time_end_effector_min > DEBOUNCE_DELAY)
    {
        last_trigger_time_end_effector_min = now;
        if (digitalRead(LIMIT_WAIST_MIN) == LOW)
        {
            limit_end_effector_min_activated = 1;
            // Serial.println("Waist min limit reached");
            Waist.stop();
        }
        else
        {
            limit_end_effector_min_activated = 0;
        }
    }
}
#endif

void attach_all_interrupts()
{
#if TEST_WRIST_PITCH_CYTRON == 1
    attachInterrupt(digitalPinToInterrupt(LIMIT_WRIST_PITCH_MAX), limit_wrist_pitch_max_int, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_WRIST_PITCH_MIN), limit_wrist_pitch_min_int, CHANGE);
#endif

#if TEST_END_EFFECTOR_CYTRON == 1
    attachInterrupt(digitalPinToInterrupt(LIMIT_END_EFFECTOR_MAX), limit_end_effector_max_int, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_END_EFFECTOR_MIN), limit_end_effector_min_int, CHANGE);
#endif

#if TEST_ELBOX_SERVO == 1
    attachInterrupt(digitalPinToInterrupt(LIMIT_ELBOW_MAX), limit_elbow_max_int, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_ELBOW_MIN), limit_elbow_min_int, CHANGE);
#endif

#if TEST_WAIST_SERVO == 1
    attachInterrupt(digitalPinToInterrupt(LIMIT_WAIST_MAX), limit_waist_max_int, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIMIT_WAIST_MIN), limit_waist_min_int, CHANGE);
#endif
}

// void serialEvent()
// {
//     while (Serial.available())
//     {
//         // Read the incoming string
//         String incomingString = Serial.readStringUntil('\n');

//         // Check if the incoming string starts with "setpoint"
//         // Initialize parameters
//         double param1, param2, param3;

//         // Use sscanf to extract the parameters
//         sscanf(incomingString.c_str(), "%lf %lf %lf", &param1, &param2, &param3);

//         // Now you can use param1, param2, param3
//         // Serial.printf("Received angles: %f, %f, %f\r\n", param1, param2, param3);

//         // EEPROM.
//         byte val = EEPROM.read(0);
//         // Serial.printf("Read %p from EEPROM\r\n", val);
//         EEPROM.write(0, param1);
//         // Serial.printf("Wrote %f to EEPROM\r\n", param1);

//         // Call new_setpoint() with the received angle
//         // bool result = Wrist_Roll.new_setpoint(angle);
// #if TEST_WRIST_ROLL_CYTRON == 1
//         bool result1 = Wrist_Roll.new_setpoint(param1);
// #endif
// #if TEST_WRIST_PITCH_CYTRON == 1
//         bool result2 = Wrist_Pitch.new_setpoint(param1);
// #endif
// #if TEST_END_EFFECTOR_CYTRON == 1
//         bool result3 = End_Effector.new_setpoint(param1);
// #endif
// #if TEST_ELBOW_SERVO == 1
//         bool result4 = Elbow.new_setpoint(param1);
// #endif
// #if TEST_SHOULDER_SERVO == 1
//         bool result5 = Shoulder.new_setpoint(param1);
// #endif

//         // Print status.
// #if TEST_WRIST_ROLL_CYTRON == 1
//         Serial.printf("Wrist_Roll new_setpoint at %lf result: %d\r\n", param1, result1);
// #endif
// #if TEST_WRIST_PITCH_CYTRON == 1
//         // Serial.printf("Wrist_Pitch new_setpoint at %lf result: %d\r\n", param2, result2);
// #endif
// #if TEST_END_EFFECTOR_CYTRON == 1
//         Serial.printf("End_Effector new_setpoint at %lf result: %d\r\n", param3, result3);
// #endif
// #if TEST_ELBOW_SERVO == 1
//         Serial.printf("Elbow new_setpoint at %lf result: %d\r\n", param1, result4);
// #endif
// #if TEST_SHOULDER_SERVO == 1
//         Serial.printf("Shoulder new_setpoint at %lf result: %d\r\n", param1, result5);
// #endif
//     }
// }
