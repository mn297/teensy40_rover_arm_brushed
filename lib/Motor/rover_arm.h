// META SETTINGS---------------------------------------------------------------
#define USE_TEENSY 1
#define IRQ_DEBUG_PRIORITY 10
#define BRUSHED_ARM 0
#define BRUSHLESS_ARM 1

// CONFIGURATION---------------------------------------------------------------
#define TEST_ENCODER 0
#define SKIP_MASTERING 0
#define MASTERING 0
#define MASTERING_TEST 0
#define TICK 1
#define TEST_LOOP 1
#define TEST_LIMIT_SWITCH 0

#define PID_PERIOD_US 10000
#define PID_DT (PID_PERIOD_US * 1e-6f)
#define ROVER_LOOP_PERIOD_MS 250

// TEENSY PINS------------------------------------------------------------------
#ifdef USE_TEENSY
// Teensy 4.0 compatibility.
#define SPI_HandleTypeDef void
#define GPIO_TypeDef void
#define GPIO_PinState int
#define TIM_HandleTypeDef void
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#endif

#if BRUSHED_ARM == 1
#define DELAY_FACTOR 1
#define CS1 2
#define CS2 3
#define CS3 4
#define DIR1 5
#define PWM1 6
#define DIR2 7
#define PWM2 8
#define DIR3 9
#define PWM3 10

#define NO1 22
#define NO2 21
#define NO3 20
#define NO4 19
#define NO5 18

#define END_EFFECTOR_LASER NO5

#define LIMIT_WRIST_PITCH_MAX NO1
#define LIMIT_WRIST_PITCH_MIN NO2
#define LIMIT_END_EFFECTOR_MAX NO3
#define LIMIT_END_EFFECTOR_MIN NO4
#endif

#if BRUSHLESS_ARM == 1
#define DELAY_FACTOR 2
#define CS1 10
#define CS2 9
#define CS3 8
#define PWM1 2
#define PWM2 3
#define PWM3 4

#define NO1 22
#define NO2 21
#define NO3 20
#define NO4 19
#define NO5 18
#define NO6 17

#define ELBOW_BRAKE 14
#define SHOULDER_BRAKE 15

#define LIMIT_ELBOW_MAX NO1
#define LIMIT_ELBOW_MIN NO2
#define LIMIT_SHOULDER_MAX NO3
#define LIMIT_SHOULDER_MIN NO4
#define LIMIT_WAIST_MAX NO5
#define LIMIT_WAIST_MIN NO6
#endif

// DEBUG SETTINGS---------------------------------------------------------------
#define DEBUG_GDB_STUB 0
#define DEBUG_PID 0
#define DEBUG_ROVER_ARM_MOTOR 0
#define DEBUG_PRINT_MOTOR 1
#define DEBUG_ROVER_ARM_MOTOR_TICK 0

// DEADBAND SETTINGS------------------------------------------------------------
#define DEADBAND_CYTRON 0.8f
#define DEADBAND_SERVO 18.0f
#define IRQ_DEBUG IRQ_SOFTWARE
#define USE_DMA 0

// MOTOR SETTINGS---------------------------------------------------------------
#if BRUSHED_ARM == 1
#define TEST_WRIST_ROLL_CYTRON 0
#define TEST_WRIST_PITCH_CYTRON 1
#define TEST_END_EFFECTOR_CYTRON 0
#endif

#if BRUSHLESS_ARM == 1
#define TEST_ELBOW_SERVO 1
#define TEST_SHOULDER_SERVO 1
#define TEST_WAIST_SERVO 0
#endif

#define ELBOW_GEAR_RATIO 1.0f

// WRIST_ROLL_CYTRON
#define REG_KP_WRIST_ROLL 0.5f
#define REG_KI_WRIST_ROLL 0.1f
#define REG_KD_WRIST_ROLL 0.02f
#define REG_KP_WRIST_ROLL_AGG 0.3f
#define REG_KI_WRIST_ROLL_AGG 0.1f
#define REG_KD_WRIST_ROLL_AGG 0.02f
#define WRIST_ROLL_MIN_ANGLE MIN_FLOAT
#define WRIST_ROLL_MAX_ANGLE MAX_FLOAT
#define WRIST_ROLL_GEAR_RATIO 2.672222f
#define WRIST_ROLL_ZERO_ANGLE 270.0f

// WRIST_PITCH_CYTRON
#define REG_KP_WRIST_PITCH 1.2f
#define REG_KI_WRIST_PITCH 1.8f
#define REG_KD_WRIST_PITCH 0
#define REG_KP_WRIST_PITCH_AGG 1.8f
#define REG_KI_WRIST_PITCH_AGG 2.3f
#define REG_KD_WRIST_PITCH_AGG 0
#define WRIST_PITCH_MIN_ANGLE -75.0f
// #define WRIST_PITCH_MAX_ANGLE 75.0f
#define WRIST_PITCH_MAX_ANGLE 38.0f
#define WRIST_PITCH_ZERO_ANGLE (326.0f + (-1.0f * 360.0f))
// #define WRIST_PITCH_ZERO_ANGLE 60.0f
#define WRIST_PITCH_GEAR_RATIO 1.0f

// END_EFFECTOR_CYTRON
#define REG_KP_END_EFFECTOR 0.4f
#define REG_KI_END_EFFECTOR 0.0
#define REG_KD_END_EFFECTOR 0.0
#define END_EFFECTOR_MIN_ANGLE MIN_FLOAT
#define END_EFFECTOR_MAX_ANGLE MAX_FLOAT
#define END_EFFECTOR_ZERO_ANGLE MAX_FLOAT
#define END_EFFECTOR_GEAR_RATIO 1.0f

// ELBOW_SERVO
#define REG_KP_ELBOW 0.6f
#define REG_KI_ELBOW 2.5f
#define REG_KD_ELBOW 0
#define REG_KP_ELBOW_AGG 0.8f
#define REG_KI_ELBOW_AGG 3.0f
#define REG_KD_ELBOW_AGG 0
#define ELBOW_MIN_ANGLE -70.0f
#define ELBOW_MAX_ANGLE 75.0f
#define ELBOW_ZERO_ANGLE (240.0f + (0 * 360.0f))

// SHOULDER_SERVO
#define REG_KP_SHOULDER 0.8f
#define REG_KI_SHOULDER 1.5f
#define REG_KD_SHOULDER 0
#define REG_KP_SHOULDER_AGG 1.0f
#define REG_KI_SHOULDER_AGG 1.8f
#define REG_KD_SHOULDER_AGG 0
#define SHOULDER_MIN_ANGLE -60.0f
#define SHOULDER_MAX_ANGLE 90.0f
#define SHOULDER_ZERO_ANGLE 86.0f + (-256.0f * 360.0f)

// WAIST_SERVO
#define REG_KP_WAIST 0.4f
#define REG_KI_WAIST 0.1f
#define REG_KD_WAIST 0.1f
#define REG_KP_WAIST_AGG 0.6f
#define REG_KI_WAIST_AGG 0.1f
#define REG_KD_WAIST_AGG 0.1f
#define WAIST_MIN_ANGLE -125.97f
#define WAIST_MAX_ANGLE 118.76f
#define WAIST_ZERO_ANGLE 0

// FUNCTION PROTOTYPES----------------------------------------------------------
void print_motor(char *msg, void *pMotor);
void rover_arm_setup(void);
void rover_arm_loop(void);
void test_limit_switches();
