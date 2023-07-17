// META SETTINGS---------------------------------------------------------------
#define USE_TEENSY 1
#define IRQ_DEBUG_PRIORITY 10

// TEENSY PINS------------------------------------------------------------------
#ifdef USE_TEENSY
// Teensy 4.0 compatibility.
#define SPI_HandleTypeDef void
#define GPIO_TypeDef void
#define GPIO_PinState int
#define TIM_HandleTypeDef void
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

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

#define LIMIT_WRIST_PITCH_MAX NO1
#define LIMIT_WRIST_PITCH_MIN NO2
#define LIMIT_END_EFFECTOR_MAX NO3
#define LIMIT_END_EFFECTOR_MIN NO4

#else
#define AMT22_TIM htim15
#endif

// DEBUG SETTINGS---------------------------------------------------------------
#define DEBUG_GDB_STUB 0
#define DEBUG_PID 0
#define DEBUG_ROVER_ARM_MOTOR 0
#define DEBUG_PRINT_MOTOR 1

// DEADBAND SETTINGS------------------------------------------------------------
#define DEADBAND_CYTRON 0.8f
#define DEADBAND_SERVO 10.0f

// MOTOR SETTINGS---------------------------------------------------------------
#define TEST_WRIST_ROLL_CYTRON 0
#define TEST_WRIST_PITCH_CYTRON 1
#define TEST_END_EFFECTOR_CYTRON 0

#define TEST_ELBOW_SERVO 0
#define TEST_SHOULDER_SERVO 0
#define TEST_WAIST_SERVO 0

#define TEST_ENCODER 0
#define SIMULATE_LIMIT_SWITCH 1

#define ELBOW_MIN_ANGLE MIN_FLOAT
#define ELBOW_MAX_ANGLE MAX_FLOAT
#define ELBOW_GEAR_RATIO 1.0f

// WRIST_ROLL_TESTBENCH (DC)
#define REG_KP_WRIST_ROLL 0.8
#define REG_KI_WRIST_ROLL 0.1
#define REG_KD_WRIST_ROLL 0.1
#define WRIST_ROLL_MIN_ANGLE MIN_FLOAT
#define WRIST_ROLL_MAX_ANGLE MAX_FLOAT
#define WRIST_ROLL_GEAR_RATIO 2.672222f

// WRIST_PITCH_CYTRON
#define REG_KP_WRIST_PITCH 0.6
#define REG_KI_WRIST_PITCH 0
#define REG_KD_WRIST_PITCH 0
#define WRIST_PITCH_MIN_ANGLE 0.0f
#define WRIST_PITCH_MAX_ANGLE 120.0f
#define WRIST_PITCH_GEAR_RATIO 1.0f

// END_EFFECTOR_CYTRON
#define REG_KP_END_EFFECTOR 0.4
#define REG_KI_END_EFFECTOR 0.0
#define REG_KD_END_EFFECTOR 0.0
#define END_EFFECTOR_MIN_ANGLE MIN_FLOAT
#define END_EFFECTOR_MAX_ANGLE MAX_FLOAT
#define END_EFFECTOR_GEAR_RATIO 1.0f

// ELBOW_SERVO
#define REG_KP_ELBOW 0.6
#define REG_KI_ELBOW 0.2
#define REG_KD_ELBOW 0.1

// WAIST_SERVO
#define REG_KP_WAIST 1.8
#define REG_KI_WAIST 1.0
#define REG_KD_WAIST 0.5

// SHOULDER_SERVO
#define REG_KP_SHOULDER 1.0
#define REG_KI_SHOULDER 0.5
#define REG_KD_SHOULDER 0.2

// FUNCTION PROTOTYPES----------------------------------------------------------
void print_motor(char *msg, void *pMotor);
void rover_arm_setup(void);
void rover_arm_loop(void);
