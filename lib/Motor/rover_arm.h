#define USE_TEENSY 1

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

#endif


#define TEST_END_EFFECTOR_CYTRON 0
#define TEST_WRIST_ROLL_CYTRON 1
#define TEST_WRIST_PITCH_CYTRON 0

#define TEST_ELBOW_SERVO 0
#define TEST_SHOULDER_SERVO 0
#define TEST_WAIST_SERVO 0

#define TEST_ENCODER 0
#define SIMULATE_LIMIT_SWITCH 1

#define AMT22_TIM htim15

// WRIST_ROLL_TESTBENCH (DC)
#define REG_KP_WRIST_ROLL 0.8
#define REG_KI_WRIST_ROLL 0.1
#define REG_KD_WRIST_ROLL 0.1

// WRIST_PITCH_CYTRON
#define REG_KP_WRIST_PITCH 0.5
#define REG_KI_WRIST_PITCH 0.2
#define REG_KD_WRIST_PITCH 0.2

// WAIST_SERVO
// Vincent's configuration
#define REG_KP_WAIST_VINCENT 1.8
#define REG_KI_WAIST_VINCENT 1.0
#define REG_KD_WAIST_VINCENT 0.5

// Sam's configuration
// #define REG_KP_WAIST_SAM 1.2
// #define REG_KI_WAIST_SAM 0.4
// #define REG_KD_WAIST_SAM 0.4

// Speed configuration
// #define REG_KP_WAIST_SPEED 2
// #define REG_KI_WAIST_SPEED 0.4
// #define REG_KD_WAIST_SPEED 0.4

// ELBOW_SERVO
// Vincent's configuration
#define REG_KP_ELBOW_VINCENT 0.6
#define REG_KI_ELBOW_VINCENT 0.2
#define REG_KD_ELBOW_VINCENT 0.1

// END_EFFECTOR_CYTRON
#define REG_KP_END_EFFECTOR 0.4
#define REG_KI_END_EFFECTOR 0.0
#define REG_KD_END_EFFECTOR 0.0

void print_motor(char *msg, void *pMotor);
void rover_arm_setup(void);
void rover_arm_loop(void);

