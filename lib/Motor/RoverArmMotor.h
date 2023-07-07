// #include <PID_v1.h>
#include "pid.h" // This is the file we're working on
#include "Teensy_PWM.h"
#include <movingAvg.h>
#include <cstdint>
#define SPI_HandleTypeDef void  
#define GPIO_TypeDef void
#define GPIO_PinState int
#define TIM_HandleTypeDef void
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

class RoverArmMotor
{

public:
// Our motors use two different ESC's with different control schemes
#define CYTRON 0
#define BLUE_ROBOTICS 1

// ADC values representing 359 and 0 degrees respectively
#define MAX_ADC_VALUE 4095 // 3850
#define MIN_ADC_VALUE 0    // 200

    RoverArmMotor(int pwm_pin, int dir_pin, int encoder_pin, int esc_type, 
                double minimum_angle, double maximum_angle, int limit_switch_pin = -1);

    // Setters for various tunable parameters of our motors
    void set_PID_params(double regP, double regI, double regD); // mn297

    void setAggressiveCoefficients(double P, double I, double D);
    void setRegularCoefficients(double P, double I, double D);
    void setRetuningGapLimit(int gap);
    void setAngleLimits(double lowest_angle, double highest_angle);
    void set_zero_angle();    // unused
    void set_zero_angle_sw(); // mn297 software zero angle
    void set_max_angle_sw();

    uint32_t get_turns_encoder(); // mn297
    void reset_encoder();         // mn297
    bool setMultiplierBool(bool mult, double ratio);
    bool newSetpoint(double angle);

    void setPIDOutputLimits(double lower_end, double upper_end);
    void setMovingAverageWindowSize(int size);
    void disengageBrake();
    void engageBrake();

    double get_current_angle_avg(); // mn297
    double get_current_angle();     // mn297
    int get_current_angle_multi(double *angle);
    int get_current_angle_sw(double *angle);

    double getSetpoint();
    double getCurrentOutput();
    int getDirection();
    double getRatio();
    // void setGearRatio(double ratio);

    void begin(double regP, double regI, double regD);
    void tick();
    int forward(int percentage_speed = 25);
    int reverse(int percentage_speed = 25);
    void stop();
    void setGearRatio(double ratio);
    int get_turn_count(); // mn297

private:
public: // TESTING only
    // Default to open loop, will need to enter the coefficients to begin
    PID *internalPIDInstance;
    Teensy_PWM *pwmInstance;

    double regularKp, regularKi, regularKd;
    // PINS
    int _pwm, _dir, _encoder, _limit_switch;

    int _pwm_freq;

    int movingAverageWindowSize;
    double lowestAngle, highestAngle;
    int escType;
    int adcResult;
    double currentAngle, lastAngle;
    bool wrist_waist;
    // int multiplier;
    volatile double input;
    volatile double output;
    volatile double setpoint;
    int actuationState;
    double gearRatio;

    int useSwAngle;       // mn297
    double zero_angle_sw; // mn297
    int turn_count;       // mn297
    int servo_dir;        // mn297
    double forwardDistance;
    double backwardDistance;

    enum ActuationStates
    {
        FIRST_ROTATION_REGION,
        SECOND_ROTATION_REGION,
        RATIO_IS_ONE
    };

    double mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

    void WatchdogISR();
};
