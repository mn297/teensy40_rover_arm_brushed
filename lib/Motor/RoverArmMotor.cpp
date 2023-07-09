// #include <Arduino.h>
#include "RoverArmMotor.h"
#include "Teensy_PWM.h"
#include <Arduino.h>
#include "AMT22.h"
#include "rover_arm.h"
#include <cstdlib>
#include <stdio.h>
#include <inttypes.h>
#include <cmath>

#define BLUE_ROBOTICS_STOP_DUTY_CYCLE 0.6f

// The motor will not move until begin() is called!
/**
 * @brief  Constructor for RoverArmMotor class
 * @param  spi_handle: encoder SPI handle
 * @param  pwm_pin: pin for the PWM
 * @param  dir_pin: only used for Cytron ESC
 * @param  encoder_pin: pin for the encoder
 * @param  esc_type: CYTRON or BLUE_ROBOTICS
 * @param  minimum_angle: minimum angle of the motor
 * @param  maximum_angle: maximum angle of the motor
 * @param  limit_switch_pin: pin for the brake or limit switch
 * @retval None
 */
RoverArmMotor::RoverArmMotor(int pwm_pin, int dir_pin, int encoder_pin, int esc_type,
                             double minimum_angle, double maximum_angle, int limit_switch_pin = -1)
{

    // constructor
    _pwm = pwm_pin;
    _dir = dir_pin;
    _encoder = encoder_pin;
    _limit_switch = limit_switch_pin;
    escType = esc_type;
    lowestAngle = minimum_angle;
    highestAngle = maximum_angle;

    _pwm_freq = 400;

    // clean up variables
    input = 0;
    output = 0;
    lastAngle = 0;
    useSwAngle = 1;    // default use software angle
    zero_angle_sw = 0; // default no offset
    gearRatio = 1.0;   // default no multiplier
    wrist_waist = false;
}

void RoverArmMotor::begin(double regP, double regI, double regD)
{
    Serial.println("RoverArmMotor::begin()");
    /*------------------Set pin modes------------------*/
    pinMode(_pwm, OUTPUT);
    pinMode(_dir, OUTPUT);
    pinMode(_encoder, OUTPUT);
    if (_limit_switch != -1)
    {
        pinMode(_limit_switch, INPUT_PULLUP);
    }
    pwmInstance = new Teensy_PWM(_pwm, _pwm_freq, 0.0f); // 400Hz equals to 2500us period
    Serial.println("RoverArmMotor::begin() 2");

    /*------------------Initialize timers------------------*/
    // HAL_TIM_PWM_Start(pwm.p_tim, pwm.tim_channel);
    delay(1000);                              // wait for the motor to start up
    this->stop();                             // stop the motor
    delay(100);                               // wait for the motor to start up
    this->stop();                             // stop the motor
    Serial.println("_pwm = " + String(_pwm)); // mn297
    Serial.println("RoverArmMotor::begin() 3");

    /*------------------Initialize PID------------------*/
    if (escType == CYTRON)
    {
        internalPIDInstance = new PID(0.005, 99.0, -99.0, regP, regD, regI);
    }
    else if (escType == BLUE_ROBOTICS)
    {
        internalPIDInstance = new PID(0.005, 350.0, -350.0, regP, regD, regI);
    }
    Serial.println("RoverArmMotor::begin() 4");

    /*------------------Get setpoint------------------*/
    // Get current location and set it as setpoint. Essential to prevent jerkiness
    // as the microcontroller initializes.
    // adcResult = internalAveragerInstance.reading(analogRead(encoder));
    // after setup, currentAngle is same as setpoint
    delay(100);                                      // safety
    int error = get_current_angle_sw(&currentAngle); // fix setpoint not equal to current angle
    if (error == -1)
    {
        printf("ERROR: get_current_angle_sw() returned -1 from begin()\r\n");
        return;
    }
    Serial.println("Motor current angle: " + String(currentAngle));
    setpoint = currentAngle;
    lastAngle = currentAngle;
    Serial.println("RoverArmMotor::begin() 5");

    /*------------------Set PID parameters------------------*/
    regularKp = regP;
    regularKi = regI;
    regularKd = regD;

    // if(brake)  engageBrake(); //use brake if there is one
    if (_limit_switch != -1)
        engageBrake(); // use brake if there is one
    Serial.println("RoverArmMotor::begin() 6");

    /*------------------Reverse to hit zero angle------------------*/
    delay(1000); // wait for the motor to start up
    this->reverse();
    Serial.println("RoverArmMotor::begin() 7");
    delay(500); // wait for the motor to start up
    this->reverse();
    Serial.println("RoverArmMotor::begin() 8");
    return;
    Serial.println("RoverArmMotor::begin() 8");
}

int positive_rezeros = 0;
double real_angle = 0;

// Needs to be called in each loop
void RoverArmMotor::tick()
{
    // this->stop(); // stop the motor

    /*------------------Get current angle------------------*/
    int error = get_current_angle_sw(&currentAngle);
    if (error == -1)
    {
        output = 0;
        this->stop(); // stop the motor
        printf("ERROR: get_current_angle_sw() returned -1 from tick()\r\n");
        return;
    }

    //------------------remove jitter------------------//
    // If the change in angle is less than the threshold, return early
    double diff = min(abs(currentAngle - setpoint), 360.0 - abs(currentAngle - setpoint));
    if (diff < 0.5)
    {
        output = 0;
        this->stop(); // stop the motor
        return;
    }

    input = currentAngle; // range is R line
    lastAngle = currentAngle;

    //------------------Compute PID------------------//
    // Compute distance, retune PID if necessary. Less aggressive tuning params for small errors
    // Find the shortest from the current position to the set point

    if (wrist_waist)
    {
        forwardDistance = (setpoint > input) ? setpoint - input : (360 - input) + setpoint;
        backwardDistance = (setpoint > input) ? (360 - setpoint) + input : input - setpoint;
        // GO BACKWARDS CW
        if (backwardDistance < forwardDistance - 1.0) // handle hysterisis
        {
            if (setpoint > input)
            {
                output = internalPIDInstance->calculate(setpoint, input + 360); // buff it 360 to go backwards
                Serial.printf("Case 1, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
            }
            else
            {
                output = internalPIDInstance->calculate(setpoint, input); // wrapped around so bigger so no need buff 360
                Serial.printf("Case 2, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
            }
        }
        // GO FORWARDS CCW
        else
        {
            if (setpoint > input)
            {
                output = internalPIDInstance->calculate(setpoint, input); //  wrapped around so bigger so no need nerf 360
                Serial.printf("Case 3, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
            }
            else
            {
                output = internalPIDInstance->calculate(setpoint, input - 360); // nerf it 360 to go forwards
                Serial.printf("Case 4, setpoint = %f, input = %f, output = %f\r\n", setpoint, input, output);
            }
        }
    }
    else
    {
        output = internalPIDInstance->calculate(setpoint, input); // return value stored in output
    }

    //------------------SAFETY------------------//
    //    if (currentAngle >= (highestAngle - 2) || currentAngle <= (lowestAngle + 2))
    //        output = 0.0;

    //------------------Write to motor------------------//
    if (escType == CYTRON)
    {
        // Interpret sign of the error signal as the direction pin value
        if (output > 0)
        {
            // HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_SET); // B high
            digitalWrite(_dir, HIGH);
        }
        else
        {
            // HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_RESET); // A high
            digitalWrite(_dir, LOW);
        }
        // Write to PWM pin
        double test_output = abs(output); // smoothing
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, (int)test_output);
        pwmInstance->setPWM(_pwm, _pwm_freq, test_output);
        return;
    }

    // This one is more straightforward since we already defined the output range
    // from 1100us to 1900us
    else if (escType == BLUE_ROBOTICS)
    {

        //------------------DEADBAND------------------//
        volatile double temp_output = output;
        int deadband = 10;
        if (abs(output) <= deadband)
        {
            temp_output = 0;
        }
        else
        {
            // if (output > 0)
            // {
            //     temp_output = (output + deadband / 2);
            // }
            // else
            // {
            //     temp_output = (output - deadband / 2);
            // }
        }
        volatile double output_actual = (1500 - 1 + temp_output) / 2500;
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, (int)output_actual);
        // uint32_t compare_actual = __// HAL_TIM_GET_COMPARE(pwm.p_tim, pwm.tim_channel);
        // printf("setpoint: %f, currentAngle: %f, lastAngle: %f ", setpoint, currentAngle, lastAngle);
        // printf("output_actual: %f, compare: ", output_actual);
        // printf("%" PRIu32 "\r\n", compare_actual);
        return;
    }
    return;
}

int RoverArmMotor::forward(int percentage_speed)
{
    if (percentage_speed < 0 || percentage_speed > 100)
    {
        return -1;
    }
    if (escType == CYTRON)
    {
        // HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_SET); // B high
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, percentage_speed);
        digitalWrite(_dir, HIGH);
        pwmInstance->setPWM(_pwm, _pwm_freq, percentage_speed);
        return 0;
    }
    else if (escType == BLUE_ROBOTICS)
    {
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 1499 + 350 * percentage_speed / 100);
        double duty_cycle = BLUE_ROBOTICS_STOP_DUTY_CYCLE + (BLUE_ROBOTICS_STOP_DUTY_CYCLE * percentage_speed / 100); // TODEBUG
        pwmInstance->setPWM(_pwm, _pwm_freq, duty_cycle);
        return 0;
    }
    return -1;
}

int RoverArmMotor::reverse(int percentage_speed)
{
    if (percentage_speed < 0 || percentage_speed > 100)
    {
        return -1;
    }
    if (escType == CYTRON)
    {
        // HAL_GPIO_WritePin(dir.port, dir.pin, GPIO_PIN_RESET); // A high
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, percentage_speed);
        digitalWrite(_dir, LOW);
        pwmInstance->setPWM(_pwm, _pwm_freq, percentage_speed);
        return 0;
    }
    else if (escType == BLUE_ROBOTICS)
    {
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 1499 - 350 * percentage_speed / 100);
        double duty_cycle = BLUE_ROBOTICS_STOP_DUTY_CYCLE - (BLUE_ROBOTICS_STOP_DUTY_CYCLE * percentage_speed / 100);
        pwmInstance->setPWM(_pwm, _pwm_freq, duty_cycle);
        return 0;
    }   
    return -1;
}

void RoverArmMotor::stop()
{
    if (escType == CYTRON)
    {
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 0);
        pwmInstance->setPWM(_pwm, _pwm_freq, 0);
        return;
    }
    else if (escType == BLUE_ROBOTICS)
    {
        // __HAL_TIM_SET_COMPARE(pwm.p_tim, pwm.tim_channel, 1499);
        pwmInstance->setPWM(_pwm, _pwm_freq, BLUE_ROBOTICS_STOP_DUTY_CYCLE);
        return;
    }
    return;
}

// Useless at the moment.
void RoverArmMotor::set_PID_params(double regP, double regI, double regD)
{
    regularKp = regP;
    regularKi = regI;
    regularKd = regD;
    return;
}

bool RoverArmMotor::setMultiplierBool(bool mult, double ratio)
{
    wrist_waist = mult;
    gearRatio = ratio;
    // a bit redundant but just a sanity check of a second getter method
    if (getRatio() == ratio)
        return true;
    return false;
}

// For display purposes
double RoverArmMotor::getSetpoint()
{
    return setpoint / gearRatio;
}

bool RoverArmMotor::newSetpoint(double angle)
{
    double setpoint_test = angle * gearRatio;
    if (wrist_waist)
    {
        setpoint_test = std::fmod(setpoint_test, 360 * gearRatio);
        if (setpoint_test < 0)
        {
            setpoint_test += (360 * gearRatio);
        }
    }
    if (setpoint_test >= lowestAngle && setpoint_test <= highestAngle)
    {
        setpoint = setpoint_test;
        return true;
    }
    else
    {
        return false;
    }
}

int RoverArmMotor::getDirection()
{
    // return (digitalRead(dir) == HIGH) ? FWD : REV;
    // return (// HAL_GPIO_ReadPin(dir.port, dir.pin) == GPIO_PIN_SET) ? FWD : REV; // mn297, TODO check if this is correct
}

void RoverArmMotor::setGearRatio(double ratio)
{
    gearRatio = ratio;
    return;
}

void RoverArmMotor::setAngleLimits(double lowest, double highest)
{
    lowestAngle = lowest * gearRatio;
    highestAngle = highest * gearRatio;
    return;
}

void RoverArmMotor::set_zero_angle()
{
    setZeroSPI(_encoder); // timer not used, so nullptr
}

void RoverArmMotor::reset_encoder()
{
    resetAMT22(_encoder); // timer not used, so nullptr
    return;
}

void RoverArmMotor::set_zero_angle_sw()
{
    this->get_current_angle_multi(&zero_angle_sw);
    return;
}

void RoverArmMotor::set_max_angle_sw()
{
    double temp = 0;
    this->get_current_angle_multi(&temp);
    zero_angle_sw = temp - highestAngle;
}

uint32_t RoverArmMotor::get_turns_encoder()
{
    uint32_t turns = get_turns_AMT22(_encoder, 12);
    return turns;
}

void RoverArmMotor::disengageBrake() // TODO
{
    if (_limit_switch != -1)
    {
        // HAL_GPIO_WritePin(limit_switch.port, limit_switch.pin, GPIO_PIN_RESET); // mn297
    }
}

void RoverArmMotor::engageBrake() // TODO
{
    if (_limit_switch != -1)
    {
        // HAL_GPIO_WritePin(limit_switch.port, limit_switch.pin, GPIO_PIN_SET); // mn297
    }
}

// double RoverArmMotor::get_current_angle_avg()
//{ // UNSUPPORTED
//     // return currentAngle / gearRatio;
//     uint16_t encoderData = getPositionSPI(spi, encoder.port, encoder.pin, 12, nullptr);  // timer not used, so nullptr
//     adcResult = internalAveragerInstance.reading(encoderData);                           // implicit cast to int
//     currentAngle = mapFloat((float)adcResult, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); // mn297 potentiometer encoder
//     return currentAngle / gearRatio;
// }

double RoverArmMotor::get_current_angle()
{                                                                                          // mn297
    uint16_t encoderData = getPositionSPI(_encoder, 12);                                   // timer not used, so nullptr
    currentAngle = mapFloat((float)encoderData, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); // mn297 potentiometer encoder
    return currentAngle;
}

int RoverArmMotor::get_current_angle_multi(double *angle)
{
    int16_t result_arr[2];
    int error = getTurnCounterSPI(result_arr, _encoder, 12); // timer not used, so nullptr
    if (error == -1)
    {
        printf("ERROR: getTurnCounterSPI() returned -1 from get_current_angle_multi()\r\n");
        return -1;
    }

    double angle_raw = mapFloat((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f); // mn297 potentiometer encoder
    int turns = result_arr[1];
    *angle = angle_raw + 360 * turns;

    return 0;
}

int RoverArmMotor::get_current_angle_sw(double *angle)
{
    double current_angle_multi;
    int error = get_current_angle_multi(&current_angle_multi);
    if (error == -1)
    {
        printf("ERROR: get_current_angle_multi() returned -1 from get_current_angle_sw()\r\n");
        return -1;
    }

    double diff = current_angle_multi - zero_angle_sw;
    if (wrist_waist) // TODO optimize
    {
        double temp;
        temp = std::fmod(diff, (360 * gearRatio));
        if (temp < 0)
        {
            temp += (360 * gearRatio);
        }
        *angle = temp;
        // printf("diff: %f, angle: %f\r\n", diff, *angle);
    }
    else
    {
        *angle = diff;
    }
    return 0; // return 0 on success
}

double RoverArmMotor::getCurrentOutput()
{
    return output;
}

double RoverArmMotor::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    double result = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

    return result;
}

double RoverArmMotor::getRatio()
{
    return gearRatio;
}

int RoverArmMotor::get_turn_count()
{
    int16_t result_arr[2];
    getTurnCounterSPI(result_arr, _encoder, 12); // timer not used, so nullptr
    return result_arr[1];
}

void RoverArmMotor::WatchdogISR()
{
    // Get current angle

    // Set setpoint to that angle
}
