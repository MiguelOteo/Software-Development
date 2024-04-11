#include "MyApp.h"

#include <algorithm>
#include <math.h>

MyApp::MyApp()
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
}

MyApp::~MyApp()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

int MyApp::preProc()
{
    // Left motor encoder wraparound detection
    const int leftMotorEncoderRaw = FpgaInput.channel2;
    if (leftMotorEncoderRaw < 300 && leftPrevEncoderCount_ > 16000)
    {
        leftEncoderRollovers_++;
    }
    else if (leftMotorEncoderRaw > 16000 && leftPrevEncoderCount_ < 300)
    {
        leftEncoderRollovers_--;
    }
    long int leftAbsEncoderCount = (long int)ENCODER_RANGE * (long int)leftEncoderRollovers_ + (long int)leftMotorEncoderRaw;

    // Right motor encoder wraparound detection
    const int rightMotorEncoderRaw = FpgaInput.channel1;
    if (rightMotorEncoderRaw < 300 && rightPrevEncoderCount_ > 16000)
    {
        rightEncoderRollovers_++;
    }
    else if (rightMotorEncoderRaw > 16000 && rightPrevEncoderCount_ < 300)
    {
        rightEncoderRollovers_--;
    }
    long int rightAbsEncoderCount = (long int)ENCODER_RANGE * (long int)rightEncoderRollovers_ + (long int)rightMotorEncoderRaw;

    // Store encoder count for next iteration
    leftPrevEncoderCount_ = leftMotorEncoderRaw;
    rightPrevEncoderCount_ = rightMotorEncoderRaw;

    // Calculate the actual motor angles (of the output shaft)
    const double leftMotorAngle = (double)leftAbsEncoderCount * 2 * M_PI / (ENCODER_COUNT_PER_REVOLUTION * GEAR_RATIO);
    const double rightMotorAngle = (double)rightAbsEncoderCount * 2 * M_PI / (ENCODER_COUNT_PER_REVOLUTION * GEAR_RATIO);

    // Debug print statements
    // evl_printf("left motor encoder value : %d, angle %f, setpoint vel : %f\n", leftAbsEncoderCount, leftMotorAngle, RosData.setpoint_vel_right);
    // evl_printf("right motor encoder value : %d, angle %f, setpoint vel : %f\n", rightAbsEncoderCount, rightMotorAngle, RosData.setpoint_vel_left);

    // Set 20sim model inputs
    u[0] = leftMotorAngle * LEFT_MOTOR_DIRECTION;   // PosLeft (left encoder value on channel 2, in radians)
    u[1] = rightMotorAngle * RIGHT_MOTOR_DIRECTION; // PosRight (right encoder value on channel 1, in radians)
    u[2] = RosData.setpoint_vel_left;               // SetVelLeft (left motor setpoint velocity, in rad/s)
    u[3] = RosData.setpoint_vel_right;              // SetVelRight (right motor setpoint velocity, in rad/s)

    return 0;
}

int MyApp::postProc()
{
    // Get the outputs from the 20sim controller
    const double leftMotorPwmPercentage = y[0];  // SteerLeft output of 20sim model (in percentage of maximum PWM)
    const double rightMotorPwmPercentage = y[1]; // SteerRight output of 20sim model (in percentage of maximum PWM)

    // Set the PWM value for the motors to send to the FPGA, but clamp the fraction between -1 and +1
    FpgaOutput.pwm1 = MAX_PWM_VALUE * std::clamp(rightMotorPwmPercentage / 100, -1.0, 1.0) * RIGHT_MOTOR_DIRECTION;
    FpgaOutput.pwm2 = MAX_PWM_VALUE * std::clamp(leftMotorPwmPercentage / 100, -1.0, 1.0) * LEFT_MOTOR_DIRECTION;

    // Debug print
    // evl_printf("left motor pwm percentage: %f, output PWM %d\n", leftMotorPwmPercentage, FpgaOutput.pwm2);
    // evl_printf("right motor pwm percentage: %f, output PWM %d\n", rightMotorPwmPercentage, FpgaOutput.pwm1);

    // We do not need to send information back to ROS 2 for this implementation

    return 0;
}