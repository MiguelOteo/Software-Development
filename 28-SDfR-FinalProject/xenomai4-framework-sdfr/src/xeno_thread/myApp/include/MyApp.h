#pragma once

#include "Native20Sim.h"

class MyApp : public Native20Sim
{
public:
    MyApp();
    ~MyApp();
    
private:
    // Variables to keep track of encoder rollover
    uint leftPrevEncoderCount_ = 0;
    uint rightPrevEncoderCount_ = 0;

    int leftEncoderRollovers_ = 0;
    int rightEncoderRollovers_ = 0;

    // Constants
    const int RIGHT_MOTOR_DIRECTION = -1;
    const int LEFT_MOTOR_DIRECTION = 1;
    const double CORRECTION_MULTIPLIER = 4; // Correction multiplier since the encoders seem to have a 4x offset (probably due to X4 encoding)
    const double ENCODER_COUNT_PER_REVOLUTION = 1024 * CORRECTION_MULTIPLIER;
    const double GEAR_RATIO = 15.58;
    const int ENCODER_RANGE = 16383;


protected:
    int preProc() override;
    int postProc() override;
};
