#ifndef MICROSTEPPING_H_
#define MICROSTEPPING_H_
#include "Arduino.h"

enum {
    STEPS200 = 200,
    STEPS400 = 400,
    STEPS800 = 800,
    STEPS1600 = 1600,
    STEPS3200 = 3200,
    STEPS6400 = 6400,
    STEPPER_stepsPerRotation_M0 = 16,
    STEPPER_stepsPerRotation_M1 = 4,
    STEPPER_stepsPerRotation_M2 = 15
};

#endif /* MICROSTEPPING_H_ */
#pragma once
