#include "Pins.h"
#include "Arduino.h"

Pins::Pins() {
    //255 means unused
    HeaterPin =255;
    HeaterPwmChannel =255;
    StepperPwmStepPin =255;
    StepperPwmChannel =255;
    StepperEnablePin =255;
    I2SoClockPin =255;
    I2SoLatchPin =255;
    LedPin =255;
    PotentiometerPin =255;
    TempSensorPin =255;
    SpeakerPin =255;
    SpeakerChannel =255;
    FanPin =255;
    FanPwmChannel =255;
    MemoryCsPin =255;
    SpindleEnablePin =255;  // do we need this? Yes, it would be stepperOnOffEnablePin
    UseOneWireForTemperature = false;
    MKSBoard = true;
}


