#include "AdrianOsPcbPins.h"
#include "Arduino.h"
#include "Pins.h"

AdrianOsPcbPins::AdrianOsPcbPins() {
    Serial.print("TADAA: AdrianOsPcbPins Constructor called");
    HeaterPin = NOT_PRESENT;
    HeaterPwmChannel = NOT_PRESENT;
    // DIR_PIN = 32,// not used using the standard direction
    StepperPwmStepPin = 25;  // it's called STEP_PIN in the minimal pcb
    StepperPwmChannel = 2; 
    StepperEnablePin = 26;
    I2SoClockPin = NOT_PRESENT;
    I2SoLatchPin = NOT_PRESENT;
    LedPin = 2;
    PotentiometerPin = 127;
    TempSensorPin = NOT_PRESENT;
    SpeakerPin = 12;    // see the my board esp minipins.png in the Eagle directory
    SpeakerChannel = 4;
    FanPin = NOT_PRESENT;
    FanPwmChannel = NOT_PRESENT;
    MemoryCsPin = NOT_PRESENT;
    SpindleEnablePin = NOT_PRESENT;  // do we need this?
    UseOneWireForTemperature = false;
    MKSBoard = false;
}

// const int LED_PIN_PROCESSOR = 2;
// // we have 2 io available and we use the one labeled 35
// // as in IO35 which is connected to the connector IO pin 6
// //(the last pin is 3.3 v and it's in the right top corner when controller key is on the left and board is horizontal - top view
// // that is pulled to ground by R29 and serialized by current limiting R28
// const int POTENTIOMETER_PIN = 35;
// const int BUZZ_CHANNEL = 5;  // for the Buzzer
// const int MOTOR_PWMChannel = 0;
