#ifndef Pins_H_
#define Pins_H_
#include "Arduino.h"

class Pins {
    public:
    Pins(); // Constructor declaration  
      static const uint8_t NOT_PRESENT = 255;
      uint8_t HeaterPin;
      uint8_t HeaterPwmChannel;
      uint8_t StepperPwmStepPin;
      uint8_t StepperPwmChannel;
      uint8_t StepperEnablePin ;
      uint8_t I2SoDataPin;
      uint8_t I2SoClockPin ;
      uint8_t I2SoLatchPin;
      uint8_t LedPin;
      uint8_t PotentiometerPin ;
      uint8_t TempSensorPin ;
      uint8_t SpeakerPin ;
      uint8_t SpeakerChannel;
      uint8_t FanPin ;
      uint8_t MemoryCsPin ;
      uint8_t FanPwmChannel;
      uint8_t SpindleEnablePin ;
      bool UseOneWireForTemperature = false;
      bool MKSBoard = true;
 
};

#endif /* Pins_H_ */
#pragma once
