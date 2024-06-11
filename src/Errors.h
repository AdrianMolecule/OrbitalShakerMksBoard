#ifndef ERRORS_H_
#define ERRORS_H_
#include "Arduino.h"

class Errors {
public:
	static void raiseUserInputAlarm(String);
	static void raiseDeveloperAlarm(String);
	static void raiseSensorReadValueAlarm(String);
};

#endif /* ERRORS_H_ */
#pragma once
