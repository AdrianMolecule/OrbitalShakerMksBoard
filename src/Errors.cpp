#include "Errors.h"


//
void Errors::raiseUserInputAlarm(String message) {
    Serial.print("!!!!!!!!!!!! USER ERROR !!!!!!!!!!!!   ");
    Serial.println(message);
    //userInputAlarmBuzz();
}

 void Errors::raiseDeveloperAlarm(String message) {
	Serial.print("!!!!!!!!!!!! DEVELOPER ERROR !!!!!!!!!!!!!!!    ");
	Serial.println(message);
	//developerAlarmBuzz();
}

void Errors::raiseSensorReadValueAlarm(String message) {
    Serial.print("!!!!!!!!!!!! SENSOR READ VALUE ERROR !!!!!!!!!!!!    ");
    Serial.println(message);
    //sensorReadValueAlarmBuzz();
}
