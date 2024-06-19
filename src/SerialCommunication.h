#ifndef SERIALCOMMUNICATION_H_
#define SERIALCOMMUNICATION_H_

#include <Arduino.h>
#include "Splitter.h"
//


class SerialCommunication final {
public:
	SerialCommunication();
	virtual ~SerialCommunication();
	void checkForCommand(String& command, String& commandArguments);
	void executeCommand(char *token, String extra);
};

#endif /* SERIALCOMMUNICATION_H_ */
