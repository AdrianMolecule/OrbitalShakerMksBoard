#ifndef SERIALCOMMUNICATION_H_
#define SERIALCOMMUNICATION_H_

#include <Arduino.h>
#include "Splitter.h"
//


class SerialCommunication final {
public:
	SerialCommunication();
	virtual ~SerialCommunication();
	String checkForCommand();
	void executeCommand(char *token, String extra);
};

#endif /* SERIALCOMMUNICATION_H_ */
