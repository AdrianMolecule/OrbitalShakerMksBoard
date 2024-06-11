#include "SerialCommunication.h"

#include "Arduino.h"
#include "Errors.h"
#include <SPI.h>
#include <SD.h>
#include <FS.h>

SerialCommunication::SerialCommunication() {

}

SerialCommunication::~SerialCommunication() {
    // TODO Auto-generated destructor stub
}
//
void SerialCommunication::executeCommand(char *token, String extra) {
//    Splitter splitter = Splitter(extra);
//    if (*token == '?') {
//        Serial.println(" All commands");
//
//        }
//        Serial.println("? list all registered commands");
//        Serial.println("version: Basic Bioreactor VSC 1");
//        return;
//    }
//    boolean found = false;
//    for (CommandFunctionStruct *commandToupleP = commands; commandToupleP->commandName != ""; commandToupleP++) {
//        if (0 == strcasecmp(commandToupleP->commandName, token)) {
//            (*commandToupleP->func)(&splitter);
//            found = true;
//            break;
//        }
//    }
//    if (!found) {
//        Errors::raiseUserInputAlarm(String("command not found"));
//    }
//    delay(1000);
}
//
String SerialCommunication::checkForCommand() {
	String command="";
    if (Serial.available()) {                           // if there is data coming
        command = Serial.readStringUntil('\n');  // read string until newline character
        Serial.print("Execute command ");
        Serial.println(command);
        int index = command.indexOf(" ");
//        String extra = "";
//        if (index > 0) {
//            extra = command.substring(index + 1);
//            //			Serial.print(arg);
//            command = (command.substring(0, index));
//        }
//        int str_len = command.length() + 1;
//        char tok_array[str_len];
//        command.toCharArray(tok_array, str_len);
       //executeCommand(tok_array, extra);
    }
    return command;
}
