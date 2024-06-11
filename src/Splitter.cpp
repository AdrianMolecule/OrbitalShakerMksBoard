#include "Splitter.h"
//splits a string in multiple strings at blank spaces
Splitter::Splitter(String str) {
	if (str.isEmpty()) {
		size = 0;
	}
	size = 1;
	for (int x = 0; x < str.length(); (str[x] == ' ') ? size++ : 0, x++)
		;
	if(size>maxSize){
		Errors::raiseUserInputAlarm(String("ERROR These command arguments have more than :") + String(maxSize)+String(" commands. Abandoning splitting ..."));
		size=0;
		return;
	}
	int stringPosition = 0;
	while (str.length() > 0) {
		int index = str.indexOf(' ');
		if (index == -1) { // No space found
			strings[stringPosition++] = str;
			break;
		} else {
			strings[stringPosition++] = str.substring(0, index);
			str = str.substring(index + 1);
		}
	}
	Serial.print("size:");
	Serial.println(size);
	Serial.println("values:");
	for (int x = 0; x < size; x++) {
		Serial.print("-");
		Serial.print(strings[x]);
		Serial.println("-,");
	}
}

Splitter::~Splitter() {
	//Serial.println("destructor"); //todo check for mem leaks
	for (int x = 0; x < size; x++) {
		strings[x].clear();
	}
}

//
int Splitter::Splitter::getItemCount() {
	return size;
}
//
String Splitter::getItemAtIndex(int index) {
	if (index > size - 1) {
		Serial.print("index not found, max index is:");
		Serial.println(size - 1);
		return String("");
	}
	// Serial.print("index:");
	// Serial.println(index);
	return strings[index];
}

