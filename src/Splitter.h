#ifndef SPLITTER_H_
#define SPLITTER_H_

#include "Arduino.h"
#include "Errors.h"

class Splitter final {
public:
	Splitter(String str);
	virtual ~Splitter();
	String getItemAtIndex(int index);
	int getItemCount();
private:
	const static int maxSize=3;
	String strings[maxSize];
	int size=0;
};

#endif /* SPLITTER_H_ */
