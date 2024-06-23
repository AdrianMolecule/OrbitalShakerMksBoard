// main Orbital Shaker working code using the MKS board. Supports temperature, buzzer, stepper, switch...
// FOR MAKERBASE BOARD :           C:\a\diy\eagle\makerBaseDlc32v2.1.002/MKS_DLC32 _V2.1_002_SCH.pdf
// https://github.com/makerbase-mks/MKS-DLC32/blob/main/MKS-DLC32-main/doc/DLC32%20wiring%20manual.pdf
// int pins[] = { 2, 5,23, 25, 26, 27, /*32 is the spindle heater*/ 33 };// LED, 5=EXT1-3, 23=EXT2-6, 25=EXT1-7, 26=EXT1-5, 27=EXT1-4 or 8
#include <Melody.h>

#include "Arduino.h"
#include "DHTesp.h"  //for DHT temp sensor
// memory card https://www.mischianti.org/2021/03/28/how-to-use-sd-card-with-esp32-2/
//  include the SD library:
#include <DallasTemperature.h>
#include <FS.h>
#include <OneWire.h>
#include <SD.h>
#include <SPI.h>

#include "SerialCommunication.h"
#include "Splitter.h"
//***********SD card********** */
// Suggest SD card: Class4 or Class10; 4~16G memory; Fat32 format. File format support: .NC; .GC; .GCODE
// SD memory card SD_D0=I12 ,SD_DI=IO13,SD_CK=IO14,SD_CS=IO15,SD_DET=IO39/SENSOR_VN/ based on the C:\a\diy\my machines and tools\my components\makerBaseDlc32v2.1.002.sch
/* Pins_Arduino_h */
// music
#define SERIAL_BAUDRATE 115200
//********STEPPER***********
// we use PROBE_PIN=22 for stepper so connect PROBE S to DR8825 so we bend out the STEP pin of the leftmost X driver and put a wire
// The step pin is the second from the left bottom see C:\a\diy\eagle\makerBaseDlc32v2.1.002MKS_DLC32 _V2.1_002_SCH.pdf

// and we connect that to ProbeS Lower pin of connector J12 as
// for temperature sensor use either the dh TempAndHumidity with GND and 5 Volt from Probe connector and
//  DAT to DHT_PIN=19 which is on extension LCD_MISO
//
//*********BUZZER***********
// for speaker we use GND and on + in series with 100  OHM we use (GPIO23 which is the same as LCD_MOSI).
// That means LCD_MOSI or Extension X2 pin 6 ---Buzzer with proper polarity --[100 Ohm]---GND
//
//*********TEMP Sensor
// we use one main pin for euther dh or OneWIre and that is pin GPIO19 on pin 1 of Expansion 2 connector***********
//
//*********MOTOR ON OFF Switch***********
// REMOVED TODO CHANGE WITH CAPACITIVE  connect switch from GND to TURN_ON_STEPPER_PIN = 36/Sensor_VP SVP/- Xlimit on J9 (last to the right lower pin)) MOTOR starts when pin36 is put to Gnd
//
//  for motor current // Vref = Imax /2
// for m17hs16-2004s1 https://www.omc-stepperonline.com/nema-17-bipolar-45ncm-64oz-in-2a-42x42x40mm-4-wires-w-1m-cable-connector-17hs16-2004s1 and Imax=2 AMp hold torque=Holding Torque: 45Ncm(64oz.in)
// and https://www.youtube.com/watch?v=BV-ouxhZamI
//   to measure on screw-driver=2*8*.068 about 1 volt but I'll go less maybe .9 volts
//
// good doc for pins at https://github.com/diruuu/FluidNC/blob/main/example_configs/MKS-DLC32-v2.0.yaml
enum {  // the number means IO pin heater=32 means heather is GPIO32
    HEATER_PIN = 32 /*this is internally connected to spindle*/,
    HEATER_PWM_CHANNEL = 0,
    STEPPER_PWM_STEP_PIN = 22 /*probe pin*/,
    STEPPER_PWM_CHANNEL = 2, /*NOT USED I2SO_BUZZ_PIN = 0000???,*/
    I2SO_DATA_PIN = 21,
    I2SO_CLOCK_PIN = 16,
    I2SO_LATCH_PIN = 17,
    LED_PIN = 2,
    POTENTIOMETER_PIN = 127,
    TEMP_SENSOR_PIN = 19 /* LCD_MISO*/,
    TURN_ON_STEPPER_PIN = 36 /*Sensor_VP SVP -*/,
    SPEAKER = 23,
    /*LCD_MOSI*/ SPEAKER_CHANNEL = 4,
    FAN_PIN = 33 /*LCD_RS*/,
    FAN_PWM_CHANNEL = 6,
    MEMORY_CS_PIN = 15,
    /*used internally by SD no need to declare*/ SD_D0_PIN = 12,
    SD_DI = 13,
    SD_CK = 14,
    SD_CS = 15,
    SD_DET = 39,
    SPINDLE_ENABLE_PIN = 27, /*internally connected*/
};
//
enum {  // for behaviour
    TIME_DISPLAY = true,
    TEMPERATURE_DISPLAY = true,
    MOST_MUSIC_OFF = false,  // turns off all music except for errors, warnings, time reached and first time desired temperature reached
    TEMPERATURE_REACHED_MUSIC_ON = true,
    // for hardware config
    USE_STEPPER_ON_OFF_SWITCH = false,// on case we add an optional ON/OFF button for all the rotator. It could be a capacitive one.
    USE_ONE_WIRE_FOR_TEMPERATURE=true,
};
//
enum {
    DEBUG_HEATER = true,
    DEBUG_FAN = false,
    DEBUG_SWITCH = true,
    DEBUG_LOW_HUMIDITY = true
};
// FAN https://fdossena.com/?p=ArduinoFanControl/i.md
// timer
// temperature sensor stuff
const float MODERATE_HEAT_POWER = 0.9;
// Temperature either dh or oneWireDallas
DHTesp dhTempSensor;               // used in setup and readTemperature
OneWire oneWire(TEMP_SENSOR_PIN);  // GPIO where the DS18B20 is connected to. Used to be GPIO36 but not anymore
DallasTemperature tempSensor(&oneWire);
int lastTempHumidityReadTime = 0;  // never
int lastAlertTime = 0;             // never
float desiredTemperature = 37.0;   // seed it
float desiredRPM = 80;
int desiredEndTime=-1;//in minutes
int microstepping = 4;
float oldTemperature = 0.;
float minHumidity = 60.;
boolean heaterWasOn = false;
boolean firstTimeTurnOnHeater = true;
boolean firstTimeReachDesiredTemperature = true;
float maxTemperature = 0;
// motor what we want for OS motor
// https://github.com/nenovmy/arduino/blob/master/leds_disco/leds_disco.ino
/* Setting all PWM motor, Heater PWM Properties */
const int PWMResolution = 10;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
// fan
const int halfDutyCycle = MAX_DUTY_CYCLE / 2;
const float fanDutyCyclePercentage = 1;
const int FAN_FREQUENCY = 40000;
// heater pad
int maxHeaterDutyCycle = MAX_DUTY_CYCLE * .75;  // 80x100mm 12V DC 20W Silicone Heated Bed Heating Pad
// de-bounce variables
const int DEBOUNCE_TIME = 200;
unsigned long lastDebounceTime = 0;
int lastSteadyState = LOW;        // the previous steady state from the input pin
int lastFlickerableState = -100;  // the previous flicker-able state from the input pin
int lastButtonState;              // the current reading from the input pin which could be unreliable due to vibration

// declare some methods because we don't have an include file for this .ino file
void play(Melody melody);
void play(Melody melody, bool force);
void startStepper();
void stopStepper();
void fanSetup();
void fan(bool);
int readPotentiometer();
void heater(boolean on, int duty);
String getFormatedTimeSinceStart();
void desiredEndTimeCheck();
void setupI2SOShiftEnableMotor();
void setupI2SOShiftDisableMotor();
void createDir(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const String &message);
String formatTime(unsigned long time);
String getFormatedTimeSinceStart();
int readTurnOnStepper();
void setupSDCard();
void printDirectory(File dir, int numTabs);
int getTemperature(float &temp, float &humid);
void writeData(byte *bits);
void processCommand();
double rpmToHertz(float rpm);
// end INCLUDES.h
// music;
Melody scaleLouder("c>>> d>> e>f g< a<< b<<< c*<<<<", 480);
Melody invalidChoice(" (cg_)__");
Melody validChoice(" (cgc*)**---");
Melody frereJacques("(cdec)x2   (efgr)x2;//   ((gagf)-ec)x2     (c g_ c+)x2");
Melody frereJacquesFull("(cdec)x2   (efgr)x2  ((gagf)-ec)x2     (c g_ c+)x2");
Melody auClairDeLaLune("cccde+dr  ceddc+.r");
Melody darthVader(
    " (ggg e,-. b,-- | g e,-. b,-- g+ (ddde,-.)* b,--  | g, e,-. b,-- g+ | g* g-.g--  (g g,-. f-- (ed#)-- e-)* r- g#- c#* b#-.b-- |  (b,a)-- b,- r- e,- g, e,-. g,-- | b, g-. b,-- d*+  | g* g-.g--  (g g,-. f-- (ed#)-- e-)* r- g#- c#* b#-.b-- |  (b,a)-- b,- r- e,- g, e,-. b,-- | g e,-. b,-- g+ |)<<_ ");
// time
unsigned long startTime = millis();  // in ms
// SD memory card
const int32_t SPIfreq = 40000;
SerialCommunication serialCommunication = SerialCommunication();
//
// need to Open Folder C:\a\diy\espProjects\OrbitalShakerAdrianBoard for the libraries

void setup() {
    Serial.begin(115200);
    Serial.println("================================= Setup Starting Now =============================");
    // Declare pins as output:
    pinMode(LED_PIN, OUTPUT);
    pinMode(STEPPER_PWM_STEP_PIN, OUTPUT);
    pinMode(I2SO_LATCH_PIN, OUTPUT);
    pinMode(I2SO_CLOCK_PIN, OUTPUT);
    pinMode(I2SO_DATA_PIN, OUTPUT);
    // pinMode(FAN_PIN, OUTPUT);
    pinMode(TURN_ON_STEPPER_PIN, INPUT);
    pinMode(SPEAKER, OUTPUT);
    // alternate(LED_PIN, 50, 5);
    //  speaker
    ledcSetup(SPEAKER_CHANNEL, 5000, 8);
    ledcAttachPin(SPEAKER, SPEAKER_CHANNEL);
    ledcWrite(SPEAKER_CHANNEL, 0);  // duty Cycle = 0
    play(validChoice);
    // temperature sensor
    if (!USE_ONE_WIRE_FOR_TEMPERATURE) {
        dhTempSensor.setup(TEMP_SENSOR_PIN, DHTesp::DHT22);
    } else {
        tempSensor.begin();
    }
    Serial.println("Temp sensor initiated");
    // heater
    pinMode(HEATER_PIN, OUTPUT);
    // setup the heater
    ledcSetup(HEATER_PWM_CHANNEL, 40000, PWMResolution);  // normal PWM frrequency for MKS is 5000HZ
    delay(20);
    ledcAttachPin(HEATER_PIN, HEATER_PWM_CHANNEL); /* Attach the STEP_PIN PWM Channel to the GPIO Pin */
    delay(20);
    Serial.println("stepper desiredRPM:");
    Serial.println(desiredRPM);
    // motor
    Serial.println("Initial disabling of the stepper in setup");
    stopStepper();
    play(scaleLouder);
    fanSetup();
    // setupSDCard();
    if (!USE_STEPPER_ON_OFF_SWITCH) {
        startStepper();
    }
    // oneFullRotation();
    Serial.println("=================================END Setup==================================");
}

//
void loop() {
    processCommand();
    if (USE_STEPPER_ON_OFF_SWITCH) {
        int stepperButtonPosition = readTurnOnStepper();  //-1 for unchanged
        Serial.print("stepperButtonPosition:" + String(stepperButtonPosition));
        if (stepperButtonPosition == 1) {
            startStepper();
            fan(false);
        } else {
            if (stepperButtonPosition == 0) {
                stopStepper();
                fan(true);
            }
        }
    }
    desiredEndTimeCheck();
        // Get temperature
        float temperature;
    float humidity;
    if (((millis() - lastTempHumidityReadTime) / 1000) > 2) {  // every 2 seconds
        getTemperature(temperature, humidity);
        lastTempHumidityReadTime = millis();
        if (temperature > maxTemperature) {
            maxTemperature = temperature;
        }
        if (TEMPERATURE_DISPLAY) {
            Serial.print("Current temp: " + String(temperature) + ", " + (USE_ONE_WIRE_FOR_TEMPERATURE == 0 ? "Humidity:" + String(humidity) : ""));
            Serial.print("DesiredTemperature: ");
            Serial.print(desiredTemperature);
            Serial.print(" max temperature: " + String(maxTemperature));
            if (TIME_DISPLAY) {
                Serial.println(" Time since start: " + getFormatedTimeSinceStart());
            } else {
                Serial.println("");
            }
        }
        if (temperature < desiredTemperature) {
            if (firstTimeTurnOnHeater) {
                play(auClairDeLaLune);
                firstTimeTurnOnHeater = false;
            }
            digitalWrite(LED_PIN, HIGH);
            if (desiredTemperature - temperature >= 2) {
                heater(true, maxHeaterDutyCycle);  // Heater start
            } else {
                heater(true, maxHeaterDutyCycle * MODERATE_HEAT_POWER);
            }
            heaterWasOn = true;
        } else {                // no need to heat
            if (heaterWasOn) {  // was on before this, used to stop printing too many times
                Serial.println("Turning heater OFF");
                if (firstTimeReachDesiredTemperature) {
                    if (TEMPERATURE_REACHED_MUSIC_ON) {
                        play(frereJacquesFull, true);
                    }
                    firstTimeReachDesiredTemperature = false;
                } else
                    play(frereJacques);
            }
            digitalWrite(LED_PIN, LOW);
            heater(false, 0);  // second arg is ignored when heater is turned off
            heaterWasOn = false;
            if (!USE_ONE_WIRE_FOR_TEMPERATURE && humidity < minHumidity && ((millis() - lastAlertTime) / 1000) > 200 /* about 3 minutes*/) {
                Serial.println("WARNING !!! humidity dropped less then minimal humidity");
                lastAlertTime = millis();
                if (DEBUG_LOW_HUMIDITY) {
                    play(invalidChoice);
                }
            }
        }
    }
}

void processCommand() {
    String command = "";
    String commandArguments = "";
    serialCommunication.checkForCommand(command, commandArguments);
    if (!command.equals("")) {
        if (command.indexOf("d") == 0) {
            File dir = SD.open("/");
            printDirectory(dir, 0);
        } else if (command.indexOf("s") == 0) {
            // Splitter splitter = Splitter(command);
            // String newRPMAsString = splitter.getItemAtIndex(1);
            if (commandArguments.isEmpty()) {
                Serial.print("Please enter the new desired RPM after s. Like s 80 for RPM=80");
            }
            desiredRPM = commandArguments.toFloat();
            Serial.println("new RPM is: " + commandArguments);
            startStepper();
        } else if (command.indexOf("t") == 0) {
            if (commandArguments.isEmpty()) {
                Serial.print("Please enter the new desired temperature . Like command 29 for temp 29 Celsius");
            }
            desiredTemperature = commandArguments.toFloat();
            Serial.print("new desiredTemperature is: ");
            Serial.println(desiredTemperature);
        } else if (command.indexOf("m") == 0) {
            if (commandArguments.isEmpty()) {
                Serial.print("Please enter m 0 for motor off or m anything else for motor on");
            }
            int arg = commandArguments.toInt();
            if (arg == 0) {
                stopStepper();
            } else {
                startStepper();
            }
            Serial.print("new desiredTemperature is: ");
            Serial.println(desiredTemperature);
        } else if (command.indexOf("r") == 0) {
            startTime = millis();
            desiredTemperature = commandArguments.toFloat();
            Serial.println("Reset start time to now ");
        } else if (command.indexOf("e") == 0) {
            if (commandArguments.isEmpty()) {
                Serial.print("Please enter an end time in minutes");
            }
            desiredEndTime = commandArguments.toInt();
            Serial.print("new desiredEndTime is: ");
            Serial.println(desiredEndTime);
        } else if (command.indexOf("?") == 0) {
            Serial.println("=========== current values ============");
            Serial.println("desiredRPM: " + String(desiredRPM));
            Serial.println("desiredTemperature: " + String(desiredTemperature));
            Serial.println("timeSinceStartOrTimeReset: " + getFormatedTimeSinceStart());
            Serial.println("=========== Commands help ============");
            Serial.println("Possible commands:\n s desired RPM. Like s 80 for RPM=80");
            Serial.println("r reset start time to now");
            Serial.println("m 0 stop rotator, m anything else start rotation");
            Serial.println("e time in minutes End time in minutes that will trigger an alarm to be played. So e 60 meand play alarm in 1 hour");
        } else {
            Serial.print("cannot find command ");
            Serial.println(command);
        }
    }
}

void startStepper() {
    Serial.println("START stepper gradually");
    setupI2SOShiftEnableMotor();
    for (int rpm = 5; rpm <= desiredRPM; rpm += 5) {
        double f = rpmToHertz(rpm);
        Serial.println("START STEPPER with frequency:" + String(f) + " and RPM:" + String(rpm));
        ledcSetup(STEPPER_PWM_CHANNEL, f, PWMResolution);
        // delay(5);
        ledcAttachPin(STEPPER_PWM_STEP_PIN, STEPPER_PWM_CHANNEL); /* Attach the STEP_PIN PWM Channel to the GPIO Pin */
        // delay(5);
        ledcWrite(STEPPER_PWM_CHANNEL, halfDutyCycle);
        // if (rpm < 40) {
        //     delay(300);  // this could be adjusted based on rpm but it seems to work fine
        // } else {
        delay(500);
        //}
    }
    double f = rpmToHertz(desiredRPM);
    Serial.println("STARTED STEPPER with frequency:" + String(f) + " and RPM:" + String(desiredRPM));
    ledcSetup(STEPPER_PWM_CHANNEL, f, PWMResolution);
    // delay(5);
    ledcAttachPin(STEPPER_PWM_STEP_PIN, STEPPER_PWM_CHANNEL); /* Attach the STEP_PIN PWM Channel to the GPIO Pin */
    // delay(5);
    ledcWrite(STEPPER_PWM_CHANNEL, halfDutyCycle);
    // play(auClairDeLaLune);
}

double rpmToHertz(float rpm) {
    return microstepping * (int)(rpm / 60 * 200);  // in hertz
}
//
void stopStepper() {
    Serial.println("STOP STEPPER");
    setupI2SOShiftDisableMotor();
    ledcDetachPin(STEPPER_PWM_STEP_PIN); /* Detach the STEP_PIN PWM Channel to the GPIO Pin */
    delay(200);
}

int getTemperature(float &temp, float &humid) {
    if (USE_ONE_WIRE_FOR_TEMPERATURE) {
        tempSensor.requestTemperatures();
        // Serial.print("Temperature: ");  // print the temperature in Celsius
        temp = tempSensor.getTempCByIndex(0);
        // Serial.println(temp);
    } else {
        // Reading temperature for humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
        TempAndHumidity newValues = dhTempSensor.getTempAndHumidity();
        // Check if any reads failed and exit early (to try again).
        if (dhTempSensor.getStatus() != 0) {
            Serial.println("DHT12 error status: " + String(dhTempSensor.getStatusString()));
            play(auClairDeLaLune);
            return false;
        }
        temp = newValues.temperature;
        if (temp < 0 || temp > 70) {
            for (int i = 0; i < 10; i++) {
                play(darthVader, true);// temperature out of range
            }
        }
        humid = newValues.humidity;
    }
    return true;
}

// current https://reprap.org/wiki/NEMA_17_Stepper_motor 17hs16 20044s1(black ones) rated for 2 A
// adjustment guide https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/ Vref on Pot = max current /2 =1V
// xDir is 2 xStep is 1 and beeper is 7
void setupI2SOShiftEnableMotor() {
    // Serial.println("#################### setupI2SOShift   EnableMotor #####################################");
    byte bits[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // enable is pin 0 and should be low to start stepper, last one is the beeper
    writeData(bits);
    delay(100);
}
//
void setupI2SOShiftDisableMotor() {
    // Serial.println("#################### setupI2SOShift    DisableMotor #####################################");
    byte bits[16] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0};  // enable is pin 0 and should be high to stop stepper, last one is the beeper
    writeData(bits);
    delay(10);
}
//
void writeData(byte *bits) {
    // alternate data we can just capture the 2 data values like 2254, 2124
    int data1 = 0;
    for (int i = 0; i < 8; i++) {
        data1 |= bits[i + 8] << i;
    }
    int data2 = 0;
    for (int i = 0; i < 8; i++) {
        data2 |= bits[i] << i;
    }
    // disable update
    digitalWrite(I2SO_LATCH_PIN, LOW);
    // shift out the data (second shift register must be send first)
    shiftOut(I2SO_DATA_PIN, I2SO_CLOCK_PIN, MSBFIRST, data1);  // NOT NEEDED FOR X axes
                                                               // Serial.print("data2");Serial.println(data2);
    shiftOut(I2SO_DATA_PIN, I2SO_CLOCK_PIN, MSBFIRST, data2);  //*
                                                               // update the shift register output pins
    digitalWrite(I2SO_LATCH_PIN, HIGH);
}

//
// void alternate(int pin, int de, int times) {
// //	Serial.println("alternate on pin:");
// //	Serial.println(pin);
// 	for (int var = 0; var < times; ++var) {
// 		digitalWrite(pin, HIGH);
// 		delay(de);
// 		digitalWrite(pin, LOW);
// 		delay(de);
// 	}
// }

// Function for reading the Potentiometer
int readPotentiometer() {
    uint16_t customDelay = analogRead(POTENTIOMETER_PIN);  // Reads the potentiometer
    int newRPM = map(customDelay, 0, 1023, 0, 300);        // read values of the potentiometer from 0 to 1023 into  d0->300
    return 300;
}

// WeMos D1 esp8266: D8 as standard
const int chipSelect = SS;

void setupSDCard() {
    Serial.print("\nInitializing SD card...");
    SPIClass hspi = SPIClass(HSPI);                           // HSPI has the 12-15 pins already configured // actually a reference
    if (!SD.begin(MEMORY_CS_PIN, hspi, SPIfreq, "/sd", 2)) {  // copied from Fluid SDCard.cpp //if (SD.begin(csPin, SPI, SPIfreq, "/sd", 2)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        while (1);
    } else {
        Serial.println("A card is present.");
    }
    // print the type of card
    Serial.println();
    Serial.print("Card type:    ");
    switch (SD.cardType()) {
        case CARD_NONE:
            Serial.println("NONE");
            break;
        case CARD_MMC:
            Serial.println("MMC");
            break;
        case CARD_SD:
            Serial.println("SD");
            break;
        case CARD_SDHC:
            Serial.println("SDHC");  // for me it prints this!
            break;
        default:
            Serial.println("Unknown");
    }
    // print the type and size of the first FAT-type volume
    //  uint32_t volume size;
    //  Serial.print("Volume type is:    FAT");
    //  Serial.println(SDFS.usefatType(), DEC);
    Serial.print("Card size:  ");
    Serial.println((float)SD.cardSize() / 1000);
    Serial.print("Total bytes: ");
    Serial.println(SD.totalBytes());
    Serial.print("Used bytes: ");
    Serial.println(SD.usedBytes());
    File dir = SD.open("/", FILE_READ);
    createDir(SD, "/adriandir");
    printDirectory(dir, 0);
    writeFile(SD, "/lastWrittenTime.txt", formatTime(millis()));
    writeFile(SD, "/hello.txt", "  Hello");
    Serial.println("after writeFile and adrianDir subdirectory ListDir");
}
//
void printDirectory(File dir, int numTabs) {
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) {  // no more files
            break;
        }
        for (uint8_t i = 0; i < numTabs; i++) {
            Serial.print('\t');
        }
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
            printDirectory(entry, numTabs + 1);
        } else {
            // files have sizes, directories do not
            Serial.print("\t\t");
            Serial.print(entry.size(), DEC);
            time_t lw = entry.getLastWrite();
            struct tm *tmstruct = localtime(&lw);
            Serial.printf("\tLAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
        }
        entry.close();
    }
}

void writeFile(fs::FS &fs, const char *path, const String &message) {
    Serial.printf("Writing file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

/** return -1 if unchanged and 0 for off and 1 for on*/
int readTurnOnStepper() {
    lastButtonState = !digitalRead(TURN_ON_STEPPER_PIN);
    int ret = -1;  // means unchanged
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:
    // If the switch/button changed, due to noise or pressing:
    if (lastButtonState != lastFlickerableState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
        // save the the last flickerable state
        lastFlickerableState = lastButtonState;
    }
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:
        // if the button state has changed:
        if (lastSteadyState == HIGH && lastButtonState == LOW) {
            if (DEBUG_SWITCH) {
                Serial.println("The stepper motor button was turned off");
                play(validChoice);
            }
            ret = 0;
        } else if (lastSteadyState == LOW && lastButtonState == HIGH) {
            if (DEBUG_SWITCH) {
                Serial.println("The stepper motor button was turned on");
                play(validChoice);
            }
            ret = 1;
        }
        // save the the last steady state
        lastSteadyState = lastButtonState;
    }
    return ret;
}
//
void setLoudness(int loudness) {
    // Loudness could be use with a mapping function, according to your buzzer or sound-producing hardware
    const int MIN_HARDWARE_LOUDNESS = 0;
    const int MAX_HARDWARE_LOUDNESS = 16;
    ledcWrite(SPEAKER_CHANNEL, map(loudness, -4, 4, MIN_HARDWARE_LOUDNESS, MAX_HARDWARE_LOUDNESS));
}
//
String getFormatedTimeSinceStart() {
    unsigned long time = (unsigned long)((millis() - startTime) / 1000);  // finds the time since last print in secs
    return formatTime(time);
}
//
void fanSetup() {
    pinMode(FAN_PIN, OUTPUT);
    ledcSetup(FAN_PWM_CHANNEL, FAN_FREQUENCY /*Hz*/, 10 /*resolution 2 power=256 values*/);
    fan(true);
    Serial.print("Fan pin ");
    Serial.print(FAN_PIN);
    Serial.print(" on channel ");
    Serial.print(FAN_PWM_CHANNEL);
    Serial.print(" at frequency ");
    Serial.print(FAN_FREQUENCY);
    Serial.print(" at duty cycle ");
    Serial.print(MAX_DUTY_CYCLE * fanDutyCyclePercentage);
    Serial.print(" or percentage ");
    Serial.println(fanDutyCyclePercentage);
    delay(3000);
    fan(false);
}
//
void fan(boolean on) {
    if (on) {
        ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
        ledcWrite(FAN_PWM_CHANNEL, MAX_DUTY_CYCLE * fanDutyCyclePercentage);
    } else {
        ledcDetachPin(FAN_PIN);
        ledcWrite(FAN_PWM_CHANNEL, 0);
    }
    if (DEBUG_FAN) {
        Serial.print("Fan set to: ");
        Serial.println(on);
    }
}

void heater(boolean on, int duty) {
    // this works only after setup was called to initialize the channel
    if (on) {
        ledcWrite(HEATER_PWM_CHANNEL, duty);  // Heater start
    } else {
        ledcWrite(HEATER_PWM_CHANNEL, 0);  // Heater stop
    }
    if (DEBUG_HEATER) {
        Serial.print("Heater set to: ");
        Serial.print(on);
        if (on) {
            Serial.print(" with duty at:");
            Serial.print(((float)duty / MAX_DUTY_CYCLE));
            Serial.print("%");
        }
        Serial.println();
    }
}

//
String formatTime(unsigned long time) {
    String result = "";
    int hours = (unsigned long)(time / 3600);
    if (hours > 0) {
        result = +hours;
        result += ("h ");
    }
    result += ((unsigned long)(time % 3600) / 60);
    result += ("m ");
    result += (time % 60);
    result += ("s");
    return result;
}
//
void play(Melody melody) {
    if (MOST_MUSIC_OFF) {
        return;
    }
    melody.restart();         // The melody iterator is restarted at the beginning.
    while (melody.hasNext())  // While there is a next note to play.
    {
        melody.next();                                   // Move the melody note iterator to the next one.
        unsigned int frequency = melody.getFrequency();  // Get the frequency in Hz of the curent note.
        unsigned long duration = melody.getDuration();   // Get the duration in ms of the curent note.
        unsigned int loudness = melody.getLoudness();    // Get the loudness of the curent note (in a subjective relative scale from -3 to +3).
                                                         // Common interpretation will be -3 is really soft (ppp), and 3 really loud (fff).
        if (frequency > 0) {
            ledcWriteTone(SPEAKER_CHANNEL, frequency);
            setLoudness(loudness);
        } else {
            ledcWrite(SPEAKER_CHANNEL, 0);
        }
        delay(duration);
        // This 1 ms delay with no tone is added to let a "breathing" time between each note.
        // Without it, identical consecutives notes will sound like just one long note.
        ledcWrite(SPEAKER_CHANNEL, 0);
        delay(1);
    }
    ledcWrite(SPEAKER_CHANNEL, 0);
    delay(1000);
}

void play(Melody melody, bool force) {
    if (force) {
        play(melody);
    }
}

void desiredEndTimeCheck(){
    if (desiredEndTime != -1 && desiredEndTime * 60 * 1000 >= (millis()-startTime)) {
        Serial.println("Desired end time reached " + getFormatedTimeSinceStart());
        play(scaleLouder, true);
    }
}
void createDir(fs::FS &fs, const char *path) {
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path)) {
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

//
// void oneFullRotation(){
//	Serial.print("oneFullRotation ");
//	play(validChoice);
//	setupI2SOShiftEnableMotor();
//	for(int i=0;i<(4*200)+1;i++){
//		digitalWrite(STEPPER_PWM_STEP_PIN, HIGH);
//		delay(5);
//		digitalWrite(STEPPER_PWM_STEP_PIN, LOW);
//		delay(5);
//	}
//	Serial.print("oneFullRotation end");
//	play(validChoice);
//}
