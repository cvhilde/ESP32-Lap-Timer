#include <storage.h>
#include <FS.h>
#include <SPIFFS.h>
#include <globals.h>
#include <gps.h>

String currLogFile = "";
String currTimeLogFile = "";

unsigned long lastLogTime = 0;
unsigned long bufferWaitTime = 5000;    //5 second buffer
String logBuffer;

void initStorage() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
    }
}

// starts session by updating the curr Strings, and creating the file
// prints the csv file header for both, and updates the sessions.txt manifest
void startSession() {
    uint16_t year = gps.getYear();
    uint8_t month = gps.getMonth();
    uint8_t day = gps.getDay();
    uint8_t hour = gps.getHour() - 4; // timezone difference
    uint8_t minute = gps.getMinute();
    uint8_t second = gps.getSecond();

    if (hour < 0) {
        hour += 24;
    }

    char timestamp[50];
    sprintf(timestamp, "%04d%02d%02d_%02d%02d", year, month, day, hour, minute);
    currentTimestamp = String(timestamp);

    currLogFile = "/log_" + currentTimestamp + ".csv";
    currTimeLogFile = "/timestamps_" + currentTimestamp + ".csv";

    Serial.printf("GNSS NAV-PVT time: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    File log = SPIFFS.open(currLogFile, FILE_WRITE);
    File time = SPIFFS.open(currTimeLogFile, FILE_WRITE);
    if (!log || !time) {
        Serial.println("Failed to create session files");
    }

    log.println("Latitude,Longitude,Speed(MPH)");
    time.println("LapNumber,Laptime,Sector1,Sector2,Sector3");

    log.close();
    time.close();

    File manifest = SPIFFS.open("/sessions.txt", FILE_APPEND);
    if (manifest) {
        manifest.println(currentTimestamp);
        manifest.close();
    }

    Serial.println("Sessions started: " + currentTimestamp);
    sessionActive = true;
}

// writes incoming updates in a log, and will only flush to the log file
// every 5 seconds. done to reduce amount of times opening and closing file
void writeToLogFile(double lat, double lng, double speed) {
    char str[128];
    sprintf(str, "%.8lf,%.8lf,%.2lf\n", lat, lng, speed);

    logBuffer += str;

    if (logBuffer.length() > 1024 || millis() - lastLogTime >= bufferWaitTime) {
        lastLogTime = millis();

        File log = SPIFFS.open(currLogFile, FILE_APPEND);
        if (log) {
            log.print(logBuffer);
            log.close();
            
            logBuffer = "";
        }
    }
}

void writeToTimeLog(unsigned long lapTime, unsigned long sector1, unsigned long sector2, unsigned long sector3) {
    char str[128];
    sprintf(str, "%d,%lu,%lu,%lu,%lu\n", lapNumber, lapTime, sector1, sector2, sector3);

    File timeLog = SPIFFS.open(currTimeLogFile, FILE_APPEND);
    if (timeLog) {
        timeLog.print(str);
        timeLog.close();
    }
}