#include <storage.h>
#include <FS.h>
#include <SPIFFS.h>
#include <globals.h>
#include <gps.h>

void initStorage() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
    }
}

void startSession() {
    uint16_t year = gps.getYear();
    uint8_t month = gps.getMonth();
    uint8_t day = gps.getDay();
    uint8_t hour = gps.getHour() - 4;
    uint8_t minute = gps.getMinute();
    uint8_t second = gps.getSecond();

    if (hour < 0) {
        hour += 24;
    }

    char timestamp[50];
    sprintf(timestamp, "%04d%02d%02d_%02d%02d", year, month, day, hour, minute);
    currentTimestamp = String(timestamp);

    String logFile = "/log_" + currentTimestamp + ".csv";
    String timeFile = "/timestamps_" + currentTimestamp + ".csv";

    Serial.printf("GNSS NAV-PVT time: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    File log = SPIFFS.open(logFile, FILE_WRITE);
    File time = SPIFFS.open(timeFile, FILE_WRITE);
    if (!log || !time) {
        Serial.println("Failed to create session files");
    }

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