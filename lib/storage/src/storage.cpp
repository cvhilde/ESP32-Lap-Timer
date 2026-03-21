#include <storage.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <globals.h>
#include <gps.h>

String waypointsFile = "/waypoints.json";
String currLogFile = "";
String currTimeLogFile = "";

unsigned long lastLogTime = 0;
unsigned long bufferWaitTime = 5000;    //5 second buffer
String logBuffer;
unsigned long logTimeBegin = 0;

constexpr size_t kRamLimit   = 180 * 1024;  // 180 kB ≈ 18 min @ 10 kB/min
constexpr size_t kLineMax    = 128; // longest CSV line

static char   logBuf[kRamLimit];
static size_t logPos = 0;   // # bytes currently used

void flushRamToFlash();
double storageUsage();


void initStorage() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
    }

    if (!loadWaypoints()) {
        Serial.println("Failed to load waypoints. Creating default file");
        File file = SPIFFS.open(waypointsFile, "w");
        file.close();

        storedWaypoints.wp1.p1.lat = 0.0;
        storedWaypoints.wp1.p1.lng = 0.0;
        storedWaypoints.wp1.p2.lat = 0.0;
        storedWaypoints.wp1.p2.lng = 0.0;
        storedWaypoints.wp1.isActive = 0;

        storedWaypoints.wp2.p1.lat = 0.0;
        storedWaypoints.wp2.p1.lng = 0.0;
        storedWaypoints.wp2.p2.lat = 0.0;
        storedWaypoints.wp2.p2.lng = 0.0;
        storedWaypoints.wp2.isActive = 0;

        storedWaypoints.wp3.p1.lat = 0.0;
        storedWaypoints.wp3.p1.lng = 0.0;
        storedWaypoints.wp3.p2.lat = 0.0;
        storedWaypoints.wp3.p2.lng = 0.0;
        storedWaypoints.wp3.isActive = 0;
    }
}

bool loadWaypoints() {
    File file = SPIFFS.open(waypointsFile, "r");
    if (!file) {
        Serial.println("No waypoint file");
        return false;
    }

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, file)) {
        Serial.println("JSON parse error");
        return false;
    }

    trackWaypoints[0].p1.lat = doc["wp1"][0];
    trackWaypoints[0].p1.lng = doc["wp1"][1];
    trackWaypoints[0].p2.lat = doc["wp1"][2];
    trackWaypoints[0].p2.lng = doc["wp1"][3];
    trackWaypoints[0].isActive = 1;

    trackWaypoints[1].p1.lat = doc["wp2"][0];
    trackWaypoints[1].p1.lng = doc["wp2"][1];
    trackWaypoints[1].p2.lat = doc["wp2"][2];
    trackWaypoints[1].p2.lng = doc["wp2"][3];
    trackWaypoints[1].isActive = 1;

    trackWaypoints[2].p1.lat = doc["wp3"][0];
    trackWaypoints[2].p1.lng = doc["wp3"][1];
    trackWaypoints[2].p2.lat = doc["wp3"][2];
    trackWaypoints[2].p2.lng = doc["wp3"][3];
    trackWaypoints[2].isActive = 1;

    Serial.printf("Waypoints loaded: start (%.6f,%.6f)\n", storedWaypoints.wp1.p1.lat, storedWaypoints.wp1.p1.lng);
    return true;
}

void writeWaypointsFile(const uint8_t* raw, size_t len) {
    File file = SPIFFS.open(waypointsFile, "w");
    file.write(raw, len);
    file.close();
    loadWaypoints();
}

// starts session by updating the curr Strings, and creating the file
// prints the csv file header for both, and updates the sessions.txt manifest
void startSession() {
    if (sessionActive) {
        return;
    }

    uint16_t year = gps.getYear();
    uint8_t month = gps.getMonth();
    uint8_t day = gps.getDay();
    uint8_t hour = gps.getHour() - 4; // timezone difference
    hour = (hour + 24) % 24;
    uint8_t minute = gps.getMinute();
    uint8_t second = gps.getSecond();

    char timestamp[25];
    sprintf(timestamp, "%04d%02d%02d_%02d%02d", year, month, day, hour, minute);
    currentTimestamp = String(timestamp);

    currLogFile = "/log_" + currentTimestamp + ".csv";
    currTimeLogFile = "/timestamps_" + currentTimestamp + ".csv";

    Serial.printf("GNSS NAV-PVT time: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    File logFile = SPIFFS.open(currLogFile, FILE_WRITE);
    File timeFile = SPIFFS.open(currTimeLogFile, FILE_WRITE);
    if (!logFile || !timeFile) {
        Serial.println("Failed to create session files");
    }

    logFile.println("Latitude,Longitude,Speed(MPH),Millis");
    timeFile.println("LapNumber,Laptime,Sector1,Sector2,Sector3");

    logFile.close();
    timeFile.close();

    logPos = 0;
    logTimeBegin = millis();

    File manifest = SPIFFS.open("/sessions.txt", FILE_APPEND);
    if (manifest) {
        manifest.println(currentTimestamp);
        manifest.close();
    }

    Serial.println("Sessions started: " + currentTimestamp);
    sessionActive = true;
}

void startRouteSession() {
    if (sessionActive) {
        return;
    }

    uint16_t year = gps.getYear();
    uint8_t month = gps.getMonth();
    uint8_t day = gps.getDay();
    uint8_t hour = gps.getHour() - 4; // timezone difference
    hour = (hour + 24) % 24;
    uint8_t minute = gps.getMinute();
    uint8_t second = gps.getSecond();

    char timestamp[25];
    sprintf(timestamp, "%04d%02d%02d_%02d%02d", year, month, day, hour, minute);
    currentTimestamp = String(timestamp);

    currLogFile = "/route_" + currentTimestamp + ".csv";
    Serial.printf("GNSS NAV-PVT time: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    File routeFile = SPIFFS.open(currLogFile, FILE_WRITE);
    if (!routeFile) {
        Serial.println("Failed to create route session file");
    }

    routeFile.println("Latitude,Longitude,Speed(MPH),Altitude(Ft),Millis");
    routeFile.close();
    
    logPos = 0;
    logTimeBegin = millis();

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
    if (!sessionActive) {
        return;
    }

    char line[kLineMax];
    int  n = snprintf(line, sizeof(line), "%.7lf,%.7lf,%.2lf,%lu\n", lat, lng, speed, millis() - logTimeBegin);

    if (logPos + n > kRamLimit) {
        flushRamToFlash();          // write the 180 kB chunk
        if (n > kRamLimit) return;
    }

    memcpy(logBuf + logPos, line, n);
    logPos += n;
}

void writeToTimeLog(unsigned long lapTime, unsigned long sector1, unsigned long sector2, unsigned long sector3) {
    char str[128];
    sprintf(str, "%d,%lu,%lu,%lu,%lu\n", lapNumber, lapTime, sector1, sector2, sector3);

    File timeFile = SPIFFS.open(currTimeLogFile, FILE_APPEND);
    if (timeFile) {
        timeFile.print(str);
        timeFile.close();
    }
}

void writeToRouteLog(double lat, double lng, double speed, double altitude) {
    if (!sessionActive) {
        return;
    }

    char line[kLineMax];
    int  n = snprintf(line, sizeof(line), "%.7lf,%.7lf,%.2lf,%.2lf,%lu\n", lat, lng, speed, altitude, millis() - logTimeBegin);

    if (logPos + n > kRamLimit) {
        flushRamToFlash();          // write the 180 kB chunk
        if (n > kRamLimit) return;
    }

    memcpy(logBuf + logPos, line, n);
    logPos += n;
}

void endSession() {
    flushRamToFlash();
    sessionActive = false;
}

void endRouteSession() {
    flushRamToFlash();
    sessionActive = false;
}

void flushRamToFlash() {
    if (logPos == 0) {
        return;
    }

    File logFile = SPIFFS.open(currLogFile, FILE_APPEND);
    if (logFile) {
        logFile.write((uint8_t*)logBuf, logPos);
        logFile.close();
    }
    logPos = 0;
}

double storageUsage() {
    size_t total = SPIFFS.totalBytes();   // size of the SPIFFS partition
    size_t used  = SPIFFS.usedBytes();    // how much is already occupied

    return (used * 100.0) / total;
}