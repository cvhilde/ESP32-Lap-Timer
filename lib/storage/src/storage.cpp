#include <storage.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <globals.h>
#include <gps.h>
#include <math.h>

String waypointsFile = "/waypoints.json";
String currLogFile = "";
String currTimeLogFile = "";
String currSummaryFile = "";

unsigned long lastLogTime = 0;
unsigned long bufferWaitTime = 5000;    //5 second buffer
String logBuffer;
unsigned long logTimeBegin = 0;

constexpr size_t kRamLimit   = 180 * 1024;  // 180 kB ≈ 18 min @ 10 kB/min
constexpr size_t kLineMax    = 128; // longest CSV line
constexpr double kFeetPerMile = 5280.0;
constexpr double kMinDistanceSegmentFt = 3.0;
constexpr double kStationarySpeedMph = 1.0;
constexpr double kStationarySegmentRejectFt = 12.0;

static char   logBuf[kRamLimit];
static size_t logPos = 0;   // # bytes currently used
static double sessionDistanceFt = 0.0;
static double lastDistanceLat = 0.0;
static double lastDistanceLng = 0.0;
static bool haveLastDistancePoint = false;

void flushRamToFlash();
double storageUsage();

static void resetSessionDistance() {
    sessionDistanceFt = 0.0;
    lastDistanceLat = 0.0;
    lastDistanceLng = 0.0;
    haveLastDistancePoint = false;
}

static double distanceBetweenFeet(double lat1, double lng1, double lat2, double lng2) {
    double avgLatRad = ((lat1 + lat2) * 0.5) * DEG_TO_RADIANS;
    double x = (lng2 - lng1) * DEG_TO_RADIANS * cos(avgLatRad);
    double y = (lat2 - lat1) * DEG_TO_RADIANS;
    return sqrt((x * x) + (y * y)) * EARTH_RADIUS_FT;
}

static void updateSessionDistance(double lat, double lng, double speedMph) {
    if (!haveLastDistancePoint) {
        lastDistanceLat = lat;
        lastDistanceLng = lng;
        haveLastDistancePoint = true;
        return;
    }

    double segmentFt = distanceBetweenFeet(lastDistanceLat, lastDistanceLng, lat, lng);
    lastDistanceLat = lat;
    lastDistanceLng = lng;

    if (segmentFt < kMinDistanceSegmentFt) {
        return;
    }

    if (speedMph < kStationarySpeedMph && segmentFt < kStationarySegmentRejectFt) {
        return;
    }

    sessionDistanceFt += segmentFt;
}

static void writeSessionSummary(const char* sessionType) {
    if (currSummaryFile.isEmpty()) {
        return;
    }

    File summaryFile = SPIFFS.open(currSummaryFile, FILE_WRITE);
    if (!summaryFile) {
        Serial.println("Failed to create session summary file");
        return;
    }

    summaryFile.println("SessionType,TotalDistanceFt,TotalDistanceMi");
    summaryFile.printf("%s,%.2lf,%.5lf\n", sessionType, sessionDistanceFt, sessionDistanceFt / kFeetPerMile);
    summaryFile.close();
}


void initStorage() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
    }

    if (!loadWaypoints()) {
        Serial.println("Using hardcoded fallback waypoints");

        trackWaypoints[0].p1.lat = 28.613391;
        trackWaypoints[0].p1.lng = -81.179126;
        trackWaypoints[0].p2.lat = 28.613467;
        trackWaypoints[0].p2.lng = -81.179126;
        trackWaypoints[0].isActive = 1;

        trackWaypoints[1].p1.lat = 28.61392242;
        trackWaypoints[1].p1.lng = -81.17897079;
        trackWaypoints[1].p2.lat = 28.61392980;
        trackWaypoints[1].p2.lng = -81.17906224;
        trackWaypoints[1].isActive = 1;

        trackWaypoints[2].p1.lat = 28.61364765;
        trackWaypoints[2].p1.lng = -81.17963668;
        trackWaypoints[2].p2.lat = 28.61365188;
        trackWaypoints[2].p2.lng = -81.17954403;
        trackWaypoints[2].isActive = 1;
    }
}

bool loadWaypoints() {
    File file = SPIFFS.open(waypointsFile, "r");
    if (!file) {
        Serial.println("No waypoint file");
        return false;
    }

    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err) {
        Serial.printf("JSON parse error: %s\n", err.c_str());
        return false;
    }

    JsonArray wps = doc["waypoints"].as<JsonArray>();
    if (wps.isNull()) {
        Serial.println("Missing 'waypoints' array");
        return false;
    }

    if (wps.size() != 3) {
        Serial.printf("Expected 3 waypoints, got %u\n", (unsigned)wps.size());
        return false;
    }

    for (int i = 0; i < 3; i++) {
        JsonObject wp = wps[i];

        if (!wp["p1"].is<JsonObject>() || !wp["p2"].is<JsonObject>()) {
            Serial.printf("Waypoint %d missing p1/p2 object\n", i);
            return false;
        }

        if (!wp["p1"]["lat"].is<float>() || !wp["p1"]["lng"].is<float>() ||
            !wp["p2"]["lat"].is<float>() || !wp["p2"]["lng"].is<float>()) {
            Serial.printf("Waypoint %d has invalid coordinate fields\n", i);
            return false;
        }

        trackWaypoints[i].p1.lat = wp["p1"]["lat"].as<double>();
        trackWaypoints[i].p1.lng = wp["p1"]["lng"].as<double>();
        trackWaypoints[i].p2.lat = wp["p2"]["lat"].as<double>();
        trackWaypoints[i].p2.lng = wp["p2"]["lng"].as<double>();
        trackWaypoints[i].isActive = wp["active"] | 1;
    }

    Serial.printf(
        "Waypoints loaded: start (%.6f, %.6f) -> (%.6f, %.6f)\n",
        trackWaypoints[0].p1.lat,
        trackWaypoints[0].p1.lng,
        trackWaypoints[0].p2.lat,
        trackWaypoints[0].p2.lng
    );

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
    currSummaryFile = "/summary_" + currentTimestamp + ".csv";

    Serial.printf("GNSS NAV-PVT time: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    File logFile = SPIFFS.open(currLogFile, FILE_WRITE);
    File timeFile = SPIFFS.open(currTimeLogFile, FILE_WRITE);
    if (!logFile || !timeFile) {
        Serial.println("Failed to create session files");
    }

    logFile.println("Latitude,Longitude,Speed(MPH),Millis,LapNumber,IsAirborne");
    timeFile.println("LapNumber,Laptime,Sector1,Sector2,Sector3");

    logFile.close();
    timeFile.close();

    logPos = 0;
    logTimeBegin = millis();
    resetSessionDistance();

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
    currSummaryFile = "/summary_" + currentTimestamp + ".csv";
    Serial.printf("GNSS NAV-PVT time: %04d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);

    File routeFile = SPIFFS.open(currLogFile, FILE_WRITE);
    if (!routeFile) {
        Serial.println("Failed to create route session file");
    }

    routeFile.println("Latitude,Longitude,Speed(MPH),Altitude(Ft),Millis");
    routeFile.close();
    
    logPos = 0;
    logTimeBegin = millis();
    resetSessionDistance();

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

    updateSessionDistance(lat, lng, speed);

    char line[kLineMax];
    int currentLapNumber = lapNumber + 1;
    int  n = snprintf(
        line,
        sizeof(line),
        "%.7lf,%.7lf,%.2lf,%lu,%d\n",
        lat,
        lng,
        speed,
        millis() - logTimeBegin,
        currentLapNumber,
        static_cast<bool>(getAirborneState())
    );

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

    updateSessionDistance(lat, lng, speed);

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
    writeSessionSummary("lap");
    currSummaryFile = "";
    sessionActive = false;
}

void endRouteSession() {
    flushRamToFlash();
    writeSessionSummary("route");
    currSummaryFile = "";
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

double getSessionDistanceFt() {
    return sessionDistanceFt;
}
