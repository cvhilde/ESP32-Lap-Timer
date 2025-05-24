#include <display.h>
#include <globals.h>
#include <gps.h>
#include <waypoints.h>
#include <storage.h>


coord activeLocations[2]; // 0 is current, 1 is previous
wayPoint trackWaypoints[3]; // first is start/finish line, next 2 are the sector waypoints
uint8_t currSector = 0;

bool sessionActive = false;
String currentTimestamp = "";

static unsigned long loopTime = 0; 
static unsigned long lastUpdate = 0;
static unsigned long lastCrossTime = 0;
static unsigned long lastSectorTime = 0;
unsigned long lastLapTime = 0;
static unsigned long lastButtonPress = 0;
static unsigned long lastDraw = 0;
static bool prevButtonState = HIGH;
static unsigned long previousLapTime = 0;
static unsigned long sector1Time = 0;
static unsigned long sector2Time = 0;
static unsigned long sector3Time = 0;
static bool firstLap = true;


void setup() {
    Serial.begin(115200);

    initGPS();
    initStorage();
    initDisplay();
}

void loop() {
    gps.checkUblox();
    if (millis() - lastUpdate >= 100) {
        lastUpdate = millis();
        if (gps.getFixType() > 1) {
            bool currButtonState = digitalRead(BUTTON_PIN);
            if (prevButtonState == HIGH && currButtonState == LOW && millis() - lastButtonPress > 1000 && sessionActive == false) {
                lastButtonPress = millis();
                startSession();
            } else if (prevButtonState == HIGH && currButtonState == LOW && millis() - lastButtonPress > 1000 && sessionActive == true) {
                sessionActive = false;
                //endSession() logic;
                Serial.println("Ending session");
            }
            prevButtonState = currButtonState;

            double lat = getLatitude();
            double lng = getLongitude();
            storeCurrLocation(lat, lng);

            if (sessionActive == true) {
                // log data
            }

            double distance = distanceCalc(activeLocations[0], trackWaypoints[currSector]);
            if (trackWaypoints[currSector].isActive) {
                if (doIntersect(activeLocations[1], activeLocations[0], trackWaypoints[currSector])) {
                    if (millis() - lastCrossTime > 15000) {
                        lastCrossTime = millis();
                        if (currSector == 0) {
                            if (firstLap) {
                                lastLapTime = millis();
                                lastSectorTime = millis();
                                firstLap = false;
                            } else if (!firstLap) {
                                previousLapTime = millis() - lastLapTime;
                                lastLapTime = millis();
                                sector3Time = millis() - lastSectorTime;
                                lastSectorTime = millis();
                            }
                        } else {
                            if (currSector == 1) {
                                sector1Time = millis() - lastSectorTime;
                                lastSectorTime = millis();
                            } else if (currSector == 2) {
                                sector2Time = millis() - lastSectorTime;
                                lastSectorTime = millis();
                            }
                        }
                        currSector++;
                        if (currSector > 2) {
                            currSector = 0;
                        }
                    }
                }
            }
            drawLaptime(millis() - lastLapTime, sector1Time, sector2Time, sector3Time, previousLapTime);   
            
        } else {
            uint8_t sats = getSatCount();
            Serial.printf("sats: %d\n", sats);
            char satInfo[30];
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(display.width()/2, display.height()/2 - 8, "Waiting for GPS...");
            sprintf(satInfo, "Sats: %d", getSatCount());
            display.drawString(display.width()/2, display.height()/2 - 16, satInfo);
            display.display();
        }
    }
    if (millis() - loopTime > 10) {
        Serial.println(millis() - loopTime);
    }
    loopTime = millis();

}