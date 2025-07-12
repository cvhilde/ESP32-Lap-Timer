#include <display.h>
#include <globals.h>
#include <gps.h>
#include <waypoints.h>
#include <storage.h>
#include <ble.h>

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
static bool prevButtonState = HIGH;
static unsigned long previousLapTime = 0;
static unsigned long sector1Time = 0;
static unsigned long sector2Time = 0;
static unsigned long sector3Time = 0;
static bool firstLap = true;
int lapNumber = 0;
bool ledFlag = true;
int loopCount = 0;
double lat = 0.0;
double lng = 0.0;
bool currButtonState = false;
unsigned long startButtonPressDurr = 0;
unsigned long buttonPressDurr = 0;
bool beginButtonLogic = false;
bool advertising = false;
bool beginAdvLightBlink = false;
unsigned long advLightStartTime = 0;
bool advFailLogic = false;
unsigned long advFailedLightStart = 0;
bool advFailLight = false;

static unsigned long continuousLoopTime = 0;
unsigned long lastStatusDraw = 0;

int loopFrequency = 100;
bool isRouteTracking = false;


void setup() {
    Serial.begin(115200);
    startBlink(250);

    initDisplay();
    initStorage();
    initGPS();
    initBLE();

    stopBlink();
    startBlink(500);
    display.clear();
    permDraws();
}

void loop() {
    // update gps at 25 hz
    gps.checkUblox();
    BLE_loop();

    if (millis() - lastStatusDraw > 1000) {
        drawStatusScreen();
        drawCurrentMode(isRouteTracking);
        lastStatusDraw = millis();
    }
    
    // perform rest of loop at 10 hz or 5 hz
    if (millis() - lastUpdate >= loopFrequency) {
        lastUpdate = millis();

        if (gps.getFixType() > 1) {
            if (ledFlag) {
                stopBlink();
                ledFlag = false;
            }

            // button logic for counting button press duration
            currButtonState = digitalRead(BUTTON_PIN);
            if (prevButtonState == HIGH && currButtonState == LOW) {
                startButtonPressDurr = millis();
            }

            // once released, tally total and update begin variable
            if (prevButtonState == LOW && currButtonState == HIGH) {
                buttonPressDurr = millis() - startButtonPressDurr;
                beginButtonLogic = true;
            }
            prevButtonState = currButtonState;

            // once released, determine what button logic to procede with
            if (beginButtonLogic) {
                if (buttonPressDurr < 3000) {
                    if (!sessionActive) {
                        Serial.println("Beginning session");
                        if (!isRouteTracking) {
                            turnLEDOn();
                            startSession();
                        } else {
                            // route tracking session start
                            turnLEDOn();
                            startRouteSession();
                        }
                    } else if (sessionActive) {
                        Serial.println("Ending session");
                        if (!isRouteTracking) {
                            lapNumber = 0;
                            turnLEDOff();
                            endSession();
                        } else {
                            turnLEDOff();
                            endRouteSession();
                            // route tracking session end
                        }
                    }
                } else if (buttonPressDurr >= 3000 && buttonPressDurr < 6000) {
                    Serial.println("Long press detected. Starting BLE advertising");
                    startAdvertising();
                    advertising = true;
                } else if (buttonPressDurr >= 6000) {
                    Serial.println("Switching modes");
                    // switch between lap timing and route tracking
                    if (!isRouteTracking) {
                        loopFrequency = 200;
                        isRouteTracking = true;
                    } else if (isRouteTracking) {
                        loopFrequency = 100;
                        isRouteTracking = false;
                    }
                }
                beginButtonLogic = false;
            }


            // BLE led status logic
            if (advertising) {
                if (!beginAdvLightBlink) {
                    Serial.println("Beginning advertising");
                    startBlink(1000);
                    advLightStartTime = millis();
                    beginAdvLightBlink = true;
                } else if (millis() - advLightStartTime >= 60000) {
                    Serial.println("BLE pairing timeout");
                    stopBlink();
                    beginAdvLightBlink = false;
                    stopAdvertising();
                    advertising = false;
                    advFailLogic = true;
                    advFailedLightStart = millis();
                } else if (isConnected()) {
                    advertising = false;
                    beginAdvLightBlink = false;
                    advFailLogic = false;
                    Serial.println("BLE Connected");
                    stopBlink();
                }
            }

            if (!advertising && !isConnected() && advFailLogic) {
                if (!advFailLight) {
                    Serial.println("Beginning LED blinking");
                    advFailLight = true;
                    startBlink(200);
                } else if (millis() - advFailedLightStart >= 3000) {
                    stopBlink();
                    advFailLogic = false;
                    Serial.println("BLE failed LED blink stopping");
                }
            }

            // update current location
            lat = getLatitude();
            lng = getLongitude();
            storeCurrLocation(lat, lng);

            // log the data
            if (sessionActive == true) {
                if (!isRouteTracking) {
                    writeToLogFile(lat, lng, getSpeed());
                } else if (isRouteTracking) {
                    // write to route log
                    writeToRouteLog(lat, lng, getSpeed(), getAltitude());
                }
            }

            // disable waypoint crossing if in route tracking mode
            if (!isRouteTracking) {
                // only check if the waypoint is active (actually there)
                if (trackWaypoints[currSector].isActive) {
                    // determine if the line was crossed
                    if (doIntersect(activeLocations[1], activeLocations[0], trackWaypoints[currSector])) {
                        // jitter prevention
                        if (millis() - lastCrossTime > 5000) {
                            lastCrossTime = millis();
                            // 0 is the start/finish line
                            if (currSector == 0) {
                                // if its the first lap, dont update any other sector times
                                if (firstLap) {
                                    lastLapTime = millis();
                                    lastSectorTime = millis();
                                    firstLap = false;
                                } else if (!firstLap) {
                                    previousLapTime = millis() - lastLapTime;
                                    lastLapTime = millis();
                                    sector3Time = millis() - lastSectorTime;
                                    lastSectorTime = millis();
                                    lapNumber++;
                                    writeToTimeLog(previousLapTime, sector1Time, sector2Time, sector3Time);
                                }   
                            } else {
                                // sector 1 and 2 logic
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
                        } // end of sector logic
                    }
                } // end of waypoint logic
            }

        } else {
            if (!ledFlag) {
                startBlink(500);
                ledFlag = true;
            }
        }

        // this is just for testing the loop running frequency to make sure everything is running properly
        loopCount++;
        if (millis() - continuousLoopTime >= 10000) {
            continuousLoopTime = millis();
            float clockRate = loopCount / 10.0;
            Serial.printf("Loop Frequency: %.2f\n", clockRate);
            loopCount = 0;
        }

    } // end of 10 hz loop

}
