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

int lapNumber = 0;

bool ledFlag = true;

static unsigned long continuousLoopTime = 0;
int loopCount = 0;


void setup() {
    Serial.begin(115200);

    startBlink(250);

    initDisplay();
    firstDraw();
    initStorage();
    initGPS();

    stopBlink();
    startBlink(500);
}

void loop() {
    // update gps at 25 hz
    gps.checkUblox();
    
    // perform rest of loop at 10 hz
    if (millis() - lastUpdate >= 100) {
        lastUpdate = millis();

        if (gps.getFixType() > 1) {
            if (ledFlag) {
                stopBlink();
                ledFlag = false;
            }

            // button logic for starting and ending the session
            bool currButtonState = digitalRead(BUTTON_PIN);
            if (prevButtonState == HIGH && currButtonState == LOW && millis() - lastButtonPress > 1000 && sessionActive == false) {
                lastButtonPress = millis();
                turnLEDOn();
                startSession();
            } else if (prevButtonState == HIGH && currButtonState == LOW && millis() - lastButtonPress > 1000 && sessionActive == true) {
                sessionActive = false;
                lapNumber = 0;
                turnLEDOff();
                Serial.println("Ending session");
            }
            prevButtonState = currButtonState;

            // update current location
            double lat = getLatitude();
            double lng = getLongitude();
            storeCurrLocation(lat, lng);

            // log the data
            if (sessionActive == true) {
                writeToLogFile(lat, lng, getSpeed());
            }

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
            drawLaptime(millis() - lastLapTime, sector1Time, sector2Time, sector3Time, previousLapTime);   

        } else {
            if (!ledFlag) {
                startBlink(500);
                ledFlag = true;
            }
            display.setColor(BLACK);
            display.fillRect(40, 5, 64, 14);
            display.setColor(WHITE);
            display.drawString(40, 5, "No Fix");
            display.display();
        }

        

        // Serial.println(millis() - loopTime);
        // loopTime = millis();

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