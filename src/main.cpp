#include <display.h>
#include <globals.h>
#include <gps.h>
#include <waypoints.h>
#include <storage.h>
#include <ble.h>
#include <button.h>
#include <led.h>

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
unsigned long lastStatusDraw = 0;

int loopFrequency = 100;
bool isRouteTracking = false;


void setup() {
    Serial.begin(115200);

    Led::StartBlink(250U);
    Display::InitializeDisplay();
    Storage::InitializeStorage();
    GPS::InitializeUBLOX();
    initBLE();

    Led::StopBlink();
}

void loop() {
    // update gps at 25 hz
    GPS::UpdateUBLOX();

    // Grab latest fixData
    const GPS::FixData& fixData(GPS::GetFixData());

    // Screen updates can be done even if fix data is not valid.
    Display::UpdateScreen(fixData);
    
    // perform rest of loop at 10 hz or 5 hz
    if (Storage::ShouldUpdateLoop()) {

        if (fixData.fixType > 1) {
            if (ledFlag) {
                Led::StopBlink();
                ledFlag = false;
            }

            // Grab the button mode.
            const Button::Mode& mode(Button::PollButtonAction());

            // Update session logic
            Storage::UpdateSession(fixData, mode);


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

        } else {
            if (!ledFlag) {
                Led::StartBlink(500);
                ledFlag = true;
            }
        }

    } // end of 10 hz loop

}
