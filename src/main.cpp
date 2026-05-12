#include <display.h>
#include <gps.h>
#include <storage.h>
#include <ble.h>
#include <button.h>
#include <led.h>
#include <prefs.h>
#include <battery.h>

bool ledFlag = false;
unsigned long lastUpdateTime = 0;
unsigned long lastGPSTime = 0;

void setup() {
    Serial.begin(115200);
    Button::InitializeButton();
    Led::InitializeLed();
    Battery::InitializeBattery();

    // Start the blinking to signal the device is booting up.
    Led::StartBlink(250U);

    // Get the start of initialization.
    unsigned long splashStart = millis();

    bool displayOk = Display::InitializeDisplay();
    bool prefsOk   = Prefs::InitializePrefs();
    bool storageOk = Storage::InitializeStorage();
    bool gpsOk     = GPS::InitializeUBLOX();
    BLE::InitializeBLE();

    bool initialized = displayOk && prefsOk && storageOk && gpsOk;

    // Display the splashscreen for atleast 5 seconds.
    while (millis() - splashStart <= Display::SPLASH_MINIMUM_TIME)
    {
        delay(10);
    }
    
    // Determine whether to display the failed start screen.
    Display::DetermineSplashScreen(initialized);

    if (!initialized)
    {
        // Only start the blink once.
        Led::StartBlink(100U);

        while(!initialized)
        {
            // Do nothing. ESP32 failed to start.
        }
    }

    // System is green. Continue with logic.
    Led::StopBlink();
}

void loop() {
    // Grab the latest gps data always.
    GPS::UpdateUBLOX();

    double freqGPS = 1000.0 / (millis() - lastGPSTime);
    lastGPSTime = millis();
    Serial.printf("GPS Frequency: %.2lf\n", freqGPS);

    // Grab the button mode.
    const Button::Mode mode(Button::PollButtonAction());

    // Grab latest ESP32 state
    Display::StatusSnapshot status
    {
        GPS::GetFixData(),
        Storage::GetSessionMode(),
        Storage::StorageUsage(),
        BLE::GetStatus()
    };

    // Screen updates can be done even if fix data is not valid.
    Display::UpdateScreen(status);

    // Update BLE. Still allow pairing and other BLE logic even with
    // no fix.
    BLE::UpdateBLE(mode);

    // Update the session type
    Storage::UpdateSessionType(mode);

    // Update the session start/stopping
    Storage::SessionStartStop(status.fixData, mode);

    // Perform rest of loop at set refresh rate.
    if (Storage::ShouldUpdateLoop()) {
        // Only update session logic when there is atleast a 2D fix.
        if (status.fixData.valid) {

            // Only stop the blink once, to avoid stopping other blinks.
            if (ledFlag) {
                Led::StopBlink();
                ledFlag = false;
            }

            double freqSession = 1000.0 / (millis() - lastUpdateTime);
            lastUpdateTime = millis();
            Serial.printf("Frequency: %.2lf\n", freqSession);

            // Update session logic
            Storage::UpdateSession(status.fixData);

        // No valid fx. Only start the blink once.
        } else {
            if (!ledFlag) {
                Led::StartBlink(500);
                ledFlag = true;
            }
        }
    }
}
