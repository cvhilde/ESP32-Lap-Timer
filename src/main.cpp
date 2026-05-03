#include <display.h>
#include <gps.h>
#include <storage.h>
#include <ble.h>
#include <button.h>
#include <led.h>
#include <prefs.h>

bool ledFlag = false;

void setup() {
    Serial.begin(115200);
    Button::InitializeButton();
    Led::InitializeLed();

    Led::StartBlink(250U);
    Prefs::InitializePrefs();
    Display::InitializeDisplay();
    Storage::InitializeStorage();
    GPS::InitializeUBLOX();
    BLE::InitializeBLE();

    Led::StopBlink();
}

void loop() {
    // Grab the latest gps data always.
    GPS::UpdateUBLOX();

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

    // Perform rest of loop at set refresh rate.
    if (Storage::ShouldUpdateLoop()) {
        // Only update session logic when there is atleast a 2D fix.
        if (status.fixData.valid) {

            // Only stop the blink once, to avoid stopping other blinks.
            if (ledFlag) {
                Led::StopBlink();
                ledFlag = false;
            }

            // Update session logic
            Storage::UpdateSession(status.fixData, mode);

        // No valid fx. Only start the blink once.
        } else {
            if (!ledFlag) {
                Led::StartBlink(500);
                ledFlag = true;
            }
        }
    }
}
