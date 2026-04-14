///===========================================================================
///
/// led.cpp
///
/// Implements LED control helpers for steady, blinking, and one-shot blink modes.
///
///===========================================================================

#include <led.h>
#include <Arduino.h>
#include <Ticker.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    // LED ticker instance
    Ticker _led;

    // LED ticker instance for single instance (one shot)
    Ticker _ledOneShotStop;

    // Persistent variable for if another function is trying to start a blink
    bool _blinkActive(false);

    // Current state of the LED (on or off)
    bool _ledBlinkState(false);

    // Blink on duration in milliseconds. Inital value here does not matter
    uint32_t _blinkOnDurationMs = 500;

    // Blink off duration in milliseconds. Inital value here does not matter
    uint32_t _blinkOffDurationMs = 500;

    // Physical LED pin on the ESP32
    constexpr uint8_t LED_PIN = 36;

    //------------------------------------------------------------------------
    // Event bus for blinking the LED using Ticker.
    void IRAM_ATTR OnBlink()
    {
        if (!_blinkActive)
        {
            return;
        }

        _ledBlinkState = !_ledBlinkState;
        digitalWrite(LED_PIN, _ledBlinkState ? HIGH : LOW);

        uint32_t nextDuration = _ledBlinkState ? _blinkOnDurationMs : _blinkOffDurationMs;
        _led.once_ms(nextDuration, OnBlink);
    }

    //------------------------------------------------------------------------
    // Similar even bus as OnBlink(), but instead after the first 'tick', calls
    // StopBlink() to stop the blinking event.
    void IRAM_ATTR OnOneShotBlinkComplete()
    {
        Led::StopBlink();
    }
}

//----------------------------------------------------------------------------
// Led Public namespace
//----------------------------------------------------------------------------
namespace Led
{
    //------------------------------------------------------------------------
    void StartBlink(unsigned long interval)
    {
        StartBlink(interval, interval);
    }

    //------------------------------------------------------------------------
    void StartBlink(unsigned long onDuration, unsigned long offDuration)
    {
        pinMode(LED_PIN, OUTPUT);
        _led.detach();
        _ledOneShotStop.detach();
        _blinkActive = true;
        _blinkOnDurationMs = onDuration > 0 ? onDuration : 1;
        _blinkOffDurationMs = offDuration > 0 ? offDuration : 1;
        _ledBlinkState = true;
        digitalWrite(LED_PIN, HIGH);

        _led.once_ms(_blinkOnDurationMs, OnBlink);
    }

    //------------------------------------------------------------------------
    void StartOneShotBlink(unsigned long interval, unsigned long duration)
    {
        StartBlink(interval, interval);
        _ledOneShotStop.once_ms(duration > 0 ? duration : 1, OnOneShotBlinkComplete);
    }

    //------------------------------------------------------------------------
    void StopBlink()
    {
        _blinkActive = false;
        _led.detach();
        _ledOneShotStop.detach();
        _ledBlinkState = false;
        digitalWrite(LED_PIN, LOW);
    }

    //------------------------------------------------------------------------
    void TurnLedOn()
    {
        digitalWrite(LED_PIN, HIGH);
    }

    //------------------------------------------------------------------------
    void TurnLedOff()
    {
        digitalWrite(LED_PIN, LOW);
    }
}
