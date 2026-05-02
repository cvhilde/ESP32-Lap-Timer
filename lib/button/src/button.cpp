///===========================================================================
///
/// button.cpp
///
/// TODO: Implement file description
///
///===========================================================================

#include <Arduino.h>
#include <button.h>

namespace
{
    struct ButtonPersistent
    {
        // true is pressed, false is released
        bool prev;

        bool initialized;

        // in milliseconds
        unsigned long pressStart;
    };

    constexpr uint8_t BUTTON_PIN = 46;

    ButtonPersistent _logicState;

    const unsigned long DEBOUNCE_TIME = 300;

    const unsigned long LOWER_LIMIT = 3000;

    const unsigned long UPPER_LIMIT = 6000;
}

namespace Button
{
    void InitializeButton()
    {
        pinMode(BUTTON_PIN, INPUT_PULLUP);
    }

    Mode PollButtonAction()
    {
        unsigned long buttonPressDuration(0);
        bool beginButtonLogic(false);
        Mode MODE(NONE);

        if (!_logicState.initialized)
        {
            bool currentState(digitalRead(BUTTON_PIN));
            _logicState.prev = currentState;

            if (currentState == LOW)
            {
                _logicState.pressStart = millis();
            }

            _logicState.initialized = true;

            return MODE;
        }

        bool currentState(digitalRead(BUTTON_PIN));

        if (_logicState.prev == HIGH && currentState == LOW)
        {
            _logicState.pressStart = millis();
        }

        if (_logicState.prev == LOW && currentState == HIGH)
        {
            buttonPressDuration = millis() - _logicState.pressStart;
            beginButtonLogic = true;
        }

        if (beginButtonLogic)
        {
            if (        buttonPressDuration >= DEBOUNCE_TIME
                     && buttonPressDuration <  LOWER_LIMIT)
            {
                MODE = SHORT;
            }
            else if (   buttonPressDuration >= LOWER_LIMIT
                     && buttonPressDuration <  UPPER_LIMIT)
            {
                MODE = LONG;
            }
            else if (   buttonPressDuration >= UPPER_LIMIT)
            {
                MODE = VERY_LONG;
            }
        }

        _logicState.prev = currentState;

        return MODE;
    }
}
