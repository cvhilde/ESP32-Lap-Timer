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
    const Mode& PollButtonAction()
    {
        unsigned long buttonPressDuration(0);
        bool beginButtonLogic(false);
        bool currentState(digitalRead(BUTTON_PIN));
        Mode MODE(NONE);

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

        return MODE;
    }
}