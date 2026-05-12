///===========================================================================
///
/// button.cpp
///
/// Button input initialization and press-duration classification.
///
///===========================================================================

#include <Arduino.h>
#include <button.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
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

    // Physical button pin. For the V3.2
    constexpr uint8_t BUTTON_PIN = 45;

    // Constants relating to the press times to determine the Mode
    constexpr unsigned DEBOUNCE_TIME      = 100U;
    constexpr unsigned LOWER_LIMIT        = 3000U;
    constexpr unsigned UPPER_LIMIT        = 6000U;
    constexpr unsigned RESET_SESSION_TYPE = 10000U;

    // Persistent datastore for button logic
    ButtonPersistent _logicState;
}

//----------------------------------------------------------------------------
// Public namespace
//----------------------------------------------------------------------------
namespace Button
{
    //------------------------------------------------------------------------
    void InitializeButton()
    {
        pinMode(BUTTON_PIN, INPUT_PULLUP);
    }

    //------------------------------------------------------------------------
    Mode PollButtonAction()
    {
        unsigned long buttonPressDuration(0);
        bool beginButtonLogic(false);
        Mode MODE(Mode::NONE);

        // The button could be held down before boot up.
        // Therefore, grab the first millis() reading without checking
        // the previous state.
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
                MODE = Mode::SHORT;
            }
            else if (   buttonPressDuration >= LOWER_LIMIT
                     && buttonPressDuration <  UPPER_LIMIT)
            {
                MODE = Mode::LONG;
            }
            else if (   buttonPressDuration >= UPPER_LIMIT
                     && buttonPressDuration <  RESET_SESSION_TYPE)
            {
                MODE = Mode::VERY_LONG;
            }
            else if (   buttonPressDuration >= RESET_SESSION_TYPE)
            {
                MODE = Mode::EXTRA_LONG;
            }
        }

        _logicState.prev = currentState;

        return MODE;
    }
}
