#ifndef BUTTON_H
#define BUTTON_H

namespace Button
{
    // Types of button presses. Uses the constants in buttons.cpp
    // to determine how long button presses must be for each
    enum class Mode
    {
        NONE,
        SHORT,
        LONG,
        VERY_LONG
    };

    // Initalize Button Pin
    void InitializeButton();

    // Poll the current button action. A mode is only returned
    // when the button is released.
    Mode PollButtonAction();
}

#endif
