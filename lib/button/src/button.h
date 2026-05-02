#ifndef BUTTON_H
#define BUTTON_H

namespace Button
{
    enum class Mode
    {
        NONE,
        SHORT,
        LONG,
        VERY_LONG
    };

    void InitializeButton();

    Mode PollButtonAction();
}

#endif
