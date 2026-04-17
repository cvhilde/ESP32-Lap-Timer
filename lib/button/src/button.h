#ifndef BUTTON_H
#define BUTTON_H

namespace Button
{
    enum Mode
    {
        NONE,
        SHORT,
        LONG,
        VERY_LONG
    };

    const Mode& PollButtonAction();
}

#endif