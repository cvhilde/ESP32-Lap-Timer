#ifndef PREFS_H
#define PREFS_H

#include <storage.h>

namespace Prefs
{
    // Default session type when starting the device if no preference is present
    constexpr Storage::SessionType DEFAULT_SESSION_TYPE = Storage::SessionType::LAP_TIMING;

    constexpr unsigned DEFAULT_LAP_FREQUENCY = 10U;
    constexpr unsigned DEFAULT_ROUTE_FREQUENCY = 5U;

    struct Config
    {
        Storage::SessionType sessionType;

        Config():
            sessionType(Prefs::DEFAULT_SESSION_TYPE)
        {}
    };

    void InitializePrefs();

    const Config& GetConfig();

    void SetSessionType(Storage::SessionType type);
};

#endif
