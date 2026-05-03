#ifndef PREFS_H
#define PREFS_H

#include <storage.h>

namespace Prefs
{
    // Default session type when starting the device if no preference is present
    constexpr Storage::SessionType DEFAULT_SESSION_TYPE = Storage::SessionType::LAP_TIMING;

    constexpr unsigned DEFAULT_LAP_FREQUENCY = 10U;
    constexpr unsigned DEFAULT_ROUTE_FREQUENCY = 5U;

    constexpr unsigned MAX_FREQUENCY = 25U;
    constexpr unsigned MIN_FREQUENCY = 1U;

    struct Config
    {
        Storage::SessionType sessionType;
        unsigned lapLogHz;
        unsigned routeLogHz;

        Config():
            sessionType(Prefs::DEFAULT_SESSION_TYPE),
            lapLogHz(DEFAULT_LAP_FREQUENCY),
            routeLogHz(DEFAULT_ROUTE_FREQUENCY)
        {}
    };

    void InitializePrefs();

    const Config& PersistConfig();

    void SetSessionType(Storage::SessionType type);

    void SetLapLogFrequency(unsigned hz);

    void SetRouteLogFrequency(unsigned hz);
};

#endif
