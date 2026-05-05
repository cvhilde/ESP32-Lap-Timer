///===========================================================================
///
/// prefs.cpp
///
/// This file contains logic for interfacing with the NVS partition, allowing
/// easy storage of configuration values that won't be erased from flash
/// during a purge event.
///
///===========================================================================

#include <prefs.h>
#include <Preferences.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    constexpr const char* PREF_NAMESPACE   = "config";
    constexpr const char* SESSION_TYPE_KEY = "session";
    constexpr const char* LAP_FREQ_KEY     = "lapHz";
    constexpr const char* ROUTE_FREQ_KEY   = "routeHz";

    Preferences _prefs;
    Prefs::Config _persistantConfig;

    //------------------------------------------------------------------------
    bool IsValidSessionType(Storage::SessionType type)
    {
        return type == Storage::SessionType::LAP_TIMING ||
               type == Storage::SessionType::ROUTE_TRACKING;
    }
}

//----------------------------------------------------------------------------
// Public Prefs namespace
//----------------------------------------------------------------------------
namespace Prefs
{
    //------------------------------------------------------------------------
    bool InitializePrefs()
    {
        // Open read/write so the namespace is created on a freshly flashed ESP32.
        bool success = _prefs.begin(PREF_NAMESPACE, false);

        if (success)
        {
            uint8_t sessionTypeValue = _prefs.getUChar(
                SESSION_TYPE_KEY,
                static_cast<uint8_t>(DEFAULT_SESSION_TYPE)
            );

            Storage::SessionType sessionType =
                static_cast<Storage::SessionType>(sessionTypeValue);

            if (IsValidSessionType(sessionType))
            {
                _persistantConfig.sessionType = sessionType;
            }
            else
            {
                _persistantConfig.sessionType = DEFAULT_SESSION_TYPE;
            }

            _persistantConfig.lapLogHz =
                _prefs.getUInt(LAP_FREQ_KEY, DEFAULT_LAP_FREQUENCY);

            _persistantConfig.routeLogHz =
                _prefs.getUInt(ROUTE_FREQ_KEY, DEFAULT_ROUTE_FREQUENCY);

            _prefs.end();
        }

        return success;
    }

    //------------------------------------------------------------------------
    const Config& PersistConfig()
    {
        return _persistantConfig;
    }

    //------------------------------------------------------------------------
    void SetSessionType(Storage::SessionType type)
    {
        if (!IsValidSessionType(type) || _persistantConfig.sessionType == type)
        {
            return;
        }

        if (!_prefs.begin(PREF_NAMESPACE, false))
        {
            return;
        }

        _persistantConfig.sessionType = type;

        _prefs.putUChar(SESSION_TYPE_KEY, static_cast<uint8_t>(type));
        _prefs.end();

    }

    //------------------------------------------------------------------------
    void SetLapLogFrequency(unsigned hz)
    {
        unsigned hzToStore(constrain(hz, MIN_FREQUENCY, MAX_FREQUENCY));

        if (hzToStore == _persistantConfig.lapLogHz)
        {
            return;
        }

        if (!_prefs.begin(PREF_NAMESPACE, false))
        {
            return;
        }

        _persistantConfig.lapLogHz = hzToStore;

        _prefs.putUInt(LAP_FREQ_KEY, hzToStore);
        _prefs.end();
    }

    //------------------------------------------------------------------------
    void SetRouteLogFrequency(unsigned hz)
    {
        unsigned hzToStore(constrain(hz, MIN_FREQUENCY, MAX_FREQUENCY));

        if (hzToStore == _persistantConfig.routeLogHz)
        {
            return;
        }

        if (!_prefs.begin(PREF_NAMESPACE, false))
        {
            return;
        }

        _persistantConfig.routeLogHz = hzToStore;

        _prefs.putUInt(ROUTE_FREQ_KEY, hzToStore);
        _prefs.end();
    }
}
