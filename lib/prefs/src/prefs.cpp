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
    constexpr const char* PREF_NAMESPACE = "config";
    constexpr const char* SESSION_TYPE_KEY = "session";

    Preferences _prefs;
    Prefs::Config _config;

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
    void InitializePrefs()
    {
        if (!_prefs.begin(PREF_NAMESPACE, true))
        {
            return;
        }

        uint8_t sessionTypeValue = _prefs.getUChar(
            SESSION_TYPE_KEY,
            static_cast<uint8_t>(DEFAULT_SESSION_TYPE)
        );

        Storage::SessionType sessionType =
            static_cast<Storage::SessionType>(sessionTypeValue);

        if (IsValidSessionType(sessionType))
        {
            _config.sessionType = sessionType;
        }
        else
        {
            _config.sessionType = DEFAULT_SESSION_TYPE;
        }

        _prefs.end();
    }

    //------------------------------------------------------------------------
    const Config& GetConfig()
    {
        return _config;
    }

    //------------------------------------------------------------------------
    void SetSessionType(Storage::SessionType type)
    {
        if (!IsValidSessionType(type) || _config.sessionType == type)
        {
            return;
        }

        _config.sessionType = type;

        if (!_prefs.begin(PREF_NAMESPACE, false))
        {
            return;
        }

        _prefs.putUChar(SESSION_TYPE_KEY, static_cast<uint8_t>(type));
        _prefs.end();

    }
}
