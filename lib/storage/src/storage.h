#ifndef STORAGE_H
#define STORAGE_H

#include <FS.h>

// Forward Declarations
namespace Button
{
    enum class Mode;
}

namespace GPS
{
    struct FixData;
}

namespace Storage
{
    enum class SessionType
    {
        LAP_TIMING,
        ROUTE_TRACKING
    };

    // Constants relating to the storage prefixes for sessions
    const String MANIFEST_FILE = "/sessions.txt";
    const String WAYPOINTS_FILE = "/waypoints.json";
    const String LAP_LOG_PREFIX = "/log_";
    const String LAP_TIMESTAMPS_PREFIX = "/timestamps_";
    const String SUMMARY_PREFIX = "/summary_";
    const String ROUTE_LOG_PREFIX = "/route_";
    const String DRAG_ROUTE_PREFIX = "/drag_route_";
    const String DRAG_EVENTS_PREFIX = "/drag_events_";
    const String DRAG_CONFIG_PREFIX = "/drag_config_";
    const String FILE_TYPE = ".csv";
    const String JSON_FILE_TYPE = ".json";

    // Initialize the storage and LittleFS partition
    bool InitializeStorage();

    // Determines when to start/stop sessions.
    void SessionStartStop(const GPS::FixData& data, const Button::Mode mode);

    // Updates the session logic, giving new information to the current
    // session. It will also perform the session sector crossing logic.
    void UpdateSession(const GPS::FixData& data);

    // Updates the current session type outside of the UpdateSession logic.
    // This allows session type to be changed even when there is no fix
    // available from the GPS.
    void UpdateSessionType(const Button::Mode& mode);

    // Backs up the waypoints file if it exists. This is so it doesn't get lost
    // when the purge is performed.
    bool BackupWaypoints();

    // Loads the waypoints from the WAYPOINTS_FILE, if it exists
    bool LoadWaypoints();

    // Writes back the waypoints to the LittleFS flash after a purge is
    // completed. Calls LoadWaypoints afterwards to ensure ram
    // waypoints are fresh.
    void LoadBackedupWaypoints();

    // Write a new waypoints file that has been received from BLE
    void WriteWaypointsFile(const uint8_t* raw, size_t len);

    // Purges the flash of all files. This WILL erase the waypoints
    // file if not saved before called.
    bool PurgeFlash();

    // Returns the fs::File pointer to the specified file.
    // mode should be the following:
    // r - FILE_READ
    // w - FILE_WRITE
    // a - FILE_APPEND
    fs::File GetFile(const String& name, const char *mode);

    // Returns a boolean for if a file exists or not.
    bool FileExists(const String& name);

    // Returns the LittleFS partition usage in a percentage.
    double StorageUsage();

    // Returns the current session Mode
    SessionType GetSessionMode();

    // Returns true when a lap or route storage session is active.
    bool IsSessionActive();

    // Returns the current track name
    String GetTrackName();
};

#endif
