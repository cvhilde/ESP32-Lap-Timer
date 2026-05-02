#ifndef STORAGE_H
#define STORAGE_H

#include <button.h>
#include <gps.h>
#include <FS.h>

namespace Storage
{
    enum class SessionType
    {
        LAP_TIMING,
        ROUTE_TRACKING
    };

    const String MANIFEST_FILE = "/sessions.txt";

    const String WAYPOINTS_FILE = "/waypoints.json";

    const String LAP_LOG_PREFIX = "/log_";

    const String LAP_TIMESTAMPS_PREFIX = "/timestamps_";

    const String SUMMARY_PREFIX = "/summary_";

    const String ROUTE_LOG_PREFIX = "/route_";

    const String FILE_TYPE = ".csv";

    constexpr SessionType DEFAULT_SESSION_TYPE = SessionType::LAP_TIMING;

    bool InitializeStorage();

    void UpdateSession(const GPS::FixData& data, const Button::Mode& mode);

    void WriteWaypointsFile(const uint8_t* raw, size_t len);

    bool BackupWaypoints();

    bool LoadWaypoints();

    void LoadBackedupWaypoints();

    bool PurgeFlash();

    fs::File GetFile(const String& name, const char *mode);

    bool FileExists(const String& name);

    double StorageUsage();

    SessionType GetSessionMode();

    bool ShouldUpdateLoop();
};

#endif
