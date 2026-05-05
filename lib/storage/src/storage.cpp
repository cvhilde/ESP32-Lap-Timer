///===========================================================================
///
/// storage.cpp
///
/// This file contains waypoint storage and line-crossing logic used to track
/// sectors and lap progression from recent GPS coordinate updates.
///
///===========================================================================

#include <storage.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <led.h>
#include <vector>
#include <prefs.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    // 180 kB ≈ 18 min @ 10 kB/min
    constexpr size_t RAM_LIMIT_BYTES = 180 * 1024;

    // Longest CSV line
    constexpr size_t CSV_LINE_LIMIT = 128;

    // Number of sectors to use for all logic
    constexpr size_t NUMBER_OF_SECTORS = 3;

    // Constants for LED logic
    constexpr unsigned long END_SESSION_BLINK_INTERVAL    = 250U;
    constexpr unsigned long FAILED_SESSION_BLINK_INTERVAL = 500U;

    constexpr unsigned SESSION_TYPE_CHANGE_INTERVAL = 100U;
    constexpr unsigned SESSION_TYPE_CHANGE_LENGTH   = 500U;

    // Constant for the amount of time that must pass before another waypoint
    // crossing is detected
    constexpr unsigned long WAYPOINT_CROSSING_JITTER = 5000U;

    // Data associated with the ram buffer
    struct RamData
    {
        // Ram buffer for each session log
        char logBuffer[RAM_LIMIT_BYTES];

        // Number of bytes currently used
        size_t logPosition;

        // Time of session begin, relative in millis()
        unsigned long logTimeBegin;

        RamData() :
            logPosition(0),
            logTimeBegin(0)
        {}
    };

    // Data relating to active/previous sessions
    struct SessionInfo
    {
        // Is session active
        bool sessionActive;

        unsigned long lastUpdateTime;

        // Default to lap timing mode
        Storage::SessionType sessionType;

        // Current file name of the log file (both lap and route mode)
        String currentLogFile;

        // Current file name of the time log file
        String currentTimeLogFile;

        // Current waypoints.json track name for use in storing log
        // files. This should be updated whenever LoadWaypoints is called.
        String currentTrackName;

        // Current file name of the summary file. This is not created at session
        // start. This variable is updated at the start of a session to keep track
        // of the date/time the session was started so the file may be
        // associated with the route or lap timing session.
        String currentSummaryFile;

        // Current timestamp to be used in file names for the active session
        String currentTimeStamp;

        SessionInfo() :
            sessionActive(false),
            lastUpdateTime(0U),
            sessionType(Prefs::DEFAULT_SESSION_TYPE),
            currentLogFile(""),
            currentTimeLogFile(""),
            currentSummaryFile(""),
            currentTimeStamp("")
        {}
    };

    // Data relating to lap timing sessions
    struct LapTimingSessionInfo
    {
        // Current lap number. This is the number of laps completed
        unsigned lapNumber;

        unsigned      currentSector;
        unsigned long lastCrossTime;
        unsigned long lastLapTime;
        unsigned long currentLapTime;
        unsigned long lastSectorTime;
        unsigned long sector1Time;
        unsigned long sector2Time;
        unsigned long sector3Time;
        bool          firstLap;

        LapTimingSessionInfo() :
            lapNumber(0),
            currentSector(0),
            lastCrossTime(0U),
            lastLapTime(0U),
            currentLapTime(0U),
            lastSectorTime(0U),
            sector1Time(0U),
            sector2Time(0U),
            sector3Time(0U),
            firstLap(true)
        {}
    };

    // All data associated with writing to the session log
    RamData _ramData;

    // All data associated with the session information
    SessionInfo _sessionData;

    LapTimingSessionInfo _lapData;

    std::vector<uint8_t> _waypointsBackup;

    struct FileSystemEntry
    {
        String path;
        bool isDirectory;
    };

    //------------------------------------------------------------------------
    bool CollectFileSystemEntries(const String& directoryPath,
                                  std::vector<FileSystemEntry>& entries)
    {
        File directory = LittleFS.open(directoryPath, FILE_READ);
        if (!directory || !directory.isDirectory())
        {
            return false;
        }

        File entry = directory.openNextFile();
        while (entry)
        {
            const char* rawPath = entry.path();
            if (rawPath != nullptr && rawPath[0] != '\0')
            {
                const String entryPath(rawPath);
                const bool isDirectory(entry.isDirectory());

                if (isDirectory && !CollectFileSystemEntries(entryPath, entries))
                {
                    entry.close();
                    directory.close();
                    return false;
                }

                entries.push_back({entryPath, isDirectory});
            }

            entry.close();
            entry = directory.openNextFile();
        }

        directory.close();
        return true;
    }

    //------------------------------------------------------------------------
    bool RemoveFileSystemEntries(const std::vector<FileSystemEntry>& entries)
    {
        bool removedAll(true);

        // Remove nested files/directories first so parent directories can
        // be deleted afterwards.
        for (auto entry = entries.rbegin(); entry != entries.rend(); ++entry)
        {
            bool removed(false);

            if (entry->isDirectory)
            {
                removed = LittleFS.rmdir(entry->path);
            }
            else
            {
                removed = LittleFS.remove(entry->path);
            }

            removedAll = removedAll && removed;
        }

        return removedAll;
    }

    //------------------------------------------------------------------------
    void ResetSessionStateAfterPurge()
    {
        _ramData.logPosition = 0;
        _ramData.logTimeBegin = 0U;

        _sessionData.sessionActive = false;
        _sessionData.lastUpdateTime = 0U;
        _sessionData.currentLogFile = "";
        _sessionData.currentTimeLogFile = "";
        _sessionData.currentSummaryFile = "";
        _sessionData.currentTimeStamp = "";

        _lapData = LapTimingSessionInfo{};
        WayPoints::ResetRecentLocations();
    }


    //------------------------------------------------------------------------
    void WriteSessionSummary(Storage::SessionType sessionType)
    {
        // Summary file name was never updated. Don't create the file.
        if (_sessionData.currentSummaryFile.isEmpty())
        {
            return;
        }

        File summaryFile = LittleFS.open(_sessionData.currentSummaryFile, FILE_WRITE);
        if (!summaryFile)
        {
            // File was unable to be created. Not a fatal event, so no need
            // to signal to the user, but don't proceed with writing to the
            // empty file pointer.
            return;
        }

        const char* type;
        if (sessionType == Storage::SessionType::LAP_TIMING)
        {
            type = "lap";
        }
        else if (sessionType == Storage::SessionType::ROUTE_TRACKING)
        {
            type = "route";
        }

        const WayPoints::SessionDistance& distance(WayPoints::GetSessionDistance());

        summaryFile.println("SessionType,TotalDistanceFt,TotalDistanceMi");
        summaryFile.printf("%s,%.2lf,%.5lf\n",
                            type,
                            distance.distanceFeet,
                            distance.distanceMile
        );

        summaryFile.close();
    }

    //------------------------------------------------------------------------
    void FlushRamToFlash()
    {
        if (_ramData.logPosition == 0)
            return;

        File logFile = LittleFS.open(_sessionData.currentLogFile, FILE_APPEND);

        if (logFile)
        {
            logFile.write((uint8_t*)_ramData.logBuffer, _ramData.logPosition);
            logFile.close();
        }

        _ramData.logPosition = 0;
    }

    //------------------------------------------------------------------------
    void StartLapSession(const GPS::GPSTimeData& time)
    {
        if (time.valid)
        {
            _lapData = LapTimingSessionInfo{};
            WayPoints::ResetRecentLocations();

            char timestamp[25];
            sprintf(timestamp, "%04d%02d%02d_%02d%02d%02d",
                                time.year,
                                time.month,
                                time.day,
                                time.hour,
                                time.minute,
                                time.second
            );
            _sessionData.currentTimeStamp = String(timestamp);

            // log_YYYYMMDD_HHmmss.csv
            // log_track-name_YYYYMMDD_HHmmss.csv
            if (!_sessionData.currentTrackName.isEmpty())
            {
                _sessionData.currentLogFile
                    = Storage::LAP_LOG_PREFIX
                        + _sessionData.currentTrackName + "_"
                        + _sessionData.currentTimeStamp
                        + Storage::FILE_TYPE;
            }
            else
            {
                _sessionData.currentLogFile
                    = Storage::LAP_LOG_PREFIX
                        + _sessionData.currentTimeStamp
                        + Storage::FILE_TYPE;
            }

            _sessionData.currentTimeLogFile
                = Storage::LAP_TIMESTAMPS_PREFIX
                    + _sessionData.currentTimeStamp
                    + Storage::FILE_TYPE;

            _sessionData.currentSummaryFile
                = Storage::SUMMARY_PREFIX
                    + _sessionData.currentTimeStamp
                    + Storage::FILE_TYPE;

            File logFile  = LittleFS.open(_sessionData.currentLogFile, FILE_WRITE);
            File timeFile = LittleFS.open(_sessionData.currentTimeLogFile, FILE_WRITE);
            File manifest = LittleFS.open(Storage::MANIFEST_FILE, FILE_APPEND);
            if (!logFile || !timeFile || !manifest)
            {
                // Failed to write to manifest, meaning the session will
                // never be transferred to the app. Or the time/log file 
                // failed to be created as well. Basically, this is a
                // failed session start.

                Led::StartOneShotBlink(FAILED_SESSION_BLINK_INTERVAL, FAILED_SESSION_BLINK_INTERVAL * 4);
                return;
            }

            logFile.println("Latitude,Longitude,Speed(MPH),Millis,LapNumber");
            timeFile.println("LapNumber,Laptime,Sector1,Sector2,Sector3");

            logFile.close();
            timeFile.close();

            manifest.println(_sessionData.currentTimeStamp);
            manifest.close();

            _ramData.logPosition = 0;
            _ramData.logTimeBegin = millis();
            WayPoints::ResetSessionDistance();

            Led::TurnLedOn();

            _sessionData.sessionActive = true;
        }
    }

    //------------------------------------------------------------------------
    void WriteToLogFile(const GPS::FixData& data)
    {
        if (!_sessionData.sessionActive)
        {
            return;
        }

        WayPoints::UpdateSessionDistance(data.coord, data.speed);

        // _lapNumber is the number of completed laps. This log file needs to
        // to log what lap you are on.
        unsigned currentLapNumber = _lapData.lapNumber + 1;

        char line[CSV_LINE_LIMIT];
        int written = snprintf(line, sizeof(line), "%.7lf,%.7lf,%.2lf,%lu, %d\n",
                            data.coord.lat,
                            data.coord.lng,
                            data.speed,
                            millis() - _ramData.logTimeBegin,
                            currentLapNumber
        );

        size_t bytes = static_cast<size_t>(written);

        // Don't write it to memory if the line is corrupted.
        if (written < 0 || bytes >= sizeof(line))
        {
            return;
        }

        if (_ramData.logPosition + bytes > RAM_LIMIT_BYTES)
        {
            FlushRamToFlash();
        }

        memcpy(_ramData.logBuffer + _ramData.logPosition, line, bytes);
        _ramData.logPosition += bytes;
    }

    //------------------------------------------------------------------------
    void WriteToTimeLog()
    {
        // Don't write times if there is no session active.
        if (!_sessionData.sessionActive)
        {
            return;
        }

        // If this is being called, that means we completed a lap. Iterate it
        _lapData.lapNumber++;

        char string[CSV_LINE_LIMIT];
        sprintf(string, "%d,%lu,%lu,%lu,%lu\n",
                        _lapData.lapNumber,
                        _lapData.currentLapTime,
                        _lapData.sector1Time,
                        _lapData.sector2Time,
                        _lapData.sector3Time
        );

        File timeFile = LittleFS.open(_sessionData.currentTimeLogFile, FILE_APPEND);

        // If it doesn't open, it's not fatal. The user will just not have lap
        // times. Don't halt the user.
        if (timeFile)
        {
            timeFile.print(string);
            timeFile.close();
        }
    }

    //------------------------------------------------------------------------
    void EndLapSession()
    {
        FlushRamToFlash();
        WriteSessionSummary(Storage::SessionType::LAP_TIMING);
        _sessionData.currentSummaryFile = "";

        Led::TurnLedOff();
        Led::StartOneShotBlink(END_SESSION_BLINK_INTERVAL, END_SESSION_BLINK_INTERVAL * 4);

        _sessionData.sessionActive = false;
    }

    //------------------------------------------------------------------------
    void StartRouteSession(const GPS::GPSTimeData& time)
    {
        if (time.valid)
        {
            char timestamp[25];
            sprintf(timestamp, "%04d%02d%02d_%02d%02d%02d",
                                time.year,
                                time.month,
                                time.day,
                                time.hour,
                                time.minute,
                                time.second
            );
            _sessionData.currentTimeStamp = String(timestamp);

            _sessionData.currentLogFile
                = Storage::ROUTE_LOG_PREFIX
                    + _sessionData.currentTimeStamp
                    + Storage::FILE_TYPE;

            _sessionData.currentSummaryFile
                = Storage::SUMMARY_PREFIX
                    + _sessionData.currentTimeStamp
                    + Storage::FILE_TYPE;

            File routeFile = LittleFS.open(_sessionData.currentLogFile, FILE_WRITE);
            File manifest  = LittleFS.open(Storage::MANIFEST_FILE, FILE_APPEND);
            if (!routeFile || !manifest)
            {
                // Failed to write to manifest, meaning the session will
                // never be transferred to the app. Or the routeFile 
                // failed to be created as well. Basically, this is a
                // failed session start.

                Led::StartOneShotBlink(FAILED_SESSION_BLINK_INTERVAL, FAILED_SESSION_BLINK_INTERVAL * 4);
                return;
            }

            routeFile.println("Latitude,Longitude,Speed(MPH),Altitude(Ft),Millis");
            routeFile.close();

            manifest.println(_sessionData.currentTimeStamp);
            manifest.close();

            _ramData.logPosition = 0;
            _ramData.logTimeBegin = millis();
            WayPoints::ResetSessionDistance();

            Led::TurnLedOn();

            _sessionData.sessionActive = true;
        }
    }

    //------------------------------------------------------------------------
    void WriteToRouteLog(const GPS::FixData& data)
    {
        if (!_sessionData.sessionActive)
        {
            return;
        }

        WayPoints::UpdateSessionDistance(data.coord, data.speed);

        char line[CSV_LINE_LIMIT];
        int written = snprintf(line, sizeof(line), "%.7lf,%.7lf,%.2lf,%.2lf,%lu\n",
                            data.coord.lat,
                            data.coord.lng,
                            data.speed,
                            data.altitude,
                            millis() - _ramData.logTimeBegin
        );

        size_t bytes = static_cast<size_t>(written);

        // Don't write it to memory if the line is corrupted.
        if (written < 0 || bytes >= sizeof(line))
        {
            return;
        }

        if (_ramData.logPosition + bytes > RAM_LIMIT_BYTES)
        {
            FlushRamToFlash();
        }

        memcpy(_ramData.logBuffer + _ramData.logPosition, line, bytes);
        _ramData.logPosition += bytes;
    }

    //------------------------------------------------------------------------
    void EndRouteSession()
    {
        FlushRamToFlash();
        WriteSessionSummary(Storage::SessionType::ROUTE_TRACKING);
        _sessionData.currentSummaryFile = "";

        Led::TurnLedOff();
        Led::StartOneShotBlink(END_SESSION_BLINK_INTERVAL, END_SESSION_BLINK_INTERVAL * 4);

        _sessionData.sessionActive = false;
    }
}

//----------------------------------------------------------------------------
// Storage Public namespace
//----------------------------------------------------------------------------
namespace Storage
{
    //------------------------------------------------------------------------
    bool InitializeStorage()
    {
        bool success = LittleFS.begin(true);

        if (success)
        {
            _sessionData.sessionType = Prefs::PersistConfig().sessionType;

            if (!LoadWaypoints()) {
                // Wasn't able to load the waypoints for whatever reason.
                // This could mean a couple of different things, so just use default
                // values. This don't mean anything, but will prevent other code 
                // for not working until a waypoints file is uploaded.

                WayPoints::TrackedWaypoints defaultWaypoints;

                for (int i = 0; i < NUMBER_OF_SECTORS; i++)
                {
                    defaultWaypoints.at(i).p1.lat   =   i + 1;
                    defaultWaypoints.at(i).p1.lng   =   i + 1;
                    defaultWaypoints.at(i).p2.lat   = -(i + 1);
                    defaultWaypoints.at(i).p2.lng   = -(i + 1);
                    defaultWaypoints.at(i).isActive = true;
                }

                WayPoints::SetTrackWaypoints(defaultWaypoints);
            }
        }
        else
        {
            // Failed to mount flash. This is fatal.
            // Do nothing for now, and signal to main that we can't continue.
        }

        return success;
    }

    //------------------------------------------------------------------------
    void UpdateSessionType(const Button::Mode& mode)
    {
        // Long button press is related to switching session type.
        if (mode == Button::Mode::LONG)
        {
            switch (_sessionData.sessionType)
            {
                case Storage::SessionType::LAP_TIMING:
                    _sessionData.sessionType = Storage::SessionType::ROUTE_TRACKING;
                    break;
                case Storage::SessionType::ROUTE_TRACKING:
                    _sessionData.sessionType = Storage::SessionType::LAP_TIMING;
                    break;
                default:
                    // Invalid sessionType. Do nothing
                    break;
            }

            Led::StartOneShotBlink(SESSION_TYPE_CHANGE_INTERVAL,
                SESSION_TYPE_CHANGE_LENGTH);

            // Update the persistant session type
            Prefs::SetSessionType(_sessionData.sessionType);
        }
        else if (mode == Button::Mode::EXTRA_LONG)
        {
            _sessionData.sessionType = Prefs::DEFAULT_SESSION_TYPE;

            Led::StartOneShotBlink(SESSION_TYPE_CHANGE_INTERVAL,
                SESSION_TYPE_CHANGE_LENGTH);

            // Update the persistant session type
            Prefs::SetSessionType(_sessionData.sessionType);
        }
    }

    //------------------------------------------------------------------------
    void SessionStartStop(const GPS::FixData& data, const Button::Mode mode)
    {
        // Short button press is related to start/stopping session logic
        if (mode == Button::Mode::SHORT)
        {
            // No session is active, start a new one
            if (!_sessionData.sessionActive && data.valid)
            {
                switch (_sessionData.sessionType)
                {
                    case Storage::SessionType::LAP_TIMING:
                        StartLapSession(data.dateTime);
                        break;
                    case Storage::SessionType::ROUTE_TRACKING:
                        StartRouteSession(data.dateTime);
                        break;
                    default:
                        // Invalid sessionType. Do nothing
                        break;
                }
            }
            // Session is active, go ahead and stop it
            else if (_sessionData.sessionActive)
            {
                switch (_sessionData.sessionType)
                {
                case Storage::SessionType::LAP_TIMING:
                    EndLapSession();
                    break;
                case Storage::SessionType::ROUTE_TRACKING:
                    EndRouteSession();
                    break;
                default:
                    // Invalid sessionType. Do nothing
                    break;
                }
            }
        }
    }

    //------------------------------------------------------------------------
    void UpdateSession(const GPS::FixData& data)
    {
        WayPoints::StoreCurrentLocation(data.coord);

        // Handle updating active sessions
        if (_sessionData.sessionActive)
        {
            switch (_sessionData.sessionType)
            {
                case Storage::SessionType::LAP_TIMING:
                    WriteToLogFile(data);
                    break;
                case Storage::SessionType::ROUTE_TRACKING:
                    WriteToRouteLog(data);
                    break;
                default:
                    break;
            }
        }

        // Handle the sector waypoint crossing logic. This is only ran
        // for lap timing mode. Route tracking just logs the position.
        if (_sessionData.sessionActive && _sessionData.sessionType == Storage::SessionType::LAP_TIMING)
        {
            if (WayPoints::WaypointCrossed(_lapData.currentSector))
            {
                if (millis() - _lapData.lastCrossTime > WAYPOINT_CROSSING_JITTER)
                {
                    _lapData.lastCrossTime = millis();

                    switch (_lapData.currentSector)
                    {
                        case 0: // Start/finish line
                            if (_lapData.firstLap)
                            {
                                _lapData.lastLapTime    = millis();
                                _lapData.lastSectorTime = millis();
                                _lapData.firstLap       = false;
                            }
                            else
                            {
                                _lapData.currentLapTime = millis() - _lapData.lastLapTime;
                                _lapData.sector3Time    = millis() - _lapData.lastSectorTime;
                                _lapData.lastLapTime    = millis();
                                _lapData.lastSectorTime = millis();
                                WriteToTimeLog();
                            }
                            break;
                        case 1: // First sector crossing
                            _lapData.sector1Time    = millis() - _lapData.lastSectorTime;
                            _lapData.lastSectorTime = millis();
                            break;
                        case 2:
                            _lapData.sector2Time    = millis() - _lapData.lastSectorTime;
                            _lapData.lastSectorTime = millis();
                            break;
                        default:
                            break;
                    }

                    _lapData.currentSector++;
                    if (_lapData.currentSector > 2)
                    {
                        _lapData.currentSector = 0;
                    }
                } // end of sector logic
            }
        } //  end of lap timing logic
    }

    //------------------------------------------------------------------------
    bool BackupWaypoints()
    {
        bool exists(false);

        if (LittleFS.exists(WAYPOINTS_FILE))
        {
            File file = LittleFS.open(WAYPOINTS_FILE, FILE_READ);
            _waypointsBackup.resize(file.size());
            file.readBytes((char*)_waypointsBackup.data(), _waypointsBackup.size());
            file.close();
            exists = true;
        }

        return exists;
    }

    //------------------------------------------------------------------------
    bool LoadWaypoints()
    {
        File waypointsFile = LittleFS.open(WAYPOINTS_FILE, FILE_READ);
        if (!waypointsFile)
        {
            // This just means there is no waypoints file. Not fatal, just
            // means no file has been written yet. Use the defaults until
            // a new waypoints file is uploaded via BLE.
            return false;
        }

        WayPoints::TrackedWaypoints waypoints;

        JsonDocument doc;
        DeserializationError err(deserializeJson(doc, waypointsFile));
        waypointsFile.close();

        if (err)
        {
            // JSON parse error. This means either the JSON was corrupt,
            // written incorrectly, or some erranious error.
            return false;
        }

        JsonArray wps = doc["waypoints"].as<JsonArray>();
        if (wps.isNull() || wps.size() != NUMBER_OF_SECTORS)
        {
            // Missing the 'waypoints' array or not the correct number of
            // waypoints. This means the JSON was written incorrectly.
            return false;
        }

        for (int i = 0; i < NUMBER_OF_SECTORS; i++)
        {
            JsonObject wp = wps[i];

            if (!wp["p1"].is<JsonObject>() || !wp["p2"].is<JsonObject>())
            {
                // Missing the p1/p2 objects. This means the JSON was
                // written incorrectly.
                return false;
            }

            if (!wp["p1"]["lat"].is<float>() || !wp["p1"]["lng"].is<float>() ||
                !wp["p2"]["lat"].is<float>() || !wp["p2"]["lng"].is<float>())
            {
                // The lat/lng fields are the wrong value. This means the JSON
                // was written incorrectly.
                return false;
            }

            waypoints.at(i).p1.lat   = wp["p1"]["lat"].as<double>();
            waypoints.at(i).p1.lng   = wp["p1"]["lng"].as<double>();
            waypoints.at(i).p2.lat   = wp["p2"]["lat"].as<double>();
            waypoints.at(i).p2.lng   = wp["p2"]["lng"].as<double>();
            waypoints.at(i).isActive = wp["active"] | 1;
        }

        _sessionData.currentTrackName = doc["trackname"] | "";
        WayPoints::SetTrackWaypoints(waypoints);
        return true;
    }

    //------------------------------------------------------------------------
    void LoadBackedupWaypoints()
    {
        File file = LittleFS.open(WAYPOINTS_FILE, FILE_WRITE);
        file.write(_waypointsBackup.data(), _waypointsBackup.size());
        file.close();
        LoadWaypoints();
    }

    //------------------------------------------------------------------------
    void WriteWaypointsFile(const uint8_t* raw, size_t len)
    {
        File waypointsFile = LittleFS.open(WAYPOINTS_FILE, FILE_WRITE);

        waypointsFile.write(raw, len);
        waypointsFile.close();

        LoadWaypoints();
    }

    //------------------------------------------------------------------------
    bool PurgeFlash()
    {
        std::vector<FileSystemEntry> entries;
        if (!CollectFileSystemEntries("/", entries))
        {
            return false;
        }

        const bool purged = RemoveFileSystemEntries(entries);

        if (purged)
        {
            ResetSessionStateAfterPurge();
        }

        return purged;
    }

    //------------------------------------------------------------------------
    fs::File GetFile(const String& name, const char *mode)
    {
        File file;

        if (strcmp(mode, "r") == 0)
        {
            file = LittleFS.open(name, FILE_READ);
        }
        else if (strcmp(mode, "w") == 0)
        {
            file = LittleFS.open(name, FILE_WRITE);
        }
        else if (strcmp(mode, "a") == 0)
        {
            file = LittleFS.open(name, FILE_APPEND);
        }
        else
        {
            // Invalid mode called. Do nothing
        }

        return file;
    }

    bool FileExists(const String& name)
    {
        return LittleFS.exists(name);
    }

    //------------------------------------------------------------------------
    double StorageUsage()
    {
        size_t total = LittleFS.totalBytes();   // size of the LittleFS partition
        size_t used  = LittleFS.usedBytes();    // how much is already occupied

        if (total == 0)
        {
            return 0.0;
        }

        return (used * 100.0) / total;
    }

    //------------------------------------------------------------------------
    SessionType GetSessionMode()
    {
        return _sessionData.sessionType;
    }

    //------------------------------------------------------------------------
    bool ShouldUpdateLoop()
    {
        unsigned long updateTime;
        bool update(false);

        switch (_sessionData.sessionType)
        {
            case Storage::SessionType::LAP_TIMING:
                updateTime = 1000U / Prefs::PersistConfig().lapLogHz;
                break;
            case Storage::SessionType::ROUTE_TRACKING:
                updateTime = 1000U / Prefs::PersistConfig().routeLogHz;
                break;
            default:
                updateTime = 1000U;
                break;
        }

        if (millis() - _sessionData.lastUpdateTime >= updateTime)
        {
            _sessionData.lastUpdateTime = millis();
            update = true;
        }

        return update;
    }
}
