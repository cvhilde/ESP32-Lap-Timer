#ifndef STORAGE_H
#define STORAGE_H

#include <button.h>
#include <gps.h>

namespace Storage
{
    enum SessionType
    {
        LAP_TIMING,
        ROUTE_TRACKING
    };

    bool InitializeStorage();

    void UpdateSession(const GPS::FixData& data, const Button::Mode& mode);

    void WriteWaypointsFile(const uint8_t* raw, size_t len);

    bool LoadWaypoints();

    double StorageUsage();

    const SessionType GetSessionMode();

    bool ShouldUpdateLoop();
};

#endif
