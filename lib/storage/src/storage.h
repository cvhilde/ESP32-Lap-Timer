#ifndef STORAGE_H
#define STORAGE_H

#include <button.h>

namespace Storage
{
    bool InitializeStorage();

    void UpdateLapTiming(const GPS::FixData& data, Button::ButtonAction& action);

    void UpdateRouteTracking(const GPS::FixData& data, Button::ButtonAction& action);

    void WriteWaypointsFile(const uint8_t* raw, size_t len);

    bool LoadWaypoints();
};

#include <Arduino.h>

extern String waypointsFile;

#endif
