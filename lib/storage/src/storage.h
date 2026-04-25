#ifndef STORAGE_H
#define STORAGE_H

#include <button.h>

namespace Storage
{
    bool Initialize();

    void UpdateLogging(const GPS::FixData& data, Button::Mode& action);

    void WriteWaypointsFile(const uint8_t* raw, size_t len);

    bool LoadWaypoints();

    double Usage();
};

#include <Arduino.h>

extern String waypointsFile;

#endif
