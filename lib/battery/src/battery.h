#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

namespace Battery
{
    // Initialize the battery read pins.
    void InitializeBattery();

    // Returns a boolean of whether or not a battery is connected.
    // This is a user set value, not detected.
    bool IsConnected();

    // Reads the battery voltage.
    // WARNING: This will incur a small delay of 2 * BATTERY_READ_SAMPLES ms.
    // Do not call this often, otherwise it can delay the main loop.
    float ReadVoltage();

    // Returns a battery percentage based on the passed in battery voltage.
    int Percentage(float voltage);
};

#endif
