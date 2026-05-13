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

    // Reads the battery voltage and returns it.
    float ReadInstantVoltage();

    // Returns the average battery percentage based on the rolling
    // average of the read in battery voltages.
    int AveragePercentage();

    // Should be called continuously in the main loop to provide an
    // accurate battery reading whenever needed.
    void UpdateBatteryPercentage();
};

#endif
