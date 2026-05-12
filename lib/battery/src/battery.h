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

    // Reads the battery voltage and stores it in the rolling average.
    float ReadVoltage();

    // Returns the average battery percentage based on the rolling
    // average of the read in battery voltages.
    int AveragePercentage();
};

#endif
