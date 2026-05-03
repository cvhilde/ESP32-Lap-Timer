#ifndef BLE_H
#define BLE_H

#include <button.h>

namespace BLE
{
    // BLE status information with constructors
    struct Status
    {
        bool connected;
        bool advertising;
        bool sending;
        unsigned fileCount;
        unsigned currentFileNumber;

        Status(
            bool connected_val,
            bool advertising_val,
            bool sending_val,
            unsigned fileCount_val,
            unsigned currentFileNumber_val
        ) :
            connected(connected_val),
            advertising(advertising_val),
            sending(sending_val),
            fileCount(fileCount_val),
            currentFileNumber(currentFileNumber_val)
        {}

        Status() :
            connected(false),
            advertising(false),
            sending(false),
            fileCount(0),
            currentFileNumber(0)
        {}
    };

    // Initialize BLE.
    void InitializeBLE();

    // Update BLE status. Mainly used for determining
    // whether to advertise or not based on button logic.
    void UpdateBLE(const Button::Mode& mode);

    // Returns a snapshot of the BLE status needed for display
    // updates.
    BLE::Status GetStatus();
}


#endif
