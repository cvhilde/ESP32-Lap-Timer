#ifndef BLE_H
#define BLE_H

#include <button.h>

namespace BLE
{
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

    void InitializeBLE();

    void UpdateBLE(const Button::Mode& mode);

    BLE::Status GetStatus();
}


#endif
