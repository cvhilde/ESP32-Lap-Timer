#ifndef BLE_H
#define BLE_H

#include <button.h>

namespace BLE
{
    void InitializeBLE();

    void UpdateBLE(const Button::Mode& mode);

    bool IsConnected();

    bool IsAdvertising();

    unsigned GetFileCount();

    unsigned GetCurrentFileNumber();

    bool IsSending();
}


#endif
