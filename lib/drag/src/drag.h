#ifndef DRAG_H
#define DRAG_H

#include <Arduino.h>
#include <button.h>
#include <drag_config.h>
#include <gps.h>

namespace Drag
{
    bool ConfigureFromJson(const String& json, String& error);
    void Abort();
    void Update(const GPS::FixData& fixData, Button::Mode buttonMode);
    bool IsActive();
    DragState GetState();
}

#endif
