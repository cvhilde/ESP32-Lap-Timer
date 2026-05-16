#ifndef DRAG_CONFIG_H
#define DRAG_CONFIG_H

#include <Arduino.h>
#include <drag_types.h>

namespace Drag
{
    enum class FalseStartBehavior
    {
        Abort
    };

    struct DragConfig
    {
        DragType type;
        double targetSpeedMph;
        double rollingStartSpeedMph;
        double rollingToleranceMph;
        uint32_t rollingHoldMs;
        double startThresholdMph;
        double stopThresholdMph;
        uint32_t stopHoldMs;
        uint32_t speedAccThresholdMmps;
        FalseStartBehavior falseStartBehavior;

        DragConfig():
            type(DragType::QuarterMile),
            targetSpeedMph(0.0),
            rollingStartSpeedMph(0.0),
            rollingToleranceMph(5.0),
            rollingHoldMs(2000U),
            startThresholdMph(1.0),
            stopThresholdMph(1.0),
            stopHoldMs(500U),
            speedAccThresholdMmps(100U),
            falseStartBehavior(FalseStartBehavior::Abort)
        {}
    };

    bool ParseDragConfigJson(const String& json, DragConfig& config, String& error);
    String DragConfigToJson(const DragConfig& config);
    const char* ToConfigString(DragType type);
    const char* ToConfigString(DragDistanceMethod method);
    DragDistanceMethod ChooseDistanceMethod(uint32_t speedAccEstMmps,
                                            uint32_t thresholdMmps);
}

#endif
