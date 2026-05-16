#ifndef DRAG_TYPES_H
#define DRAG_TYPES_H

namespace Drag
{
    enum class DragType
    {
        QuarterMile,
        EighthMile,
        StopToSpeed,
        RollingToSpeed,
        StopStartStop
    };

    enum class DragState
    {
        Idle,
        WaitingForGps,
        WaitingForReady,
        Armed,
        RandomDelay,
        WaitingForMovement,
        WaitingForRollingSpeed,
        HoldingRollingSpeed,
        Running,
        Braking,
        Finished,
        FalseStart,
        Aborted
    };

    enum class DragDistanceMethod
    {
        SpeedIntegrated,
        StartToCurrent
    };
}

#endif
