#include <drag_config.h>
#include <ArduinoJson.h>

namespace
{
    bool ParseType(const String& value, Drag::DragType& type)
    {
        if (value == "quarter_mile")
        {
            type = Drag::DragType::QuarterMile;
            return true;
        }

        if (value == "eighth_mile")
        {
            type = Drag::DragType::EighthMile;
            return true;
        }

        if (value == "stop_to_speed")
        {
            type = Drag::DragType::StopToSpeed;
            return true;
        }

        if (value == "rolling_to_speed")
        {
            type = Drag::DragType::RollingToSpeed;
            return true;
        }

        if (value == "stop_start_stop")
        {
            type = Drag::DragType::StopStartStop;
            return true;
        }

        return false;
    }

    const char* ReadString(JsonDocument& doc,
                           const char* longKey,
                           const char* shortKey,
                           const char* fallback)
    {
        if (!doc[longKey].isNull())
        {
            return doc[longKey] | fallback;
        }

        return doc[shortKey] | fallback;
    }

    void ReadDouble(JsonDocument& doc,
                    const char* longKey,
                    const char* shortKey,
                    double& value)
    {
        if (!doc[longKey].isNull())
        {
            value = doc[longKey].as<double>();
        }
        else if (!doc[shortKey].isNull())
        {
            value = doc[shortKey].as<double>();
        }
    }

    void ReadUInt(JsonDocument& doc,
                  const char* longKey,
                  const char* shortKey,
                  uint32_t& value)
    {
        if (!doc[longKey].isNull())
        {
            value = doc[longKey].as<uint32_t>();
        }
        else if (!doc[shortKey].isNull())
        {
            value = doc[shortKey].as<uint32_t>();
        }
    }

    bool RequirePositiveSpeed(JsonDocument& doc,
                              const char* longKey,
                              const char* shortKey,
                              double& value,
                              String& error)
    {
        if (doc[longKey].isNull() && doc[shortKey].isNull())
        {
            error = String("missing ") + longKey;
            return false;
        }

        ReadDouble(doc, longKey, shortKey, value);
        if (value <= 0.0)
        {
            error = String("invalid ") + longKey;
            return false;
        }

        return true;
    }

    bool ValidateConfig(const Drag::DragConfig& config, String& error)
    {
        if (config.rollingToleranceMph <= 0.0 ||
            config.rollingHoldMs == 0U ||
            config.startThresholdMph < 0.0 ||
            config.stopThresholdMph < 0.0 ||
            config.stopHoldMs == 0U ||
            config.speedAccThresholdMmps == 0U)
        {
            error = "invalid threshold";
            return false;
        }

        switch (config.type)
        {
            case Drag::DragType::StopToSpeed:
            case Drag::DragType::StopStartStop:
                if (config.targetSpeedMph <= 0.0)
                {
                    error = "invalid target_speed_mph";
                    return false;
                }
                break;

            case Drag::DragType::RollingToSpeed:
                if (config.targetSpeedMph <= 0.0)
                {
                    error = "invalid target_speed_mph";
                    return false;
                }
                if (config.rollingStartSpeedMph <= 0.0)
                {
                    error = "invalid rolling_start_speed_mph";
                    return false;
                }
                if (config.targetSpeedMph <= config.rollingStartSpeedMph)
                {
                    error = "target must exceed rolling speed";
                    return false;
                }
                break;

            case Drag::DragType::QuarterMile:
            case Drag::DragType::EighthMile:
                break;
        }

        return true;
    }
}

namespace Drag
{
    bool ParseDragConfigJson(const String& json, DragConfig& config, String& error)
    {
        JsonDocument doc;
        DeserializationError parseError = deserializeJson(doc, json);
        if (parseError)
        {
            error = "invalid json";
            return false;
        }

        const char* mode = ReadString(doc, "mode", "m", "");
        if (strlen(mode) > 0 && String(mode) != "drag")
        {
            error = "mode must be drag";
            return false;
        }

        const char* typeRaw = ReadString(doc, "type", "t", "");
        if (!ParseType(String(typeRaw), config.type))
        {
            error = "invalid drag type";
            return false;
        }

        ReadDouble(doc, "rolling_tolerance_mph", "rt", config.rollingToleranceMph);
        ReadUInt(doc, "rolling_hold_ms", "rhms", config.rollingHoldMs);
        ReadDouble(doc, "start_threshold_mph", "st", config.startThresholdMph);
        ReadDouble(doc, "stop_threshold_mph", "sp", config.stopThresholdMph);
        ReadUInt(doc, "stop_hold_ms", "shms", config.stopHoldMs);
        ReadUInt(doc, "speed_acc_threshold_mmps", "sacc", config.speedAccThresholdMmps);

        const char* falseStart =
            ReadString(doc, "false_start_behavior", "fs", "abort");
        if (String(falseStart) != "abort")
        {
            error = "unsupported false_start_behavior";
            return false;
        }

        config.falseStartBehavior = FalseStartBehavior::Abort;

        switch (config.type)
        {
            case DragType::StopToSpeed:
            case DragType::StopStartStop:
                if (!RequirePositiveSpeed(doc, "target_speed_mph", "ts",
                                          config.targetSpeedMph, error))
                {
                    return false;
                }
                break;

            case DragType::RollingToSpeed:
                if (!RequirePositiveSpeed(doc, "target_speed_mph", "ts",
                                          config.targetSpeedMph, error) ||
                    !RequirePositiveSpeed(doc, "rolling_start_speed_mph", "rs",
                                          config.rollingStartSpeedMph, error))
                {
                    return false;
                }
                if (config.targetSpeedMph <= config.rollingStartSpeedMph)
                {
                    error = "target must exceed rolling speed";
                    return false;
                }
                break;

            case DragType::QuarterMile:
            case DragType::EighthMile:
                config.targetSpeedMph = 0.0;
                config.rollingStartSpeedMph = 0.0;
                break;
        }

        return ValidateConfig(config, error);
    }

    String DragConfigToJson(const DragConfig& config)
    {
        JsonDocument doc;
        doc["mode"] = "drag";
        doc["type"] = ToConfigString(config.type);

        if (config.targetSpeedMph > 0.0)
        {
            doc["target_speed_mph"] = config.targetSpeedMph;
        }
        else
        {
            doc["target_speed_mph"] = nullptr;
        }

        if (config.rollingStartSpeedMph > 0.0)
        {
            doc["rolling_start_speed_mph"] = config.rollingStartSpeedMph;
        }
        else
        {
            doc["rolling_start_speed_mph"] = nullptr;
        }

        doc["rolling_tolerance_mph"] = config.rollingToleranceMph;
        doc["rolling_hold_ms"] = config.rollingHoldMs;
        doc["start_threshold_mph"] = config.startThresholdMph;
        doc["stop_threshold_mph"] = config.stopThresholdMph;
        doc["stop_hold_ms"] = config.stopHoldMs;
        doc["speed_acc_threshold_mmps"] = config.speedAccThresholdMmps;
        doc["false_start_behavior"] = "abort";

        String out;
        serializeJson(doc, out);
        return out;
    }

    const char* ToConfigString(DragType type)
    {
        switch (type)
        {
            case DragType::QuarterMile:
                return "quarter_mile";
            case DragType::EighthMile:
                return "eighth_mile";
            case DragType::StopToSpeed:
                return "stop_to_speed";
            case DragType::RollingToSpeed:
                return "rolling_to_speed";
            case DragType::StopStartStop:
                return "stop_start_stop";
            default:
                return "unknown";
        }
    }

    const char* ToConfigString(DragDistanceMethod method)
    {
        switch (method)
        {
            case DragDistanceMethod::SpeedIntegrated:
                return "SpeedIntegrated";
            case DragDistanceMethod::StartToCurrent:
                return "StartToCurrent";
            default:
                return "Unknown";
        }
    }

    DragDistanceMethod ChooseDistanceMethod(uint32_t speedAccEstMmps,
                                            uint32_t thresholdMmps)
    {
        if (speedAccEstMmps > 0U && speedAccEstMmps <= thresholdMmps)
        {
            return DragDistanceMethod::SpeedIntegrated;
        }

        return DragDistanceMethod::StartToCurrent;
    }
}
