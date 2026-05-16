#include <drag.h>
#include <Arduino.h>
#include <FS.h>
#include <esp_system.h>
#include <led.h>
#include <math.h>
#include <storage.h>
#include <waypoints.h>

namespace
{
    constexpr unsigned DRAG_GPS_HZ = 25U;
    constexpr uint32_t MAX_GPS_AGE_MS = 200U;
    constexpr uint8_t START_DEBOUNCE_SAMPLES = 3U;
    constexpr uint8_t ROLLING_BAD_SAMPLE_LIMIT = 3U;
    constexpr uint32_t RANDOM_DELAY_MIN_MS = 3000U;
    constexpr uint32_t RANDOM_DELAY_SPAN_MS = 3001U;
    constexpr uint32_t FINISH_BLINK_MS = 5000U;
    constexpr double FEET_PER_METER = 3.28084;
    constexpr double MPH_TO_MPS = 0.44704;

    struct DragSample
    {
        WayPoints::Coord coord;
        uint64_t timeUs;
        double speedMph;
        double speedMps;
        uint32_t speedAccMmps;

        DragSample():
            coord(),
            timeUs(0U),
            speedMph(0.0),
            speedMps(0.0),
            speedAccMmps(0U)
        {}
    };

    struct EventMetrics
    {
        uint64_t timeUs;
        double speedMph;
        uint32_t speedAccMmps;
        double officialDistanceFt;
        double integratedDistanceFt;
        double radialDistanceFt;

        EventMetrics():
            timeUs(0U),
            speedMph(0.0),
            speedAccMmps(0U),
            officialDistanceFt(0.0),
            integratedDistanceFt(0.0),
            radialDistanceFt(0.0)
        {}
    };

    struct Data
    {
        Drag::DragConfig config;
        Drag::DragState state;
        Drag::DragState ledState;
        Drag::DragDistanceMethod distanceMethod;

        bool distanceMethodChosen;
        bool filesOpen;
        bool hasLastSample;
        bool hasLastDistanceSample;
        bool finishAfterRoute;
        bool sixtyLogged;
        bool threeThirtyLogged;
        bool eighthLogged;
        bool thousandLogged;
        bool quarterLogged;
        bool targetLogged;
        bool stopPending;
        bool gpsRateApplied;
        bool abortRequested;

        uint8_t movementSamples;
        uint8_t rollingBadSamples;
        uint32_t randomDelayEndMs;
        uint32_t terminalStateEnteredMs;
        uint64_t logStartUs;
        uint64_t lightsOutTimeUs;
        uint64_t movementStartTimeUs;
        uint64_t stopHoldStartUs;

        double integratedDistanceM;
        double integratedDistanceFt;
        double radialDistanceFt;
        double officialDistanceFt;

        DragSample lastSample;
        DragSample lastDistanceSample;
        DragSample pendingMovementSample;
        EventMetrics pendingStopMetrics;
        WayPoints::Coord startCoord;

        String timestamp;
        String routePath;
        String eventsPath;
        String configPath;

        File routeFile;
        File eventsFile;

        Data():
            config(),
            state(Drag::DragState::Idle),
            ledState(Drag::DragState::Idle),
            distanceMethod(Drag::DragDistanceMethod::SpeedIntegrated),
            distanceMethodChosen(false),
            filesOpen(false),
            hasLastSample(false),
            hasLastDistanceSample(false),
            finishAfterRoute(false),
            sixtyLogged(false),
            threeThirtyLogged(false),
            eighthLogged(false),
            thousandLogged(false),
            quarterLogged(false),
            targetLogged(false),
            stopPending(false),
            gpsRateApplied(false),
            abortRequested(false),
            movementSamples(0U),
            rollingBadSamples(0U),
            randomDelayEndMs(0U),
            terminalStateEnteredMs(0U),
            logStartUs(0U),
            lightsOutTimeUs(0U),
            movementStartTimeUs(0U),
            stopHoldStartUs(0U),
            integratedDistanceM(0.0),
            integratedDistanceFt(0.0),
            radialDistanceFt(0.0),
            officialDistanceFt(0.0),
            lastSample(),
            lastDistanceSample(),
            pendingMovementSample(),
            pendingStopMetrics(),
            startCoord(),
            timestamp(),
            routePath(),
            eventsPath(),
            configPath(),
            routeFile(),
            eventsFile()
        {}
    };

    Data _data;

    //------------------------------------------------------------------------
    const char* StateString(Drag::DragState state)
    {
        switch (state)
        {
            case Drag::DragState::Idle:
                return "IDLE";
            case Drag::DragState::WaitingForGps:
                return "WAITING_FOR_GPS";
            case Drag::DragState::WaitingForReady:
                return "WAITING_FOR_READY";
            case Drag::DragState::Armed:
                return "ARMED";
            case Drag::DragState::RandomDelay:
                return "RANDOM_DELAY";
            case Drag::DragState::WaitingForMovement:
                return "WAITING_FOR_MOVEMENT";
            case Drag::DragState::WaitingForRollingSpeed:
                return "WAITING_FOR_ROLLING_SPEED";
            case Drag::DragState::HoldingRollingSpeed:
                return "HOLDING_ROLLING_SPEED";
            case Drag::DragState::Running:
                return "RUNNING";
            case Drag::DragState::Braking:
                return "BRAKING";
            case Drag::DragState::Finished:
                return "FINISHED";
            case Drag::DragState::FalseStart:
                return "FALSE_START";
            case Drag::DragState::Aborted:
                return "ABORTED";
            default:
                return "UNKNOWN";
        }
    }

    //------------------------------------------------------------------------
    bool IsTerminalState(Drag::DragState state)
    {
        return state == Drag::DragState::Finished ||
               state == Drag::DragState::FalseStart ||
               state == Drag::DragState::Aborted;
    }

    //------------------------------------------------------------------------
    bool IsTimingState(Drag::DragState state)
    {
        return state == Drag::DragState::Running ||
               state == Drag::DragState::Braking;
    }

    //------------------------------------------------------------------------
    bool IsDistanceRun()
    {
        return _data.config.type == Drag::DragType::QuarterMile ||
               _data.config.type == Drag::DragType::EighthMile;
    }

    //------------------------------------------------------------------------
    double DistanceFeet(const WayPoints::Coord& p1, const WayPoints::Coord& p2)
    {
        double avgLatRadians = ((p1.lat + p2.lat) * 0.5) *
            WayPoints::DEG_TO_RADIANS;
        double x = (p2.lng - p1.lng) * WayPoints::DEG_TO_RADIANS *
            cos(avgLatRadians);
        double y = (p2.lat - p1.lat) * WayPoints::DEG_TO_RADIANS;
        return sqrt((x * x) + (y * y)) * WayPoints::EARTH_RADIUS_FT;
    }

    //------------------------------------------------------------------------
    WayPoints::Coord InterpolateCoord(const WayPoints::Coord& prev,
                                      const WayPoints::Coord& curr,
                                      double ratio)
    {
        WayPoints::Coord coord;
        coord.lat = prev.lat + ((curr.lat - prev.lat) * ratio);
        coord.lng = prev.lng + ((curr.lng - prev.lng) * ratio);
        return coord;
    }

    //------------------------------------------------------------------------
    double InterpolateDouble(double prev, double curr, double ratio)
    {
        return prev + ((curr - prev) * ratio);
    }

    //------------------------------------------------------------------------
    uint64_t InterpolateTime(uint64_t prev, uint64_t curr, double ratio)
    {
        double delta = static_cast<double>(curr - prev) * ratio;
        return prev + static_cast<uint64_t>(delta);
    }

    //------------------------------------------------------------------------
    DragSample InterpolateSample(const DragSample& prev,
                                 const DragSample& curr,
                                 double ratio)
    {
        DragSample sample;
        sample.coord = InterpolateCoord(prev.coord, curr.coord, ratio);
        sample.timeUs = InterpolateTime(prev.timeUs, curr.timeUs, ratio);
        sample.speedMph = InterpolateDouble(prev.speedMph, curr.speedMph, ratio);
        sample.speedMps = sample.speedMph * MPH_TO_MPS;
        sample.speedAccMmps = curr.speedAccMmps;
        return sample;
    }

    //------------------------------------------------------------------------
    EventMetrics InterpolateMetrics(const EventMetrics& prev,
                                    const EventMetrics& curr,
                                    double ratio)
    {
        EventMetrics metrics;
        metrics.timeUs = InterpolateTime(prev.timeUs, curr.timeUs, ratio);
        metrics.speedMph = InterpolateDouble(prev.speedMph, curr.speedMph, ratio);
        metrics.speedAccMmps = curr.speedAccMmps;
        metrics.officialDistanceFt =
            InterpolateDouble(prev.officialDistanceFt,
                              curr.officialDistanceFt,
                              ratio);
        metrics.integratedDistanceFt =
            InterpolateDouble(prev.integratedDistanceFt,
                              curr.integratedDistanceFt,
                              ratio);
        metrics.radialDistanceFt =
            InterpolateDouble(prev.radialDistanceFt,
                              curr.radialDistanceFt,
                              ratio);
        return metrics;
    }

    //------------------------------------------------------------------------
    EventMetrics MetricsForSample(const DragSample& sample)
    {
        EventMetrics metrics;
        metrics.timeUs = sample.timeUs;
        metrics.speedMph = sample.speedMph;
        metrics.speedAccMmps = sample.speedAccMmps;
        metrics.officialDistanceFt = _data.officialDistanceFt;
        metrics.integratedDistanceFt = _data.integratedDistanceFt;
        metrics.radialDistanceFt = _data.radialDistanceFt;
        return metrics;
    }

    //------------------------------------------------------------------------
    uint32_t RelativeTimeUs(uint64_t absoluteTimeUs)
    {
        if (absoluteTimeUs <= _data.logStartUs)
        {
            return 0U;
        }

        return static_cast<uint32_t>(absoluteTimeUs - _data.logStartUs);
    }

    //------------------------------------------------------------------------
    double RunTimeSeconds(uint64_t eventTimeUs)
    {
        if (_data.movementStartTimeUs == 0U ||
            eventTimeUs <= _data.movementStartTimeUs)
        {
            return 0.0;
        }

        return (eventTimeUs - _data.movementStartTimeUs) / 1000000.0;
    }

    //------------------------------------------------------------------------
    bool IsGpsUsable(const GPS::FixData& fixData)
    {
        if (!fixData.valid ||
            !fixData.speedAccValid ||
            fixData.speedAccEstMmps == 0U)
        {
            return false;
        }

        if (fixData.updateTimeMs == 0U ||
            (millis() - fixData.updateTimeMs) > MAX_GPS_AGE_MS)
        {
            return false;
        }

        if (fixData.coord.lat == 0.0 && fixData.coord.lng == 0.0)
        {
            return false;
        }

        return true;
    }

    //------------------------------------------------------------------------
    DragSample SampleFromFix(const GPS::FixData& fixData)
    {
        DragSample sample;
        sample.coord = fixData.coord;
        sample.timeUs = static_cast<uint64_t>(micros());
        sample.speedMph = fixData.speed;
        sample.speedMps = fixData.speed * MPH_TO_MPS;
        sample.speedAccMmps = fixData.speedAccEstMmps;
        return sample;
    }

    //------------------------------------------------------------------------
    String TimestampFromGps(const GPS::GPSTimeData& time)
    {
        char timestamp[25];
        snprintf(timestamp, sizeof(timestamp), "%04u%02u%02u_%02u%02u%02u",
                 time.year,
                 time.month,
                 time.day,
                 time.hour,
                 time.minute,
                 time.second);
        return String(timestamp);
    }

    //------------------------------------------------------------------------
    void CloseFiles()
    {
        if (_data.routeFile)
        {
            _data.routeFile.close();
        }

        if (_data.eventsFile)
        {
            _data.eventsFile.close();
        }

        _data.filesOpen = false;
    }

    //------------------------------------------------------------------------
    void ResetForNextSession()
    {
        CloseFiles();
        _data = Data{};
        Led::StopBlink();
        GPS::SetUpdateFrequency(1U);
    }

    //------------------------------------------------------------------------
    void ApplyLedForState(Drag::DragState state)
    {
        if (_data.ledState == state)
        {
            return;
        }

        _data.ledState = state;

        switch (state)
        {
            case Drag::DragState::WaitingForReady:
                Led::StartBlink(750U);
                break;
            case Drag::DragState::WaitingForRollingSpeed:
            case Drag::DragState::Braking:
                Led::StartBlink(150U);
                break;
            case Drag::DragState::Armed:
            case Drag::DragState::RandomDelay:
            case Drag::DragState::HoldingRollingSpeed:
                Led::StopBlink();
                Led::TurnLedOn();
                break;
            case Drag::DragState::WaitingForMovement:
            case Drag::DragState::Running:
                Led::StopBlink();
                Led::TurnLedOff();
                break;
            case Drag::DragState::Finished:
                Led::StartOneShotBlink(500U, FINISH_BLINK_MS);
                break;
            case Drag::DragState::WaitingForGps:
            case Drag::DragState::FalseStart:
            case Drag::DragState::Aborted:
                Led::StartBlink(100U);
                break;
            case Drag::DragState::Idle:
            default:
                Led::StopBlink();
                break;
        }
    }

    //------------------------------------------------------------------------
    void SetState(Drag::DragState state)
    {
        _data.state = state;
        if (IsTerminalState(state))
        {
            _data.terminalStateEnteredMs = millis();
        }
        ApplyLedForState(state);
    }

    //------------------------------------------------------------------------
    void ResetMovementDebounce()
    {
        _data.movementSamples = 0U;
        _data.pendingMovementSample = DragSample{};
    }

    //------------------------------------------------------------------------
    bool OpenLogs(const GPS::FixData& fixData, const DragSample& sample)
    {
        if (!fixData.dateTime.valid)
        {
            return false;
        }

        _data.timestamp = TimestampFromGps(fixData.dateTime);
        _data.routePath = Storage::DRAG_ROUTE_PREFIX +
            _data.timestamp + Storage::FILE_TYPE;
        _data.eventsPath = Storage::DRAG_EVENTS_PREFIX +
            _data.timestamp + Storage::FILE_TYPE;
        _data.configPath = Storage::DRAG_CONFIG_PREFIX +
            _data.timestamp + Storage::JSON_FILE_TYPE;

        _data.routeFile = Storage::GetFile(_data.routePath, "w");
        _data.eventsFile = Storage::GetFile(_data.eventsPath, "w");
        File configFile = Storage::GetFile(_data.configPath, "w");
        File manifest = Storage::GetFile(Storage::MANIFEST_FILE, "a");

        if (!_data.routeFile || !_data.eventsFile || !configFile || !manifest)
        {
            if (configFile)
            {
                configFile.close();
            }
            if (manifest)
            {
                manifest.close();
            }
            CloseFiles();
            return false;
        }

        _data.logStartUs = sample.timeUs;

        _data.routeFile.println(
            "time_us,lat,lng,speed_mph,speed_acc_mmps,"
            "integrated_distance_ft,radial_distance_ft,official_distance_ft,"
            "distance_method,state");
        _data.eventsFile.println(
            "event,time_us,run_time_s,speed_mph,speed_acc_mmps,"
            "official_distance_ft,integrated_distance_ft,radial_distance_ft");

        configFile.println(Drag::DragConfigToJson(_data.config));
        configFile.close();

        manifest.println(_data.timestamp);
        manifest.close();

        _data.filesOpen = true;
        return true;
    }

    //------------------------------------------------------------------------
    void LogRouteSample(const DragSample& sample)
    {
        if (!_data.filesOpen || !_data.routeFile)
        {
            return;
        }

        const char* method = _data.distanceMethodChosen
            ? Drag::ToConfigString(_data.distanceMethod)
            : "Unselected";

        _data.routeFile.printf(
            "%lu,%.7lf,%.7lf,%.2lf,%lu,%.2lf,%.2lf,%.2lf,%s,%s\n",
            static_cast<unsigned long>(RelativeTimeUs(sample.timeUs)),
            sample.coord.lat,
            sample.coord.lng,
            sample.speedMph,
            static_cast<unsigned long>(sample.speedAccMmps),
            _data.integratedDistanceFt,
            _data.radialDistanceFt,
            _data.officialDistanceFt,
            method,
            StateString(_data.state));
    }

    //------------------------------------------------------------------------
    void LogEvent(const char* event, const EventMetrics& metrics)
    {
        if (!_data.filesOpen || !_data.eventsFile)
        {
            return;
        }

        _data.eventsFile.printf(
            "%s,%lu,%.3lf,%.2lf,%lu,%.2lf,%.2lf,%.2lf\n",
            event,
            static_cast<unsigned long>(RelativeTimeUs(metrics.timeUs)),
            RunTimeSeconds(metrics.timeUs),
            metrics.speedMph,
            static_cast<unsigned long>(metrics.speedAccMmps),
            metrics.officialDistanceFt,
            metrics.integratedDistanceFt,
            metrics.radialDistanceFt);
        _data.eventsFile.flush();
    }

    //------------------------------------------------------------------------
    void IntegrateDistanceSegment(const DragSample& from, const DragSample& to)
    {
        if (to.timeUs <= from.timeUs)
        {
            return;
        }

        double dtSec = (to.timeUs - from.timeUs) / 1000000.0;
        if (dtSec <= 0.0 || dtSec > 0.5)
        {
            return;
        }

        if (IsTimingState(_data.state))
        {
            double avgSpeedMps = 0.5 * (from.speedMps + to.speedMps);
            _data.integratedDistanceM += avgSpeedMps * dtSec;
        }

        _data.integratedDistanceFt = _data.integratedDistanceM * FEET_PER_METER;
        _data.radialDistanceFt = DistanceFeet(_data.startCoord, to.coord);
        _data.officialDistanceFt =
            _data.distanceMethod == Drag::DragDistanceMethod::SpeedIntegrated
                ? _data.integratedDistanceFt
                : _data.radialDistanceFt;
    }

    //------------------------------------------------------------------------
    void UpdateDistances(const DragSample& sample)
    {
        if (!_data.hasLastDistanceSample)
        {
            _data.lastDistanceSample = sample;
            _data.hasLastDistanceSample = true;
            return;
        }

        IntegrateDistanceSegment(_data.lastDistanceSample, sample);
        _data.lastDistanceSample = sample;
    }

    //------------------------------------------------------------------------
    void BeginRun(const DragSample& startSample)
    {
        _data.distanceMethod = Drag::ChooseDistanceMethod(
            startSample.speedAccMmps,
            _data.config.speedAccThresholdMmps);
        _data.distanceMethodChosen = true;

        _data.startCoord = startSample.coord;
        _data.integratedDistanceM = 0.0;
        _data.integratedDistanceFt = 0.0;
        _data.radialDistanceFt = 0.0;
        _data.officialDistanceFt = 0.0;
        _data.lastDistanceSample = startSample;
        _data.hasLastDistanceSample = true;
        _data.movementStartTimeUs = startSample.timeUs;
        SetState(Drag::DragState::Running);
    }

    //------------------------------------------------------------------------
    bool UpdateMovementDebounce(const DragSample& sample,
                                bool hasPrevious,
                                const DragSample& previous,
                                DragSample& movementSample)
    {
        if (sample.speedMph >= _data.config.startThresholdMph)
        {
            if (_data.movementSamples == 0U)
            {
                if (hasPrevious &&
                    previous.speedMph < _data.config.startThresholdMph &&
                    sample.speedMph > previous.speedMph)
                {
                    double ratio =
                        (_data.config.startThresholdMph - previous.speedMph) /
                        (sample.speedMph - previous.speedMph);
                    _data.pendingMovementSample =
                        InterpolateSample(previous, sample, ratio);
                }
                else
                {
                    _data.pendingMovementSample = sample;
                }
            }

            _data.movementSamples++;
            if (_data.movementSamples >= START_DEBOUNCE_SAMPLES)
            {
                movementSample = _data.pendingMovementSample;
                return true;
            }
        }
        else
        {
            ResetMovementDebounce();
        }

        return false;
    }

    //------------------------------------------------------------------------
    void FinishRun(const char* finalEvent, const EventMetrics& metrics)
    {
        if (finalEvent != nullptr)
        {
            LogEvent(finalEvent, metrics);
        }
        LogEvent("FINISHED", metrics);
        _data.finishAfterRoute = true;
        SetState(Drag::DragState::Finished);
    }

    //------------------------------------------------------------------------
    void AbortWithEvent(const char* event, const DragSample& sample,
                        Drag::DragState state)
    {
        EventMetrics metrics = MetricsForSample(sample);
        LogEvent(event, metrics);
        _data.finishAfterRoute = true;
        SetState(state);
    }

    //------------------------------------------------------------------------
    EventMetrics CurrentMetrics(const DragSample& sample)
    {
        return MetricsForSample(sample);
    }

    //------------------------------------------------------------------------
    void CheckDistanceSplit(const char* event,
                            double targetFt,
                            bool& logged,
                            const EventMetrics& prev,
                            const EventMetrics& curr,
                            bool finish)
    {
        if (logged ||
            curr.officialDistanceFt < targetFt ||
            prev.officialDistanceFt >= targetFt)
        {
            return;
        }

        double delta = curr.officialDistanceFt - prev.officialDistanceFt;
        if (delta <= 0.0)
        {
            return;
        }

        double ratio = (targetFt - prev.officialDistanceFt) / delta;
        EventMetrics eventMetrics = InterpolateMetrics(prev, curr, ratio);
        eventMetrics.officialDistanceFt = targetFt;
        logged = true;

        if (finish)
        {
            FinishRun(event, eventMetrics);
        }
        else
        {
            LogEvent(event, eventMetrics);
        }
    }

    //------------------------------------------------------------------------
    bool CheckSpeedTarget(const char* event,
                          double targetMph,
                          const EventMetrics& prev,
                          const EventMetrics& curr,
                          EventMetrics& eventMetrics)
    {
        if (prev.speedMph >= targetMph || curr.speedMph < targetMph)
        {
            return false;
        }

        double delta = curr.speedMph - prev.speedMph;
        if (delta <= 0.0)
        {
            return false;
        }

        double ratio = (targetMph - prev.speedMph) / delta;
        eventMetrics = InterpolateMetrics(prev, curr, ratio);
        eventMetrics.speedMph = targetMph;
        LogEvent(event, eventMetrics);
        return true;
    }

    //------------------------------------------------------------------------
    void HandleWaitingForGps(const GPS::FixData& fixData,
                             const DragSample& sample)
    {
        if (!IsGpsUsable(fixData))
        {
            return;
        }

        if (!_data.filesOpen && !OpenLogs(fixData, sample))
        {
            AbortWithEvent("LOG_OPEN_FAILED", sample, Drag::DragState::Aborted);
            return;
        }

        if (_data.config.type == Drag::DragType::RollingToSpeed)
        {
            SetState(Drag::DragState::WaitingForRollingSpeed);
        }
        else
        {
            SetState(Drag::DragState::WaitingForReady);
        }
    }

    //------------------------------------------------------------------------
    void HandleWaitingForReady(Button::Mode buttonMode)
    {
        if (buttonMode != Button::Mode::SHORT)
        {
            return;
        }

        ResetMovementDebounce();

        if (IsDistanceRun())
        {
            _data.randomDelayEndMs = millis() + RANDOM_DELAY_MIN_MS +
                (esp_random() % RANDOM_DELAY_SPAN_MS);
            SetState(Drag::DragState::RandomDelay);
        }
        else
        {
            SetState(Drag::DragState::Armed);
        }
    }

    //------------------------------------------------------------------------
    void HandleRandomDelay(const DragSample& sample,
                           bool hasPrevious,
                           const DragSample& previous)
    {
        DragSample movement;
        if (UpdateMovementDebounce(sample, hasPrevious, previous, movement))
        {
            AbortWithEvent("FALSE_START", movement, Drag::DragState::FalseStart);
            return;
        }

        if (millis() >= _data.randomDelayEndMs)
        {
            EventMetrics lightsOut = CurrentMetrics(sample);
            _data.lightsOutTimeUs = sample.timeUs;
            LogEvent("LIGHTS_OUT", lightsOut);
            ResetMovementDebounce();
            SetState(Drag::DragState::WaitingForMovement);
        }
    }

    //------------------------------------------------------------------------
    void HandleArmed(const DragSample& sample,
                     bool hasPrevious,
                     const DragSample& previous)
    {
        DragSample movement;
        if (!UpdateMovementDebounce(sample, hasPrevious, previous, movement))
        {
            return;
        }

        EventMetrics metrics = CurrentMetrics(movement);
        BeginRun(movement);
        LogEvent("MOVEMENT_START", metrics);
        IntegrateDistanceSegment(movement, sample);
    }

    //------------------------------------------------------------------------
    void HandleWaitingForMovement(const DragSample& sample,
                                  bool hasPrevious,
                                  const DragSample& previous)
    {
        DragSample movement;
        if (!UpdateMovementDebounce(sample, hasPrevious, previous, movement))
        {
            return;
        }

        EventMetrics metrics = CurrentMetrics(movement);
        BeginRun(movement);
        LogEvent("MOVEMENT_START", metrics);
        IntegrateDistanceSegment(movement, sample);
    }

    //------------------------------------------------------------------------
    bool IsInRollingWindow(double speedMph, double toleranceExtraMph)
    {
        double low = _data.config.rollingStartSpeedMph -
            _data.config.rollingToleranceMph - toleranceExtraMph;
        double high = _data.config.rollingStartSpeedMph +
            _data.config.rollingToleranceMph + toleranceExtraMph;
        return speedMph >= low && speedMph <= high;
    }

    //------------------------------------------------------------------------
    void HandleWaitingForRollingSpeed(const DragSample& sample)
    {
        if (IsInRollingWindow(sample.speedMph, 0.0))
        {
            _data.stopHoldStartUs = sample.timeUs;
            _data.rollingBadSamples = 0U;
            SetState(Drag::DragState::HoldingRollingSpeed);
        }
    }

    //------------------------------------------------------------------------
    void HandleHoldingRollingSpeed(const DragSample& sample)
    {
        if (!IsInRollingWindow(sample.speedMph, 1.0))
        {
            _data.rollingBadSamples++;
            if (_data.rollingBadSamples >= ROLLING_BAD_SAMPLE_LIMIT)
            {
                _data.rollingBadSamples = 0U;
                SetState(Drag::DragState::WaitingForRollingSpeed);
            }
            return;
        }

        _data.rollingBadSamples = 0U;
        if ((sample.timeUs - _data.stopHoldStartUs) >=
            (_data.config.rollingHoldMs * 1000ULL))
        {
            BeginRun(sample);
            EventMetrics metrics = CurrentMetrics(sample);
            LogEvent("ROLLING_START", metrics);
        }
    }

    //------------------------------------------------------------------------
    void HandleRunning(const EventMetrics& prev, const EventMetrics& curr)
    {
        if (IsDistanceRun())
        {
            CheckDistanceSplit("SIXTY_FT", 60.0, _data.sixtyLogged,
                               prev, curr, false);
            CheckDistanceSplit("THREE_THIRTY_FT", 330.0,
                               _data.threeThirtyLogged, prev, curr, false);

            bool eighthIsFinish =
                _data.config.type == Drag::DragType::EighthMile;
            CheckDistanceSplit("EIGHTH_MILE", 660.0, _data.eighthLogged,
                               prev, curr, eighthIsFinish);

            if (_data.state != Drag::DragState::Running)
            {
                return;
            }

            if (_data.config.type == Drag::DragType::QuarterMile)
            {
                CheckDistanceSplit("THOUSAND_FT", 1000.0,
                                   _data.thousandLogged, prev, curr, false);
                CheckDistanceSplit("QUARTER_MILE", 1320.0,
                                   _data.quarterLogged, prev, curr, true);
            }
            return;
        }

        EventMetrics targetMetrics;
        if (!_data.targetLogged &&
            CheckSpeedTarget("TARGET_SPEED",
                             _data.config.targetSpeedMph,
                             prev,
                             curr,
                             targetMetrics))
        {
            _data.targetLogged = true;
            if (_data.config.type == Drag::DragType::StopStartStop)
            {
                SetState(Drag::DragState::Braking);
            }
            else
            {
                FinishRun(nullptr, targetMetrics);
            }
        }
    }

    //------------------------------------------------------------------------
    void HandleBraking(const EventMetrics& prev,
                       const EventMetrics& curr,
                       const DragSample& sample)
    {
        if (sample.speedMph > _data.config.stopThresholdMph)
        {
            _data.stopPending = false;
            _data.stopHoldStartUs = 0U;
            return;
        }

        if (!_data.stopPending)
        {
            _data.stopPending = true;

            if (prev.speedMph > _data.config.stopThresholdMph &&
                curr.speedMph <= _data.config.stopThresholdMph)
            {
                double delta = prev.speedMph - curr.speedMph;
                double ratio = delta > 0.0
                    ? (prev.speedMph - _data.config.stopThresholdMph) / delta
                    : 1.0;
                _data.pendingStopMetrics = InterpolateMetrics(prev, curr, ratio);
                _data.pendingStopMetrics.speedMph = _data.config.stopThresholdMph;
            }
            else
            {
                _data.pendingStopMetrics = curr;
            }

            _data.stopHoldStartUs = _data.pendingStopMetrics.timeUs;
        }

        if ((sample.timeUs - _data.stopHoldStartUs) >=
            (_data.config.stopHoldMs * 1000ULL))
        {
            LogEvent("STOPPED", _data.pendingStopMetrics);
            FinishRun(nullptr, _data.pendingStopMetrics);
        }
    }
}

namespace Drag
{
    //------------------------------------------------------------------------
    bool ConfigureFromJson(const String& json, String& error)
    {
        if (_data.state != DragState::Idle)
        {
            error = "drag active";
            return false;
        }

        DragConfig config;
        if (!ParseDragConfigJson(json, config, error))
        {
            return false;
        }

        _data = Data{};
        _data.config = config;
        SetState(DragState::WaitingForGps);
        return true;
    }

    //------------------------------------------------------------------------
    void Abort()
    {
        if (_data.state == DragState::Idle)
        {
            return;
        }

        _data.abortRequested = true;
    }

    //------------------------------------------------------------------------
    void Update(const GPS::FixData& fixData, Button::Mode buttonMode)
    {
        if (_data.state == DragState::Idle)
        {
            return;
        }

        if (!_data.gpsRateApplied)
        {
            GPS::SetUpdateFrequency(DRAG_GPS_HZ);
            _data.gpsRateApplied = true;
        }

        if (IsTerminalState(_data.state))
        {
            if (_data.finishAfterRoute)
            {
                CloseFiles();
                _data.finishAfterRoute = false;
            }

            if ((millis() - _data.terminalStateEnteredMs) >= FINISH_BLINK_MS)
            {
                ResetForNextSession();
            }
            return;
        }

        if (_data.abortRequested && !IsGpsUsable(fixData))
        {
            DragSample sample = _data.hasLastSample
                ? _data.lastSample
                : DragSample{};
            sample.timeUs = sample.timeUs == 0U
                ? static_cast<uint64_t>(micros())
                : sample.timeUs;
            AbortWithEvent("ABORTED", sample, DragState::Aborted);
            return;
        }

        if (!IsGpsUsable(fixData))
        {
            if (IsTimingState(_data.state) ||
                _data.state == DragState::RandomDelay ||
                _data.state == DragState::WaitingForMovement ||
                _data.state == DragState::Armed ||
                _data.state == DragState::HoldingRollingSpeed)
            {
                DragSample sample = _data.hasLastSample
                    ? _data.lastSample
                    : DragSample{};
                AbortWithEvent("INVALID_GPS", sample, DragState::Aborted);
            }
            else
            {
                SetState(DragState::WaitingForGps);
            }
            return;
        }

        DragSample sample = SampleFromFix(fixData);

        if (_data.abortRequested)
        {
            AbortWithEvent("ABORTED", sample, DragState::Aborted);
            LogRouteSample(sample);
            if (_data.finishAfterRoute)
            {
                CloseFiles();
                _data.finishAfterRoute = false;
            }
            _data.lastSample = sample;
            _data.hasLastSample = true;
            return;
        }

        bool hasPrevious = _data.hasLastSample;
        DragSample previous = _data.lastSample;

        EventMetrics prevMetrics = MetricsForSample(previous);
        if (IsTimingState(_data.state))
        {
            UpdateDistances(sample);
        }
        EventMetrics currMetrics = MetricsForSample(sample);

        switch (_data.state)
        {
            case DragState::WaitingForGps:
                HandleWaitingForGps(fixData, sample);
                break;
            case DragState::WaitingForReady:
                HandleWaitingForReady(buttonMode);
                break;
            case DragState::Armed:
                HandleArmed(sample, hasPrevious, previous);
                break;
            case DragState::RandomDelay:
                HandleRandomDelay(sample, hasPrevious, previous);
                break;
            case DragState::WaitingForMovement:
                HandleWaitingForMovement(sample, hasPrevious, previous);
                break;
            case DragState::WaitingForRollingSpeed:
                HandleWaitingForRollingSpeed(sample);
                break;
            case DragState::HoldingRollingSpeed:
                HandleHoldingRollingSpeed(sample);
                break;
            case DragState::Running:
                HandleRunning(prevMetrics, currMetrics);
                break;
            case DragState::Braking:
                HandleBraking(prevMetrics, currMetrics, sample);
                break;
            default:
                break;
        }

        LogRouteSample(sample);

        if (_data.finishAfterRoute)
        {
            CloseFiles();
            _data.finishAfterRoute = false;
        }

        _data.lastSample = sample;
        _data.hasLastSample = true;
    }

    //------------------------------------------------------------------------
    bool IsActive()
    {
        return _data.state != DragState::Idle;
    }

    //------------------------------------------------------------------------
    DragState GetState()
    {
        return _data.state;
    }
}
