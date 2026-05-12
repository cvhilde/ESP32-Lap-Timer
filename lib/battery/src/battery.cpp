#include <battery.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    struct Point
    {
        float volts;
        int percent;
    };

    // This should be set according to if the device has a
    // battery as the main power source or not. This is used to
    // determine if battery percentage logic is to take place, or
    // to just report the raw input voltage as read on VADC_IN.
    constexpr bool DEVICE_HAS_BATTERY = true;

    // Actual pin number for the battery voltage output pin.
    // Labeled as VBAT_Read on the actual device.
    constexpr uint8_t VADC_IN = 1;

    // Actual pin number of the battery voltage output pin enabler.
    constexpr uint8_t ADC_Ctrl = 37;

    // Value is pulled from the datasheet for the Heltec WiFi LoRa 32 V3.2
    // https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA_V3.2.pdf
    // Page 9, footprint note 4
    // VADC = VBAT * 100 / (100 + 390)
    // VBAT = VADC * (390 + 100) / 100
    constexpr float BATTERY_DIVIDER = (390.0f + 100.0f) / 100.0f;

    // This is a user set value. To get accurate battery readings, you'll
    // need to do the following:
    // 1. Set this value to 1, then run the ESP32.
    // 2. Get the measured battery voltage.
    // 3. Use a multimeter to measure the actual battery voltage.
    // BATTERY_CALIBRATION = ACTUAL / ESP_MEASURED
    constexpr float BATTERY_CALIBRATION = 3.4f / 3.3f;

    constexpr int BATTERY_READ_SAMPLES = 16;

    constexpr Point CURVE[] = {
        {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.92f, 70},
        {3.85f, 60},  {3.79f, 50}, {3.75f, 40}, {3.70f, 30},
        {3.60f, 20},  {3.50f, 10}, {3.30f, 0},
    };

    constexpr size_t CURVE_SIZE = sizeof(CURVE) / sizeof(CURVE[0]);
}

//----------------------------------------------------------------------------
// Battery Public namespace
//----------------------------------------------------------------------------
namespace Battery
{
    //------------------------------------------------------------------------
    void InitializeBattery()
    {
        analogReadResolution(12);
        pinMode(ADC_Ctrl, OUTPUT);
        digitalWrite(ADC_Ctrl, HIGH);
        delay(10);
        analogSetPinAttenuation(VADC_IN, ADC_11db);
        pinMode(VADC_IN, INPUT);
    }

    //------------------------------------------------------------------------
    bool IsConnected()
    {
        return DEVICE_HAS_BATTERY;
    }

    //------------------------------------------------------------------------
    float ReadVoltage()
    {
        uint32_t millivolts = 0;

        for (int i = 0; i < BATTERY_READ_SAMPLES; i++)
        {
            millivolts += analogReadMilliVolts(VADC_IN);
            delay(2);
        }

        const float adcVoltage = (millivolts / static_cast<float>(BATTERY_READ_SAMPLES)) / 1000.0f;
        return adcVoltage * BATTERY_DIVIDER * BATTERY_CALIBRATION;
    }

    //------------------------------------------------------------------------
    int Percentage(float voltage)
    {
        if (voltage >= CURVE[0].volts)
        {
            return 100;
        }
        if (voltage <= CURVE[CURVE_SIZE - 1].volts)
        {
            return 0;
        }

        for (size_t i = 0; i < CURVE_SIZE - 1; i++)
        {
            const Point high = CURVE[i];
            const Point low  = CURVE[i + 1];

            // Found the two values the voltage lies between. Perform basic interpolation.
            if (voltage <= high.volts && voltage >= low.volts)
            {
                const float range = high.volts - low.volts;
                const float fraction = (voltage - low.volts) / range;

                // Convert to int, while rounding up.
                const int percent = low.percent + static_cast<int>((high.percent - low.percent) * fraction + 0.5f);

                return percent;
            }
        }

        // Should never get to this point, but return a value anyways.
        return 0;
    }
}
