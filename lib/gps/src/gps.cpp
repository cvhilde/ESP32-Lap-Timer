#include <gps.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>
#include <globals.h>

SFE_UBLOX_GNSS gps;
HardwareSerial GPS(1);

// gps module initialization
// using hardcoded waypoint markers for testing
void initGPS() {
    GPS.begin(115200, SERIAL_8N1, 19, 20);
    delay(1000);
    
    gps.begin(GPS);
    delay(1000);

    for (int i = 0; i < 2; i++) {
        activeLocations[i].lat = 0;
        activeLocations[i].lng = 0;
    }

    gps.setUART1Output(COM_TYPE_UBX);
    gps.setAutoESFRAW(true);
}

double getLatitude() {
    return gps.getLatitude() / 10000000.0;
}

double getLongitude() {
    return gps.getLongitude() / 10000000.0;
}

int getSatCount() {
    return gps.getSIV();
}

double getSpeed() {
    return gps.getGroundSpeed() * 0.00223694;
}

double getAltitude() {
    double alt = gps.getAltitude();
    double sep = gps.getGeoidSeparation();
    return (alt - sep) / 304.8;
}

AirborneData readAirborneState() {
    AirborneData out;

    bool haveX = false, haveY = false, haveZ = false;
    float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;

    uint8_t numBlocks = gps.packetUBXESFRAW->data.numEsfRawBlocks;

    for (uint8_t i = 0; i < numBlocks; i++)
    {
        auto &blk = gps.packetUBXESFRAW->data.data[i];

        uint8_t dataType = blk.data.bits.dataType;
        int32_t raw = blk.data.bits.dataField;

        if (raw & 0x00800000)
            raw |= 0xFF000000;

        // u-blox ESF-RAW accel scaling: m/s^2 * 2^-10
        float val_mps2 = raw / 1024.0f;
        float val_g = val_mps2 / 9.80665f;

        if (dataType == 16) { ax_g = val_g; haveX = true; } // X accel
        if (dataType == 17) { ay_g = val_g; haveY = true; } // Y accel
        if (dataType == 18) { az_g = val_g; haveZ = true; } // Z accel
    }

    if (!(haveX && haveY && haveZ))
        return out;

    out.valid = true;
    out.ax_g = ax_g;
    out.ay_g = ay_g;
    out.az_g = az_g;
    out.mag_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    Serial.printf("X: %.4f | Y: %.5f | Z: %.5f | Mag: %.5f\n", ax_g, ay_g, az_g, out.mag_g);

    out.airborne = (out.mag_g <= 0.3f);

    return out;
}

bool getAirborneState() {
    const float threshold_g = 0.3f;
    bool airborne = false;

    AirborneData d = readAirborneState();

    if (d.valid) {
        airborne = d.mag_g <= threshold_g;
    }

    return airborne;
}