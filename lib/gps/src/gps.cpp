///===========================================================================
///
/// This file is specifically for interfacing with the U-BLOX NEO M9N GPS
/// module and all related logic to it. This is the only file that should be
/// directly calling functions from the gps declaration. Any other file that
/// needs data from the GPS should have a helper function within this file.
///
///===========================================================================

#include <gps.h>

//----------------------------------------------------------------------------
// Forward declarations
//----------------------------------------------------------------------------
SFE_UBLOX_GNSS GPS::_gps;
HardwareSerial GPS::_GPSHardwareSerial(1);

const double GPS::MM_S_TO_MPH(0.00223694);
const double GPS::MM_TO_FEET(0.00328084);
const double GPS::LAT_LONG_TO_DEGREES(0.0000001);

//----------------------------------------------------------------------------
void GPS::InitializeGPS()
{
    _GPSHardwareSerial.begin(115200, SERIAL_8N1, GPS_Rx, GPS_Tx);
    delay(1000);
    
    _gps.begin(_GPSHardwareSerial);
    delay(1000);

    for (int i = 0; i < 2; i++) {
        activeLocations[i].lat = 0;
        activeLocations[i].lng = 0;
    }
}

//----------------------------------------------------------------------------
double GPS::Latitude()
{
    return _gps.getLatitude() * LAT_LONG_TO_DEGREES;
}

//----------------------------------------------------------------------------
double GPS::Longitude()
{
    return _gps.getLongitude() * LAT_LONG_TO_DEGREES;
}

//----------------------------------------------------------------------------
int GPS::SatelliteCount()
{
    return _gps.getSIV();
}

//----------------------------------------------------------------------------
double GPS::Speed()
{
    return _gps.getGroundSpeed() * MM_S_TO_MPH;
}

//----------------------------------------------------------------------------
double GPS::GPSAltitude()
{
    return _gps.getAltitudeMSL() * MM_TO_FEET;
}

//----------------------------------------------------------------------------
uint8_t GPS::FixType()
{
    return _gps.getFixType();
}

//----------------------------------------------------------------------------
void GPS::UpdateUBLOX()
{
    _gps.checkUblox();
}

//----------------------------------------------------------------------------
GPS::Time GPS::GPSTime()
{
    GPS::Time time;

    time.year = _gps.getYear();
    time.month = _gps.getMonth();
    time.day = _gps.getDay();
    time.hour = _gps.getHour() - 4; // timezone difference
    time.hour = (time.hour + 24) % 24;
    time.minute = _gps.getMinute();
    time.second = _gps.getSecond();
    time.valid = true;

    return time;
}
