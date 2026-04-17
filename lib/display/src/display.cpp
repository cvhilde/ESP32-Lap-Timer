///===========================================================================
///
/// display.cpp
///
/// Implements OLED display initialization and runtime status display logic.
/// This should be the only file that interacts with the ESP32 display.
///
///===========================================================================

#include <display.h>
#include "HT_SSD1306Wire.h"
#include <Wire.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    // Private declaration of the display object used to interface with the
    // actual onboard OLED display.
    SSD1306Wire _display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

    // Pin for the display power.
    constexpr unsigned ADC_CTRL = 37;

    // Initial setup flag.
    bool initialized = false;

    //------------------------------------------------------------------------
    // give power to the display
    void VextON()
    {
        pinMode(Vext,OUTPUT);
        digitalWrite(Vext, LOW);
    }

    //------------------------------------------------------------------------
    // Draw the permanent features on the display
    void permDraws()
    {
        _display.drawXbm(120, 4, 8, 8, Display::satelliteBitmap);
        _display.display();
    }
}

namespace Display
{
    //------------------------------------------------------------------------
    bool InitializeDisplay()
    {
        VextON();
        delay(100);
        pinMode(ADC_CTRL, OUTPUT);
        digitalWrite(ADC_CTRL, LOW);
        analogReadResolution(12);
        initialized = _display.init();

        if (initialized)
        {
            _display.setFont(Display::Roboto_Light_14);
            _display.drawString(34, 4, "Lap Timer");
            _display.drawString(58, 30, "By");
            _display.drawString(7, 48, "Carter Hildebrandt");
            _display.display();
            delay(5000);
        }

        return initialized;
    }

    //------------------------------------------------------------------------
	void DrawStatusScreen(const GPS::FixData& data)
    {
        char line1[30];
        String gpsStatus = "";
        int fixType = data.fixType;

        if (fixType == 0 || fixType == 1)
            gpsStatus = "No Fix";
        else if (fixType == 2)
            gpsStatus = "Poor";
        else if (fixType == 3)
            gpsStatus = "Good";
        else if (fixType > 3)
            gpsStatus = "Great";

        sprintf(line1, "GPS: %s", gpsStatus.c_str());

        char subLine1[10];
        sprintf(subLine1, "%d", data.satelliteCount);

        char line2[30];
        sprintf(line2, "Storage: %.2f%%", storageUsage());

        char line3[30];
        String bleStatus = "";

        if (isAdvertising()) {
            bleStatus = "Advertising";
        } else if (isConnected()) {
            bleStatus = "Connected";
        } else {
            bleStatus = "Idle";
        }
        sprintf(line3, "BLE: %s", bleStatus.c_str());

        _display.setColor(BLACK);
        _display.fillRect(0, 48, 128, 16);

        if (isSending()) {
            int totalFiles = getFileCount();
            int currentFile = getCurrentFileNumber();

            char line4[30];
            sprintf(line4, "Transfer File %d/%d", currentFile, totalFiles);

            _display.setColor(WHITE);
            _display.drawString(0, 48, line4);
        }  

        _display.setColor(BLACK);
        _display.fillRect(0, 0, 80, 16);
        _display.fillRect(90, 0, 30, 16);
        _display.fillRect(0, 16, 100, 16);
        _display.fillRect(0, 32, 100, 16);
        _display.setColor(WHITE);
        _display.drawString(0, 0, line1);
        _display.drawString(0, 16, line2);
        _display.drawString(0, 32, line3);

        _display.setTextAlignment(TEXT_ALIGN_RIGHT);
        _display.drawString(120, 0, subLine1);
        _display.setTextAlignment(TEXT_ALIGN_LEFT);
        
        _display.display();
    }

    //------------------------------------------------------------------------
	void DrawCurrentMode(Mode_t mode)
    {
        char line[2];

        _display.setColor(BLACK);
        _display.fillRect(100, 16, 28, 16);
        _display.setColor(WHITE);

        if (mode == LAP_TIMING)
        {
            sprintf(line, "L");
            _display.setTextAlignment(TEXT_ALIGN_RIGHT);
            _display.drawString(120, 16, line);
            _display.setTextAlignment(TEXT_ALIGN_LEFT);
            _display.drawCircle(116, 24, 8);
        }
        else if (mode == ROUTE_TRACKING)
        {
            sprintf(line, "R");
            _display.fillCircle(116, 24, 8);
            _display.setColor(BLACK);
            _display.setTextAlignment(TEXT_ALIGN_RIGHT);
            _display.drawString(121, 16, line);
            _display.setTextAlignment(TEXT_ALIGN_LEFT);
            _display.setColor(WHITE);
        }

        _display.display();
    }

    //------------------------------------------------------------------------
	void DisplayPurgingMessage()
    {
        char line[] = "Purging Flash";
        _display.setColor(BLACK);
        _display.fillRect(0, 48, 128, 16);
        _display.setColor(WHITE);
        _display.drawString(0, 48, line);
        _display.display();
    }

    //------------------------------------------------------------------------
	void ClearPurgingMessage()
    {
        _display.setColor(BLACK);
        _display.fillRect(0, 48, 128, 16);
        _display.display();
    }

    //------------------------------------------------------------------------
	void DisplayGettingMessage()
    {
        char line[] = "Getting Waypoints";
        _display.setColor(BLACK);
        _display.fillRect(0, 48, 128, 16);
        _display.setColor(WHITE);
        _display.drawString(0, 48, line);
        _display.display();
    }

    //------------------------------------------------------------------------
	void ClearGettingMessage()
    {
        _display.setColor(BLACK);
        _display.fillRect(0, 48, 128, 16);
        _display.display();
    }
}
