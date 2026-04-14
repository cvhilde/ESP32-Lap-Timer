///===========================================================================
///
/// display.cpp
///
/// TODO: Implement file description
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
    SSD1306Wire _display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

    constexpr unsigned ADC_CTRL = 37;

    //------------------------------------------------------------------------
    // initializes the display while alos drawing the basic sector times
    void initDisplay()
    {
        VextON();
        delay(100);
        pinMode(ADC_CTRL, OUTPUT);
        digitalWrite(ADC_CTRL, LOW);
        analogReadResolution(12);
        display.init();
        display.setFont(Display::Roboto_Light_14);
        display.drawString(34, 4, "Lap Timer");
        display.drawString(58, 30, "By");
        display.drawString(7, 48, "Carter Hildebrandt");
        display.display();
        delay(5000);
    }

    //------------------------------------------------------------------------
    // give power to the display
    void VextON()
    {
        pinMode(Vext,OUTPUT);
        digitalWrite(Vext, LOW);
    }

    //------------------------------------------------------------------------
    // Draw the permenant strings on the display
    void permDraws()
    {
        display.drawXbm(120, 4, 8, 8, Display::satelliteBitmap);
        display.display();
    }
}

namespace Display
{
    //------------------------------------------------------------------------
	bool InitializeDisplay()
    {

    }

    //------------------------------------------------------------------------
	void DrawStatusScreen(const GPS::FixData& data)
    {

    }

    //------------------------------------------------------------------------
	void DrawCurrentMode(bool mode)
    {

    }

    //------------------------------------------------------------------------
	void DisplayPurgingMessage()
    {

    }

    //------------------------------------------------------------------------
	void ClearPurgingMessage()
    {

    }

    //------------------------------------------------------------------------
	void DisplayGettingMessage()
    {

    }

    //------------------------------------------------------------------------
	void ClearGettingMessage()
    {

    }
}

// give power to the display
void VextON() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
}

void permDraws() {
    display.drawXbm(120, 4, 8, 8, Display::satelliteBitmap);
    display.display();
}

void drawCurrentMode(bool current) {
    char line[5];
    if (!current) {
        sprintf(line, "L");
        display.setColor(BLACK);
        display.fillRect(100, 16, 28, 16);
        display.setColor(WHITE);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(120, 16, line);
        display.setTextAlignment(TEXT_ALIGN_LEFT);

        display.drawCircle(116, 24, 8);;

        display.display();

    } else {
        sprintf(line, "R");

        display.setColor(BLACK);
        display.fillRect(100, 16, 28, 16);
        display.setColor(WHITE);

        display.fillCircle(116, 24, 8);
        display.setColor(BLACK);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(121, 16, line);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setColor(WHITE);

        display.display();
    }
}

void drawStatusScreen() {
    char line1[30];
    String gpsStatus = "";

    int fixType = GPS::FixType();
    if (fixType == 0 || fixType == 1) {
        gpsStatus = "No Fix";
    } else if (fixType == 2) {
        gpsStatus = "Poor";
    } else if (fixType == 3) {
        gpsStatus = "Good";
    } else if (fixType > 3) {
        gpsStatus = "Great";
    }
    sprintf(line1, "GPS: %s", gpsStatus.c_str());

    char subLine1[10];
    sprintf(subLine1, "%d", GPS::SatelliteCount());

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

    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);

    if (isSending()) {
        int totalFiles = getFileCount();
        int currentFile = getCurrentFileNumber();

        char line4[30];
        sprintf(line4, "Transfer File %d/%d", currentFile, totalFiles);

        display.setColor(WHITE);
        display.drawString(0, 48, line4);
    }  

    display.setColor(BLACK);
    display.fillRect(0, 0, 80, 16);
    display.fillRect(90, 0, 30, 16);
    display.fillRect(0, 16, 100, 16);
    display.fillRect(0, 32, 100, 16);
    display.setColor(WHITE);
    display.drawString(0, 0, line1);
    display.drawString(0, 16, line2);
    display.drawString(0, 32, line3);

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(120, 0, subLine1);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    
    display.display();
}

void displayPurgingMessage() {
    char line[] = "Purging Flash";
    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);
    display.setColor(WHITE);
    display.drawString(0, 48, line);
    display.display();
}

void clearPurgingMessage() {
    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);
    display.display();
}

void displayGettingMessage() {
    char line[] = "Getting Waypoints";
    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);
    display.setColor(WHITE);
    display.drawString(0, 48, line);
    display.display();
}

void clearGettingMessage() {
    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);
    display.display();
}
