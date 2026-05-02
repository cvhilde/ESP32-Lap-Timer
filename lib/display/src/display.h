#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <gps.h>
#include <storage.h>
#include <ble.h>

namespace Display
{
	struct StatusSnapshot
	{
		GPS::FixData fixData;
		Storage::SessionType mode;
		double storageUsage;
		BLE::Status bleStatus;

		StatusSnapshot(
			const GPS::FixData& fixData_val,
			Storage::SessionType mode_val,
			double storageUsage_val,
			BLE::Status bleStatus_val
		) :
			fixData(fixData_val),
			mode(mode_val),
			storageUsage(storageUsage_val),
			bleStatus(bleStatus_val)
		{}

		StatusSnapshot() :
			mode(Storage::DEFAULT_SESSION_TYPE),
			storageUsage(0.0)
		{}
	};

    // Initializes the display while also drawing the basic sector times.
	// Returns a boolean for whether or no the display was properly initialized.
	bool InitializeDisplay();

	// Draws the status screen based.
	void UpdateScreen(const StatusSnapshot& status);

	// During purging of the flash, this will draw the "Purging Flash" message.
	// This will not clear the message when purging is complete.
	void DisplayPurgingMessage();

	// Clears the "Purging Flash" message.
	void ClearPurgingMessage();

	// When getting a new Waypoints file from BLE, this should be called to
	// display the "Getting Waypoints" message. This will not clear the message
	// when getting the file is complete.
	void DisplayGettingMessage();

	// Clears the "Getting Waypoints" message.
	void ClearGettingMessage();
};

#endif
