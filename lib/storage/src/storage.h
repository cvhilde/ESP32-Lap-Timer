#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>

void startSession();
void initStorage();
void writeToLogFile(double lat, double lng, double speed);
void writeToTimeLog(unsigned long lapTime, unsigned long sector1, unsigned long sector2, unsigned long sector3);
void endSession();
double storageUsage();
void startRouteSession();
void writeToRouteLog(double lat, double lng, double speed, double altitude);
void endRouteSession();

#endif