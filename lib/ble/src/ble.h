#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SPIFFS.h>
#include <esp_rom_crc.h>
#include <freertos/queue.h>
#include <vector>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

void initBLE();
void startAdvertising();
void stopAdvertising();
bool isConnected();

void BLE_loop();
uint32_t crc32_file(File &f);
String base64_encode(const uint8_t *in, size_t len);
void txLine(const String &s);
uint32_t sendChunk();

extern bool bleConnected;
extern bool bleAdvertising;

#endif