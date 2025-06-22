# ESP32 Lap Timer

## About this project

My goal with this project was to make it something that could easily be introduced to any racing platform,
specifically dirt bikes, and act as a standalone lap timer for incredibly basic data.
I'm currently developing it to work only with the Heltec Lora Wifi 32 Kit, because of its built in
OLED screen and bluetooth. I'm also using the u-blox NEO M9N GPS thanks to its +/- 1 m horizontal
accuracy and 25 Hz polling capacity. Currently this repo is only housing the firmware for the
ESP32 and once I begin building the app for the IOS and Android, I'll include those in their own repos.

## What's done?
- Sector/Lap timing logic (waypoint crossing and storing the times)
- All logic associated with GPS functionality
- Creating the sesssion logs and writing a running a list of positions
- Drawing the sector/lap times to the display
- Writing a running list of lap times to the flash

## To do list
- Possibly change display logic to instead only feature a status screen (voltage, satellite strength, bluetooth status, etc)
- Implement all bluetooth logic
- Implement a way to clear all stored data in flash when uploaded over bluetooth

![screen demo](https://media0.giphy.com/media/v1.Y2lkPTc5MGI3NjExNTYzaXUzNzYzajUxbDU1MzN1cmloYzI1cXljNjNwcHM1cnJpcTJzbCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/QZPMouXupVzR3BWeDb/giphy.gif)
