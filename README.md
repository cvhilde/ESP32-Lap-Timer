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
- Draws a information screen on the display. Shows GPS connectivity, SIV, Storage usage, BLE Status, file transfer status, and the current mode
- Writing a running list of lap times to the flash
- Basic functionality to download log files to a pc over bluetooth
- Purges all log files in flash when uploaded
- Added a way to switch to route tracking mode. This reduces the loop clock rate to 5 Hz for longer seession storage limits.

## To do list
- Add a way to upload a json file with waypoints for lap tracking
- Add LED logic to signal if a session start fails
- Add a new csv file entry for route tracking with possibly more information and so no timestamp file will be created.
- Update main loop logic to exclude the execution of waypoint logic when in route tracking mode.

![screen demo](https://media0.giphy.com/media/v1.Y2lkPTc5MGI3NjExNTYzaXUzNzYzajUxbDU1MzN1cmloYzI1cXljNjNwcHM1cnJpcTJzbCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/QZPMouXupVzR3BWeDb/giphy.gif)
