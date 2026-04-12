# ESP32 Lap Timer

## About this project

My goal with this project was to make it something that could easily be introduced to any racing platform,
specifically dirt bikes, and act as a standalone lap timer for incredibly basic data.
I'm currently developing it to work only with the Heltec Lora Wifi 32 Kit, because of its built in
OLED screen and bluetooth. I'm also using the u-blox NEO M9N GPS thanks to its +/- 1 m horizontal
accuracy and 25 Hz polling capacity. Currently this repo is only housing the firmware for the
ESP32. The app for IOS will be in it's own repo.

## What's done?
- Sector/Lap timing logic (waypoint crossing and storing the times)
- All logic associated with GPS functionality
- Creating the sesssion logs and writing a running a list of positions
- Draws a information screen on the display. Shows GPS connectivity, SIV, Storage usage, BLE Status, file transfer status, and the current mode
- Writing a running list of lap times to the flash
- Functionality to download log files to a pc over bluetooth
- Purges all log files in flash when uploaded
- Added a way to switch to route tracking mode. This reduces the loop clock rate to 5 Hz for longer seession storage limits.
- Bluetooth upload path for uploading a new waypoints.json file to the esp. This file persists across reboots and flash purges.

## Why the V2?
My original github repo for this project was very messy. I started creating this project
when I had very little C/C++ knowledge. That is quite evident when you go look at the first few commits
for the project. I never really had a clear goal in mind for what I wanted the project to be,
as seen with the entire design changes to the screen and such throughout the project. There is also a lot of
unused code in the repo. Now that I have experience as a software engineer, I wanted to revisit the code and
clean up everything to be more maintainable. To that end, I have copied over the previous V1 repo and will slowly begin
cleaning it up and making the logic flow a lot easier to follow.

## Current status
The Lap Timer has been mounted to my dirt bike and has already had real world testing on a track. It performs great
and the lap times are quite accurate, even with all the acceleration jitter that comes naturally from the dirt bike.
Now that my brother wants his own lap timer, I thought this would be a good time to also teach him the ropes of how
git branches and pull requests work. As work progresses, I will update this readme with fix versions detailing major
changes. In the future when I start to design/create the second laptimer, I may add more information on how I made
the actual housing box and the way it was mounted on the dirt bike.


![screen demo](https://media0.giphy.com/media/v1.Y2lkPTc5MGI3NjExNTYzaXUzNzYzajUxbDU1MzN1cmloYzI1cXljNjNwcHM1cnJpcTJzbCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/QZPMouXupVzR3BWeDb/giphy.gif)
