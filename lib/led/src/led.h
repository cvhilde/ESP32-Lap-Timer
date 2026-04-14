#ifndef LED_H
#define LED_H

namespace Led
{
    // Start a blink with even intervals (in ms)
    void StartBlink(unsigned long internval);

    // Stop blinking
    void StopBlink();

    // Turns the LED on permenantly until TurnLedOff() is called
    void TurnLedOn();

    // Turns the LED off. Does not stop blinking.
    void TurnLedOff();

    // Start a blink with intervals of onDuration (in ms) and offDuration (in ms)
    void StartBlink(unsigned long onDuration, unsigned long offDuration);

    // Start a single instance of how long to blink for (duration in ms), where
    // each blink is spaced by interval (in ms). Does not require the use of
    // StopBlink() to stop the blinking, as it will finish once duration passes.
    void StartOneShotBlink(unsigned long interval, unsigned long duration);
};

#endif
