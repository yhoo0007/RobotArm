#ifndef PULSE_TIMER_H
#define PULSE_TIMER_H

#include <stdint.h>


void onPulseTimer();
void setupPulseTimer();
void pulseTimerAttachPin(int pin, int id);
void registerPulseTarget(int channel, int target);
void setFrequency(int channel, uint16_t frequency);
bool pulseTimerIsRunning();

#endif