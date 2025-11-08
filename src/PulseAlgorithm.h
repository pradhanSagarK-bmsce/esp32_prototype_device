#ifndef PULSE_ALGORITHM_H
#define PULSE_ALGORITHM_H

#include <Arduino.h>

// ===== Constants =====
#define BUFFER_SIZE 400

// ===== External buffers (you fill these from main sketch) =====
extern int32_t irBuffer[BUFFER_SIZE];
extern int32_t redBuffer[BUFFER_SIZE];
extern bool bufferFull;
extern int bufferIdx;

// ===== Public API =====
float calculateBPM();
float calculateSpO2();

// Optional: helper to reset buffers (if needed)
void resetPulseBuffers();

#endif
