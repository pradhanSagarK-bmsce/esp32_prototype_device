#include "PulseAlgorithm.h"
#include <math.h>

// ===== External variables =====

int32_t irBuffer[BUFFER_SIZE];
int32_t redBuffer[BUFFER_SIZE];
bool bufferFull = false;
int bufferIdx = 0;


// ===== Internal state =====
static float BPM = 0;
static float SpO2 = 0;

void resetPulseBuffers() {
    memset(irBuffer, 0, sizeof(irBuffer));
    memset(redBuffer, 0, sizeof(redBuffer));
    bufferFull = false;
    BPM = 0;
    SpO2 = 0;
}

// ===== Calculate BPM =====
float calculateBPM() {
    if (!bufferFull) return BPM;

    float signal[BUFFER_SIZE];
    float mean = 0;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        signal[i] = irBuffer[i];
        mean += signal[i];
    }
    mean /= BUFFER_SIZE;

    float stdDev = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        signal[i] -= mean;
        stdDev += signal[i] * signal[i];
    }
    stdDev = sqrt(stdDev / BUFFER_SIZE);
    if (stdDev < 10) return BPM;

    for (int i = 0; i < BUFFER_SIZE; i++) signal[i] /= stdDev;

    int peaks[30];
    int peakCount = 0;
    float threshold = 0.5;
    int minPeakDistance = 40; // 0.4 s → ~150 BPM max

    for (int i = 5; i < BUFFER_SIZE - 5 && peakCount < 30; i++) {
        if (signal[i] > threshold &&
            signal[i] > signal[i-1] && signal[i] > signal[i-2] &&
            signal[i] > signal[i-3] && signal[i] > signal[i-4] &&
            signal[i] >= signal[i+1] && signal[i] >= signal[i+2] &&
            signal[i] >= signal[i+3] && signal[i] >= signal[i+4]) {
            if (peakCount == 0 || (i - peaks[peakCount-1]) >= minPeakDistance)
                peaks[peakCount++] = i;
        }
    }

    if (peakCount < 3) return BPM;

    float intervals[29];
    int intervalCount = 0;

    for (int i = 1; i < peakCount; i++) {
        float interval = peaks[i] - peaks[i - 1];
        float bpmFromInterval = 6000.0 / interval;
        if (bpmFromInterval >= 45 && bpmFromInterval <= 120)
            intervals[intervalCount++] = interval;
    }

    if (intervalCount < 2) return BPM;

    // median sort
    for (int i = 0; i < intervalCount - 1; i++) {
        for (int j = 0; j < intervalCount - i - 1; j++) {
            if (intervals[j] > intervals[j + 1]) {
                float t = intervals[j];
                intervals[j] = intervals[j + 1];
                intervals[j + 1] = t;
            }
        }
    }

    float medianInterval = intervals[intervalCount / 2];
    float newBPM = 6000.0 / medianInterval;

    if (BPM > 0 && BPM >= 50 && BPM <= 110)
        newBPM = BPM * 0.75 + newBPM * 0.25;

    if (newBPM < 50 || newBPM > 110) return BPM;

    BPM = newBPM;
    return BPM;
}

// ===== Calculate SpO2 =====
float calculateSpO2() {
    if (!bufferFull) return SpO2;

    const int samples = 200;
    float redDC = 0, irDC = 0;
    float redMin = 999999, redMax = 0;
    float irMin = 999999, irMax = 0;

    int startIdx = 0;
    extern int bufferIdx; // if needed you can expose bufferIdx globally

    startIdx = (bufferIdx - samples + BUFFER_SIZE) % BUFFER_SIZE;

    for (int i = 0; i < samples; i++) {
        int idx = (startIdx + i) % BUFFER_SIZE;
        redDC += redBuffer[idx];
        irDC += irBuffer[idx];
        if (redBuffer[idx] < redMin) redMin = redBuffer[idx];
        if (redBuffer[idx] > redMax) redMax = redBuffer[idx];
        if (irBuffer[idx] < irMin) irMin = irBuffer[idx];
        if (irBuffer[idx] > irMax) irMax = irBuffer[idx];
    }

    redDC /= samples;
    irDC /= samples;

    float redAC = (redMax - redMin) / 2.0;
    float irAC = (irMax - irMin) / 2.0;

    if (irAC == 0 || irDC == 0 || redDC == 0) return SpO2;

    float R = (redAC / redDC) / (irAC / irDC);
    float newSpO2 = 110 - 25 * R;

    if (SpO2 > 0 && SpO2 >= 90 && SpO2 <= 100)
        newSpO2 = SpO2 * 0.8 + newSpO2 * 0.2;

    newSpO2 = constrain(newSpO2, 90, 100);
    SpO2 = newSpO2;
    return SpO2;
}
