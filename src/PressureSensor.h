#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

/* ─── Pin assignments ──────────────────────────────────────────────────── */
#define PRESSCH1 5          // inlet-side sensor
#define PRESSCH2 6          // outlet-side sensor

/* ─── Moving-average filter (raw ADC counts) ───────────────────────────── */
const int  WINDOW_SIZE = 10;            // larger window → smoother trace
static uint16_t readings1[WINDOW_SIZE];
static uint16_t readings2[WINDOW_SIZE];
static int      readIndex1 = 0;         // Separate index for channel 1
static int      readIndex2 = 0;         // Separate index for channel 2

/* ─── 4–20 mA → kPa conversion constants ──────────────────────────────── */
constexpr float ADC_VREF              = 3.3f;           // ESP32 reference
constexpr float PRESS_OFFSET_V        = 0.6f;           // 4 mA × 150 Ω
constexpr float PRESS_FS_SPAN_V       = 2.4f;           // (20 mA – 4 mA) × 150 Ω
constexpr float PRESS_MAX_KPA         = 500.0f;         // transmitter full-scale
constexpr float PRESS_SCALE_KPA_PER_V = PRESS_MAX_KPA / PRESS_FS_SPAN_V; // 208.333 kPa / V

/* ─── Helpers ──────────────────────────────────────────────────────────── */
inline uint16_t applyMovingAverage(uint16_t newReading, uint16_t *buffer, int &index)
{
    buffer[index] = newReading;
    uint32_t sum = 0;
    for (int i = 0; i < WINDOW_SIZE; ++i) sum += buffer[i];
    index = (index + 1) % WINDOW_SIZE;
    return sum / WINDOW_SIZE;
}

inline float adcToKpa(uint16_t adcCounts)
{
    float volts = (ADC_VREF * adcCounts) / 4095.0f;
    float kPa   = PRESS_SCALE_KPA_PER_V * (volts - PRESS_OFFSET_V);
    return constrain(kPa, 0.0f, PRESS_MAX_KPA);        // clamp noise outside 0-500 kPa
}

/* ─── Public API ───────────────────────────────────────────────────────── */
/* Initialisation */
inline void init_pressure_ch1() { pinMode(PRESSCH1, ANALOG); }
inline void init_pressure_ch2() { pinMode(PRESSCH2, ANALOG); }

/* Raw (smoothed) ADC counts — legacy functions kept for compatibility */
inline uint16_t readPressureRaw_ch1() { return applyMovingAverage(analogRead(PRESSCH1), readings1, readIndex1); }
inline uint16_t readPressureRaw_ch2() { return applyMovingAverage(analogRead(PRESSCH2), readings2, readIndex2); }

/* NEW: direct kPa readers — call these from your main code */
inline float readPressureKpa_ch1() { return adcToKpa(readPressureRaw_ch1()); }
inline float readPressureKpa_ch2() { return adcToKpa(readPressureRaw_ch2()); }

/* Optional: reset both moving-average buffers */
inline void resetPressureFilters()
{
    readIndex1 = 0;
    readIndex2 = 0;
    memset(readings1, 0, sizeof(readings1));
    memset(readings2, 0, sizeof(readings2));
}

#endif  // PRESSURE_SENSOR_H