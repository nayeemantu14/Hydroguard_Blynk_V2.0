#ifndef ADVANCED_BLOCKAGE_DETECTOR_H
#define ADVANCED_BLOCKAGE_DETECTOR_H

#include <Arduino.h>
#include "debug.h"
#include "KalmanFilter.h"
#include <math.h>
#include <EEPROM.h>

class AdvancedBlockageDetector
{
private:
    // EEPROM addresses
    static constexpr int EEPROM_BASE_ADDR = 100; // Starting address for filter data
    static constexpr int EEPROM_CALIBRATED_FLAG = EEPROM_BASE_ADDR;
    static constexpr int EEPROM_BASELINE_RESISTANCE = EEPROM_BASE_ADDR + 4;
    static constexpr int EEPROM_LAST_CALIBRATION = EEPROM_BASE_ADDR + 8;

    // Configuration constants
    static constexpr uint16_t HISTORY_SIZE = 100;
    static constexpr float FLOW_THRESHOLD = 2.0;   // Minimum reliable flow (L/min)
    static constexpr float MAX_FLOW = 60.0;        // Maximum expected flow (L/min)
    static constexpr float PRESSURE_RANGE = 600.0; // kPa

    // Default pressure-flow curve coefficients
    static constexpr float DEFAULT_QUADRATIC_COEFF = 0.0178f;
    static constexpr float DEFAULT_LINEAR_COEFF = 1.1692f;
    static constexpr float DEFAULT_CONSTANT_COEFF = -0.3355f;

    // Learning parameters
    float quadraticCoeff;
    float linearCoeff;
    float constantCoeff;
    static constexpr uint8_t MIN_LEARNING_POINTS = 3;
    uint8_t learningPoints;
    float learningFlows[10];
    float learningPressures[10];

    // Circular buffers
    float pressureDiffBuffer[HISTORY_SIZE];
    float flowRateBuffer[HISTORY_SIZE];
    uint8_t bufferIndex = 0;
    bool bufferInitialized = false;

    // System state
    float baselineResistance = 0.0;
    float adaptiveThreshold = 35.0;
    uint32_t lastCalibrationTime = 0;
    float sensorVariance = 0.0;
    float lastValidBlockage = 0.0;

    // Filters
    KalmanFilter pressureFilter;
    KalmanFilter flowFilter;

    // Private methods
    float calculateResistance(float pressureDiff, float flowRate) const;
    float calculateTrend();
    void updateVariance(float pressureDiff, float flowRate);
    bool validateReadings(float pressureDiff, float flowRate);

public:
    struct BlockageStatus
    {
        float blockagePercentage = 0.0;
        float confidence = 0.0;
        bool requiresAttention = false;
        String message = "System Initializing";
    };

    AdvancedBlockageDetector();

    void calibrate(float pressureDiff, float flowRate);
    BlockageStatus update(float inletPressure, float outletPressure, float flowRate);
    float getCurrentResistance() const;
    void reset();
    bool loadCalibration();
    bool saveCalibration();
    bool isCalibrated() const;
    uint32_t getLastCalibrationTime() const;
    float calculateExpectedPressure(float flowRate) const;
    void learnFromData(float pressureDiff, float flowRate);
};

#endif