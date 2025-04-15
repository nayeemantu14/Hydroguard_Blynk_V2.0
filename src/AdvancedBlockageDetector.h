
#ifndef ADVANCED_BLOCKAGE_DETECTOR_H
#define ADVANCED_BLOCKAGE_DETECTOR_H

#include <string>
#include <vector>
#include <numeric>
#include <Arduino.h>

class AdvancedBlockageDetector {
public:    
    void reset() {
        lastSmoothedPressure = 0.0f;
        wasAboveThreshold = false;
        pressureHistory.clear();
        blockageHistory.clear();
    }

    struct BlockageStatus {
        float blockagePercentage;
        bool requiresAttention;
        std::string message;
        float smoothedPressure;

        BlockageStatus() : blockagePercentage(0.0f), requiresAttention(false), 
                          message("System Initializing"), smoothedPressure(0.0f) {}
    };

    BlockageStatus update(float inletPressure, float outletPressure, float flowRate, float temperature = 20.0f);

private:
    // Minimum flow rate required for reliable measurements
    static constexpr float MINIMUM_FLOW_THRESHOLD = 1.0f;
    // Threshold percentage for triggering blockage attention
    static constexpr float BLOCKAGE_ATTENTION_THRESHOLD = 30.0f;
    // Hysteresis band to prevent oscillation around threshold
    static constexpr float HYSTERESIS = 8.0f;
    // Size of moving average window for smoothing
    static constexpr size_t MOVING_AVG_SIZE = 50;
    // Minimum pressure difference to consider valid (kPa)
    static constexpr float MINIMUM_PRESSURE_DIFF = 0.1f;
    // Maximum allowable pressure difference (kPa)
    static constexpr float MAXIMUM_PRESSURE_DIFF = 100.0f;
    
    float lastSmoothedPressure = 0.0f;
    std::vector<float> pressureHistory;
    std::vector<float> blockageHistory;
    bool wasAboveThreshold = false;

    float calculateExpectedPressure(float flowRate, float temperature) const;
    float applyExponentialSmoothing(float newValue, float lastValue) const;
    std::string determineBlockageMessage(float blockagePercentage) const;
    bool validateReadings(float inletPressure, float outletPressure, float flowRate) const;
};

#endif
