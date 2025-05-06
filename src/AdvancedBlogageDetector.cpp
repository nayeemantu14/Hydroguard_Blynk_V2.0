#include "AdvancedBlockageDetector.h"
#include <algorithm>

AdvancedBlockageDetector::BlockageStatus 
AdvancedBlockageDetector::update(float inletPressure, float outletPressure, float flowRate, float temperature) {
    BlockageStatus status;

    if (flowRate == 0.0f) {
        status.message            = "No flow detected";
        // blockagePercentage is already 0.0f thanks to {} init
        status.requiresAttention  = false;
        return status;
    }
    if(!validateReadings(inletPressure, outletPressure, flowRate)) {
        status.message = "Invalid sensor readings";
        // Maintain previous blockage percentage
        if (!blockageHistory.empty()) {
            status.blockagePercentage = std::accumulate(blockageHistory.begin(), blockageHistory.end(), 0.0f) / blockageHistory.size();
        }
        status.requiresAttention = wasAboveThreshold;
        return status;
    }

    if(flowRate <= MINIMUM_FLOW_THRESHOLD) {
        status.message = "Flow too low for measurement";
        return status;
    }

    float expectedPressure = calculateExpectedPressure(flowRate, temperature);
    float actualPressure = inletPressure - outletPressure;

    // Only update history if pressure difference is valid and significant
    if (actualPressure > 0.1f) { // Minimum 0.1 kPa difference to avoid noise
        // Update pressure history
        if (pressureHistory.size() >= MOVING_AVG_SIZE) {
            pressureHistory.erase(pressureHistory.begin());
        }
        pressureHistory.push_back(actualPressure);
    }

    // Calculate moving average
    float smoothedPressure = 0.0f;
    if (!pressureHistory.empty()) {
        smoothedPressure = std::accumulate(pressureHistory.begin(), pressureHistory.end(), 0.0f) / pressureHistory.size();
    }
    status.smoothedPressure = smoothedPressure;

    Serial.printf("Blockage Debug - Flow: %.2f, Expected: %.2f, Actual: %.2f, Smoothed: %.2f\n", 
                 flowRate, expectedPressure, actualPressure, smoothedPressure);

    // Calculate blockage percentage with improved algorithm
    float currentBlockage = 0.0f;
    if (expectedPressure > 0.0f) {
        float deviation = smoothedPressure - expectedPressure;
        currentBlockage = (deviation / expectedPressure) * 100.0f;
        currentBlockage = std::max(0.0f, std::min(currentBlockage, 100.0f));
    }

    // Update blockage history
    if (blockageHistory.size() >= MOVING_AVG_SIZE) {
        blockageHistory.erase(blockageHistory.begin());
    }
    blockageHistory.push_back(currentBlockage);

    // Calculate average blockage
    status.blockagePercentage = std::accumulate(blockageHistory.begin(), blockageHistory.end(), 0.0f) / blockageHistory.size();

    Serial.printf("Blockage Percentage: %.2f%%\n", status.blockagePercentage);

    // Apply hysteresis to prevent oscillation
    if (!wasAboveThreshold && status.blockagePercentage > (BLOCKAGE_ATTENTION_THRESHOLD + HYSTERESIS)) {
        wasAboveThreshold = true;
        status.requiresAttention = true;
    } else if (wasAboveThreshold && status.blockagePercentage < (BLOCKAGE_ATTENTION_THRESHOLD - HYSTERESIS)) {
        wasAboveThreshold = false;
        status.requiresAttention = false;
    } else {
        status.requiresAttention = wasAboveThreshold;
    }

    status.message = determineBlockageMessage(status.blockagePercentage);
    return status;
}

float AdvancedBlockageDetector::calculateExpectedPressure(float flowRate, float temperature) const {
    if (flowRate < MINIMUM_FLOW_THRESHOLD) {
        return 0.0f;
    }
    // Basic pressure drop equation based on your dataset
    // y = 0.0178x² + 1.1692x - 0.3355
    float expectedDrop = 0.0178f * flowRate * flowRate + 1.1692f * flowRate - 0.3355f;
    return expectedDrop > 0.0f ? expectedDrop : 0.0f;
}

float AdvancedBlockageDetector::applyExponentialSmoothing(float newValue, float lastValue) const {
    if (lastValue == 0.0f) {
        return newValue;
    }
    // Reduced smoothing factor for more stable readings
    static const float REDUCED_SMOOTHING = 0.1f;
    return REDUCED_SMOOTHING * newValue + (1.0f - REDUCED_SMOOTHING) * lastValue;
}

bool AdvancedBlockageDetector::validateReadings(float inletPressure, float outletPressure, float flowRate) const {
    return inletPressure >= 0.0f && outletPressure >= 0.0f && 
           inletPressure >= outletPressure && flowRate >= 0.0f;
}



std::string AdvancedBlockageDetector::determineBlockageMessage(float blockagePercentage) const {
    if(blockagePercentage < 25.0f) return "Optimal Flow";
    if(blockagePercentage < 50.0f) return "Moderate Restriction";
    if(blockagePercentage < 75.0f) return "Severe Restriction";
    return "Critical Blockage";
}