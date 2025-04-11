#include "AdvancedBlockageDetector.h"

AdvancedBlockageDetector::AdvancedBlockageDetector()
{
    memset(pressureDiffBuffer, 0, sizeof(pressureDiffBuffer));
    memset(flowRateBuffer, 0, sizeof(flowRateBuffer));
    memset(learningFlows, 0, sizeof(learningFlows));
    memset(learningPressures, 0, sizeof(learningPressures));
    learningPoints = 0;
    quadraticCoeff = DEFAULT_QUADRATIC_COEFF;
    linearCoeff = DEFAULT_LINEAR_COEFF;
    constantCoeff = DEFAULT_CONSTANT_COEFF;
    if (loadCalibration() == false)
    {
        reset();
    }
}

float AdvancedBlockageDetector::calculateExpectedPressure(float flowRate) const
{
    return quadraticCoeff * flowRate * flowRate +
           linearCoeff * flowRate +
           constantCoeff;
}

void AdvancedBlockageDetector::learnFromData(float pressureDiff, float flowRate)
{
    if (learningPoints < sizeof(learningFlows) / sizeof(learningFlows[0]))
    {
        learningFlows[learningPoints] = flowRate;
        learningPressures[learningPoints] = pressureDiff;
        learningPoints++;

        if (learningPoints >= MIN_LEARNING_POINTS)
        {
            // Simple least squares fitting for quadratic curve
            float sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
            float sumY = 0, sumXY = 0, sumX2Y = 0;

            for (uint8_t i = 0; i < learningPoints; i++)
            {
                float x = learningFlows[i];
                float x2 = x * x;
                float y = learningPressures[i];

                sumX += x;
                sumX2 += x2;
                sumX3 += x2 * x;
                sumX4 += x2 * x2;
                sumY += y;
                sumXY += x * y;
                sumX2Y += x2 * y;
            }

            float n = learningPoints;
            float det = sumX4 * (sumX2 * n - sumX * sumX) -
                        sumX3 * (sumX3 * n - sumX * sumX2) +
                        sumX2 * (sumX3 * sumX - sumX2 * sumX2);

            if (abs(det) > 0.0001f)
            {
                quadraticCoeff = ((sumX2Y * (sumX2 * n - sumX * sumX) -
                                   sumXY * (sumX3 * n - sumX * sumX2) +
                                   sumY * (sumX3 * sumX - sumX2 * sumX2)) /
                                  det);

                linearCoeff = ((sumX4 * sumXY - sumX3 * sumX2Y +
                                sumX2Y * sumX3 - sumXY * sumX4 +
                                sumY * sumX2 * sumX2 - sumY * sumX3 * sumX) /
                               det);

                constantCoeff = ((sumX4 * (sumX2 * sumY - sumX * sumXY) -
                                  sumX3 * (sumX3 * sumY - sumX * sumX2Y) +
                                  sumX2 * (sumX3 * sumXY - sumX2 * sumX2Y)) /
                                 det);
            }
        }
    }
}

float AdvancedBlockageDetector::calculateResistance(float pressureDiff, float flowRate) const
{
    if (flowRate <= FLOW_THRESHOLD)
    {
        return 0.0f;
    }

    if (!isCalibrated())
    {
        // Use default equation when uncalibrated
        float expectedPressure = calculateExpectedPressure(flowRate);
        return expectedPressure / flowRate;
    }

    return pressureDiff / flowRate;
}

void AdvancedBlockageDetector::calibrate(float pressureDiff, float flowRate)
{
    if (flowRate > FLOW_THRESHOLD)
    {
        // Learn from new data point
        learnFromData(pressureDiff, flowRate);

        // Update baseline resistance using learned curve
        baselineResistance = calculateResistance(pressureDiff, flowRate);
        bufferInitialized = false;
        lastCalibrationTime = millis();
        saveCalibration();
    }
}

bool AdvancedBlockageDetector::loadCalibration()
{
    uint32_t calibratedFlag;
    EEPROM.get(EEPROM_CALIBRATED_FLAG, calibratedFlag);

    if (calibratedFlag == 0xCAFEBABE)
    { // Magic number to verify calibration
        EEPROM.get(EEPROM_BASELINE_RESISTANCE, baselineResistance);
        EEPROM.get(EEPROM_LAST_CALIBRATION, lastCalibrationTime);
        return true;
    }
    return false;
}

bool AdvancedBlockageDetector::saveCalibration()
{
    uint32_t calibratedFlag = 0xCAFEBABE;
    EEPROM.put(EEPROM_CALIBRATED_FLAG, calibratedFlag);
    EEPROM.put(EEPROM_BASELINE_RESISTANCE, baselineResistance);
    EEPROM.put(EEPROM_LAST_CALIBRATION, lastCalibrationTime);
    return EEPROM.commit();
}

bool AdvancedBlockageDetector::isCalibrated() const
{
    uint32_t calibratedFlag;
    EEPROM.get(EEPROM_CALIBRATED_FLAG, calibratedFlag);
    return calibratedFlag == 0xCAFEBABE;
}

uint32_t AdvancedBlockageDetector::getLastCalibrationTime() const
{
    return lastCalibrationTime;
}

AdvancedBlockageDetector::BlockageStatus AdvancedBlockageDetector::update(
    float inletPressure, float outletPressure, float flowRate)
{
    BlockageStatus status;

    // Sensor validation
    float pressureDiff = pressureFilter.update(inletPressure - outletPressure);
    float filteredFlow = flowFilter.update(flowRate);

    if (!validateReadings(pressureDiff, filteredFlow))
    {
        status.message = "Sensor Validation Failed";
        status.confidence = 0.0f;
        status.blockagePercentage = 0.0f;
        return status;
    }

    // Update circular buffers
    pressureDiffBuffer[bufferIndex] = pressureDiff;
    flowRateBuffer[bufferIndex] = filteredFlow;
    bufferIndex = (bufferIndex + 1) % HISTORY_SIZE;
    bufferInitialized = bufferIndex == 0; // Buffer is initialized after first complete cycle

    // No auto-calibration, system relies on initial and manual calibration only

    // Calculate metrics
    float currentResistance = calculateResistance(pressureDiff, filteredFlow);
    if (filteredFlow <= FLOW_THRESHOLD)
    {
        status.blockagePercentage = lastValidBlockage;
        status.confidence = 0.0f;
        status.message = "Flow too low for accurate measurement";
        return status;
    }
    const float normalizedResistance = baselineResistance > 0 ? (currentResistance / baselineResistance) : 1.0f;

    // Dynamic threshold calculation
    float avgFlow = 0.0f;
    for (uint8_t i = 0; i < HISTORY_SIZE; i++)
    {
        avgFlow += flowRateBuffer[i];
    }
    avgFlow /= HISTORY_SIZE;
    adaptiveThreshold = 70.0f * (1.0f + (FLOW_THRESHOLD / avgFlow));

    // Blockage percentage calculation
    float resistanceComponent = pow(normalizedResistance - 1.0f, 2) * 50.0f;
    float trendComponent = calculateTrend() * 30.0f;
    float flowComponent = (1.0f - (filteredFlow / MAX_FLOW)) * 20.0f;

    status.blockagePercentage = constrain(
        resistanceComponent + trendComponent + flowComponent,
        0.0f, 100.0f);
    lastValidBlockage = status.blockagePercentage;

    // Confidence calculation
    status.confidence = constrain(
        (filteredFlow / FLOW_THRESHOLD * 60.0f) +
            (1.0f - (sensorVariance / PRESSURE_RANGE)) * 40.0f,
        0.0f, 100.0f);

    // Status determination
    status.requiresAttention = (status.blockagePercentage > adaptiveThreshold) &&
                               (status.confidence > 75.0f);

    // Message generation
    if (status.blockagePercentage < 25.0f)
    {
        status.message = "Optimal Flow";
    }
    else if (status.blockagePercentage < 50.0f)
    {
        status.message = "Moderate Restriction";
    }
    else if (status.blockagePercentage < 75.0f)
    {
        status.message = "Severe Restriction";
    }
    else
    {
        status.message = "Critical Blockage";
    }

    return status;
}

bool AdvancedBlockageDetector::validateReadings(float pressureDiff, float flowRate)
{
    static float lastPressure = pressureDiff;

    // Check for sudden impossible changes
    if (abs(pressureDiff - lastPressure) > 60.0f && flowRate < 5.0f)
    { // Increased for 600 kPa range
        return false;
    }

    // Check sensor consistency
    float currentVariance = 0.0f;
    for (uint8_t i = 0; i < HISTORY_SIZE; i++)
    {
        currentVariance += pow(pressureDiffBuffer[i] - pressureDiff, 2);
    }
    sensorVariance = currentVariance / HISTORY_SIZE;

    lastPressure = pressureDiff;
    return sensorVariance < 100.0f;
}

float AdvancedBlockageDetector::calculateTrend()
{
    float trend = 0.0f;
    float alpha = 0.15f;

    for (uint8_t i = 0; i < HISTORY_SIZE; i++)
    {
        uint8_t idx = (bufferIndex - i) % HISTORY_SIZE;
        uint8_t prev_idx = (idx - 1) % HISTORY_SIZE;

        float delta = pressureDiffBuffer[idx] - pressureDiffBuffer[prev_idx];
        trend = alpha * delta + (1 - alpha) * trend;
    }

    return constrain(trend * 10.0f, -1.0f, 1.0f);
}

void AdvancedBlockageDetector::reset()
{
    memset(pressureDiffBuffer, 0, sizeof(pressureDiffBuffer));
    memset(flowRateBuffer, 0, sizeof(flowRateBuffer));
    bufferIndex = 0;
    baselineResistance = 0.0f;
    bufferInitialized = false;
    lastCalibrationTime = 0;
    saveCalibration();
}

float AdvancedBlockageDetector::getCurrentResistance() const
{
    return calculateResistance(
        pressureDiffBuffer[(bufferIndex - 1) % HISTORY_SIZE],
        flowRateBuffer[(bufferIndex - 1) % HISTORY_SIZE]);
}