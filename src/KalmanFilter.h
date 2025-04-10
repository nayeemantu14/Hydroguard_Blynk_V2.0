#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// A basic 1D Kalman Filter for smoothing a single sensor measurement
class KalmanFilter
{
private:
    float estimate;         // Current state estimate
    float errorCovariance;  // Error covariance of the estimate
    float processNoise;     // Q: process noise variance
    float measurementNoise; // R: measurement noise variance

public:
    /**
     * Constructor
     * @param processNoise     Q: Models how quickly the system (or reading) can change.
     * @param measurementNoise R: Models the noise variance in the sensor reading.
     */
    KalmanFilter(float processNoise = 1.0f, float measurementNoise = 4.0f)
        : estimate(0.0f), errorCovariance(1.0f), processNoise(processNoise), measurementNoise(measurementNoise) {}

    /**
     * Updates the filter with a new measurement and returns the smoothed (filtered) value.
     */
    float update(float measurement)
    {
        // 1. Predict (we assume the estimate can drift by processNoise)
        errorCovariance += processNoise;

        // 2. Measurement update
        float kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
        estimate += kalmanGain * (measurement - estimate);
        errorCovariance *= (1.0f - kalmanGain);

        return estimate;
    }

    /**
     * Optional: reset the estimate and error covariance (e.g., if you suspect a large jump).
     */
    void reset(float newEstimate = 0.0f, float newErrorCov = 1.0f)
    {
        estimate = newEstimate;
        errorCovariance = newErrorCov;
    }
};

#endif // KALMAN_FILTER_H
