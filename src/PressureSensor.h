
#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

#define PRESSCH1 5  
#define PRESSCH2 6

// Moving average filter configuration - moved to top
const int WINDOW_SIZE = 20;  // Increased window size for better smoothing
static uint16_t readings1[WINDOW_SIZE];
static uint16_t readings2[WINDOW_SIZE];
static int readIndex = 0;

// Function declaration before use
uint16_t applyMovingAverage(uint16_t newReading, uint16_t* readings);

void init_pressure_ch1() {
    pinMode(PRESSCH1, INPUT_PULLDOWN);
}

void init_pressure_ch2() {
    pinMode(PRESSCH2, INPUT_PULLDOWN);
}

uint16_t readPressure_ch1() {
    uint16_t rawPressure = analogRead(PRESSCH1);
    return applyMovingAverage(rawPressure, readings1);
}

uint16_t readPressure_ch2() {
    uint16_t rawPressure = analogRead(PRESSCH2);
    return applyMovingAverage(rawPressure, readings2);
}

void resetPressureFilters() {
    readIndex = 0;
    for(int i = 0; i < WINDOW_SIZE; i++) {
        readings1[i] = 0;
        readings2[i] = 0;
    }
}

uint16_t applyMovingAverage(uint16_t newReading, uint16_t* readings) {
    readings[readIndex] = newReading;
    uint32_t sum = 0;
    
    for(int i = 0; i < WINDOW_SIZE; i++) {
        sum += readings[i];
    }
    
    readIndex = (readIndex + 1) % WINDOW_SIZE;
    return sum / WINDOW_SIZE;
}

#endif
