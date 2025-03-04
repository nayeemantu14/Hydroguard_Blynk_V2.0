#ifndef UV_H
#define UV_H

#include <Arduino.h>

#define UVPIN 7

void UV_init();
uint16_t readUV();


void UV_init()
{
    pinMode(UVPIN, INPUT);
}

uint16_t readUV()
{
    uint16_t analogUV = analogRead(UVPIN);
    return analogUV;
}

float calculateUVDosage(float *flowrate, float *irradiance) {
    float volume = 0.0007; // Volume in cubic meters (m³)

    // Convert flowrate from L/min to m³/s
    float flowrateInM3 = (*flowrate / 60.0) * 0.001; // L/min -> m³/s

    // Avoid division by zero
    if (flowrateInM3 <= 0) {
        return 0.0;
    }

    // Calculate residence time (t = Volume / Flow rate)
    float residenceTime = volume / flowrateInM3; // seconds

    // Calculate UV dosage (UV Dosage = Irradiance × Residence Time)
    return (*irradiance) * residenceTime; // mJ/cm²
}
#endif