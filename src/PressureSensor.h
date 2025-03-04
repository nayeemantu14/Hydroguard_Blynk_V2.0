#include<Arduino.h>

#define PRESSCH1 5  
#define PRESSCH2 6

void init_pressure_ch1()
{
    pinMode(PRESSCH1, INPUT_PULLDOWN);
}

void init_pressure_ch2()
{
    pinMode(PRESSCH2, INPUT_PULLDOWN);
}

uint16_t readPressure_ch1()
{
    uint16_t analogPressure = analogRead(PRESSCH1);
    return analogPressure;
}

uint16_t readPressure_ch2()
{
    uint16_t analogPressure = analogRead(PRESSCH2);
    return analogPressure;
}