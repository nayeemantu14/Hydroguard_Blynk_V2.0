#include <EEPROM.h>
#include "debug.h"
#define SERVOPIN 4
#define EEPROM_SIZE 1


uint8_t addr = 0;
uint8_t isValveOn;


void valveOn()
{
    debugln("on message received");
    //myservo.writeMicroseconds(1900);
    pinMode(SERVOPIN, OUTPUT);
    digitalWrite(SERVOPIN, HIGH);
    delay(500);
    isValveOn = 1;
    EEPROM.writeBool(addr, isValveOn);
    EEPROM.commit();
}

void valveOff()
{
    debugln("off message received");
    //myservo.writeMicroseconds(900);
    pinMode(SERVOPIN, OUTPUT);
    digitalWrite(SERVOPIN, LOW);
    delay(500);
    isValveOn = 0;
    EEPROM.writeBool(addr, isValveOn);
    EEPROM.commit();
}