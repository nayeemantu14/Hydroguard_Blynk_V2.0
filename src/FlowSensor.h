#ifndef FLOWSENSOR_H
#define FLOWSENSOR_H

#include <Arduino.h>
#include "debug.h"
#include <cmath>

// Function Prototypes
float bcdToFloat(byte b1, byte b2, byte b3, byte b4);
bool readFlowSensorData(byte *command, size_t commandSize, float &flowrate, double &cumulativeFlow, byte *data, size_t dataSize);
double readCumulativeFlow(byte *data);
void resetTotalFlow(byte *command, size_t commandSize);
/**
 * Converts a 4-byte BCD value to a float.
 * Example: BCD (78 56 34 12) -> 123456.78
 */
float bcdToFloat(byte b1, byte b2, byte b3, byte b4)
{
    int integerPart = ((b4 >> 4) * 100000) + ((b4 & 0x0F) * 10000) +
                      ((b3 >> 4) * 1000) + ((b3 & 0x0F) * 100) +
                      ((b2 >> 4) * 10) + (b2 & 0x0F);
    int decimalPart = ((b1 >> 4) * 10) + (b1 & 0x0F);
    return integerPart + (decimalPart / 100.0);
}

/**
 * Reads data from the flow sensor and calculates the flowrate.
 * @param command Command to request data from the sensor.
 * @param commandSize Size of the command array.
 * @param flowrate Reference to store the calculated flowrate.
 * @param data Buffer to store raw data received from the sensor.
 * @param dataSize Expected size of the data buffer.
 * @return True if data is successfully read and processed, false otherwise.
 */
bool readFlowSensorData(byte *command, size_t commandSize, float &flowrate, double &cumulativeFlow, byte *data, size_t dataSize)
{
    // Clear the data array
    memset(data, 0, dataSize);
    delay(50);

    // Clear any existing data in the Serial buffer
    while (Serial1.available())
    {
        Serial1.read();
    }

    // Send command to the flow sensor
    if (Serial1.availableForWrite())
    {
        Serial1.write(command, commandSize);
        Serial1.flush();
    }

    // Temporary buffer for data with possible offset
    const size_t tempBufferSize = 64;
    byte tempBuffer[tempBufferSize];
    memset(tempBuffer, 0, tempBufferSize);

    // Wait for sufficient data
    unsigned long startTime = millis();
    while (Serial1.available() < dataSize + 25)
    { // Allow for offset
        if (millis() - startTime > 1000)
        { // 1-second timeout
            debugln("Timeout waiting for data");
            return false;
        }
    }

    delay(50);
    int bytesRead = Serial1.readBytes(tempBuffer, tempBufferSize);

    // Find the start sequence (0x42, 0x4D)
    int startIndex = -1;
    for (int i = 0; i < bytesRead - 1; i++)
    {
        if (tempBuffer[i] == 0x42 && tempBuffer[i + 1] == 0x4D)
        {
            startIndex = i;
            break;
        }
    }

    // If start sequence is not found
    if (startIndex == -1)
    {
        debugln("Start sequence not found");
        debugln("Received Data:");
        for (int i = 0; i < bytesRead; i++)
        {
            Serial.printf("tempBuffer[%d]: %02X\r\n", i, tempBuffer[i]);
        }
        return false;
    }

    // Copy aligned data to the output buffer
    if (startIndex + dataSize > bytesRead)
    {
        debugln("Not enough data after start sequence");
        return false;
    }
    memcpy(data, &tempBuffer[startIndex], dataSize);

    // Validate the packet
    if (data[0] != 0x42 && data[1] != 0x4D && data[31] != 0x16)
    {
        debugln("Invalid packet");
        debugln("Start Packet:");
        for (uint8_t i = 0; i < 32; i++)
        {
            Serial.printf("data[%u]: %02X\r\n", i, data[i]);
        }
        debugln("End Packet");

        // Send reset command if the packet is invalid
        byte resetCMD[] = {0x10, 0x5D, 0xFD, 0x5A, 0x16};
        Serial1.write(resetCMD, sizeof(resetCMD));
        Serial1.flush();

        // Clear buffer
        if (Serial1.available() > 0)
        {
            byte failData = Serial1.read();
            Serial.printf("Failed Data: %02X\r\n", failData);
        }
        return false;
    }

    // Calculate flowrate
    flowrate = bcdToFloat(data[16], data[17], data[18], data[19]);
    cumulativeFlow = readCumulativeFlow(data);
    debugln("Start Packet:");
    for (uint8_t i = 0; i < 32; i++)
    {
        Serial.printf("data[%u]: %02X\r\n", i, data[i]);
    }
    debugln("End Packet");
    debugln("Flowrate read successfully");
    return true;
}

double readCumulativeFlow(byte *data)
{
    if (data[8] != 0x0A)
    {
        Serial.println("Incorrect data received");
        return -1.0;
    }

    // Helper lambda: converts a BCD encoded byte into a two-digit string.
    auto bcdToTwoDigit = [](byte value) -> String
    {
        char buf[3];
        // Extract the high nibble and low nibble and combine them into a number.
        int digitValue = (((value >> 4) & 0x0F) * 10) + (value & 0x0F);
        sprintf(buf, "%02d", digitValue);
        return String(buf);
    };

    // Build the integer part:
    // Use data[14], data[13], data[12], data[11] (each as two-digit strings)
    // then append the tens digit of data[10] (the high nibble).
    String integerPart = "";
    integerPart += bcdToTwoDigit(data[14]);
    integerPart += bcdToTwoDigit(data[13]);
    integerPart += bcdToTwoDigit(data[12]);
    integerPart += bcdToTwoDigit(data[11]);
    // For the tens digit of data[10], extract the high nibble (a single digit)
    integerPart += String((data[10] >> 4) & 0x0F);

    // Build the decimal part:
    // Use the ones digit of data[10] (low nibble) followed by data[9] as a two-digit string.
    String decimalPart = "";
    decimalPart += String(data[10] & 0x0F); // ones digit of data[10]
    decimalPart += bcdToTwoDigit(data[9]);

    // Combine the two parts into a complete string representation.
    String flowStr = integerPart + "." + decimalPart;
    Serial.print("Flow string: ");
    Serial.println(flowStr);

    // Convert the string to a double and return.
    return flowStr.toDouble();
}

void resetTotalFlow(byte *command, size_t commandSize)
{

    while (Serial1.available())
    {
        Serial1.read();
    }

    // Send command to the flow sensor
    if (Serial1.availableForWrite())
    {
        Serial1.write(command, commandSize);
        Serial1.flush();
        Serial.println(Serial1.read());
    }
}

#endif // FLOWSENSOR_H
