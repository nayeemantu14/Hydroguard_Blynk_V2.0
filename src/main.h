#define BLYNK_FIRMWARE_VERSION        "1.4.4"

#include <Arduino.h>
#include "debug.h"
#include <esp_sleep.h>
#include <WiFiClient.h>
#include "BlynkEdgent.h"
#include <ESP32Time.h>
#include <NTPClient.h>
#include "FlowSensor.h"
#include "Servo.h"
#include "PressureSensor.h"
#include "UV.h"
#include "AdvancedBlockageDetector.h"

//system defines
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 20       /* Time ESP32 will go to sleep (in seconds) */
#define UPDATE_FREQ 600000
//Blynk defines
#define BLYNK_TEMPLATE_ID "TMPL64xy5PU3f"
#define BLYNK_TEMPLATE_NAME "Hydroguard"

#define BLYNK_PRINT Serial

#define APP_DEBUG

//Objects
WiFiUDP Udp;
NTPClient timeClient(Udp);
ESP32Time rtc;
//Structs
typedef struct
{
    float flowrate;
    double cumulativeflow;
    float irradiance;
    float dosage;
    float pressure1;
    float pressure2;
    uint8_t userUpdate;
}pload_t;

typedef struct
{
    uint8_t today;          // Current day
    uint8_t yesterday;      // Previous day
    double todaysFlow;      // Flow recorded today
    double yesterdaysFlow;  // Flow recorded yesterday
    uint8_t currentMonth;   // Current month
    uint8_t previousMonth;  // Last month
    double flowThisMonth;   // Flow recorded this month
    double flowLastMonth;   // Flow recorded last month
    uint8_t currentHour;
    uint8_t previousHour;
    double flowThisHour;
    double flowPreviousHour;
}FlowUsage_t;

typedef struct {
    uint8_t consecutiveHighFlowCount;  // Number of consecutive readings above threshold
    uint8_t requiredCount;             // Number of readings required to confirm a leak
    bool leakConfirmed;                // Flag to indicate if leak has been confirmed
    bool burstDetection;
    bool valveLockedDueToLeak;
} BurstDetection_t;


// Function Prototypes
void displayFlow();
void sendESPdata();
float flowToFloat(byte b1, byte b2, byte b3, byte b4);
void checkBurst();
void checkmonthlyFlow();
void checkdailyFlow();
void checkhourlyFlow();
void processData();
void sendDatatoBlynk();
void setupTime();
void initFlowThreshold();
byte readflowCommand[] = {0x10, 0x5B, 0xFD, 0x58, 0x16};
byte rstCFlowCommand[] = {0x10, 0x5A, 0xFD, 0x57, 0x16};

//extern
extern AdvancedBlockageDetector filterMonitor;

// Global Variables
uint32_t tempTime = millis();
uint32_t flowTime = 0;
uint32_t reportTime = 0; // Track the last report time
uint32_t blereportTime = millis();

byte data[32];
byte (*pData)[32] = &data; // Pointer to the array

static float flowrate;
static float flowThreshold = 25.0;
static double cumulativeFlow;
static uint16_t pressureCH1 = 0;
static uint16_t pressureCH2 = 0;
static uint16_t UVVoltage = 0;
static String UVLampOn;
static float lastReportedFlowrate = 0.0; // Track the last reported flow rate
static double cFlowThreshold;
static bool isThresholdSet = false;
static bool isFlowAvailable = false;
static bool isTimeSet = false;
static uint8_t disableShutoff;
static pload_t blynk_data;
static FlowUsage_t flowusage;
static BurstDetection_t burstDetection;
BurstDetection_t burstData = {
    0,      // consecutiveHighFlowCount
    3,      // requiredCount
    false,  // leakConfirmed
    false,  // burstDetection
    false   // valveLockedDueToLeak
};
AdvancedBlockageDetector filterMonitor;