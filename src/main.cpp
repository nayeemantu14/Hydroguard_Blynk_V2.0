#include "main.h"
#include "Arduino.h"

void setup()
{
  Serial.begin(115200);
  Serial1.setRxFIFOFull(32);
  Serial1.begin(115200, SERIAL_8N1);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  init_pressure_ch1();
  init_pressure_ch2();
  flowThreshold = 30.0;
  BlynkEdgent.begin();
  // enableOTA();
  debugln("Setup complete");

  readFlowSensorData(readflowCommand, sizeof(readflowCommand), flowrate, cumulativeFlow, *pData, sizeof(*pData));
  initFlowThreshold();
}

void loop()
{
  BlynkEdgent.run();
  // server.handleClient();
  memset(*pData, 0, sizeof(*pData));
  uint32_t now = millis();

  if (!isTimeSet && WiFi.status() == WL_CONNECTED)
  {
    setupTime();
  }

  if (now - flowTime > 5000)
  {
    flowTime = millis();
    isFlowAvailable = readFlowSensorData(readflowCommand, sizeof(readflowCommand), flowrate, cumulativeFlow, *pData, sizeof(*pData));
    debugln(isFlowAvailable ? "Flow data available" : "Flow data not available");
    flowusage.currentHour = (uint8_t)rtc.getHour();
    flowusage.today = (uint8_t)rtc.getDay();
    flowusage.currentMonth = (uint8_t)rtc.getMonth();
  }

  if ((now - reportTime >= UPDATE_FREQ) || (isFlowAvailable && (flowrate != lastReportedFlowrate)))
  {
    reportTime = now;
    lastReportedFlowrate = flowrate;
    pressureCH1 = readPressure_ch1();
    pressureCH2 = readPressure_ch2();
    UVVoltage = readUV();
    displayFlow();
    sendESPdata();
    processData();
    checkBurst();
    checkShutoff();
    checkhourlyFlow();
    checkdailyFlow();
    checkmonthlyFlow();
    sendDatatoBlynk();
    memset(*pData, 0, sizeof(*pData)); // Clear the array after use
  }
}
void checkShutoff()
{
  UVLampOn = (UVVoltage >= 650) ? "On" : "Off"; // Example threshold for 6 mW/cm²

  if (disableShutoff == 0 && !burstData.valveLockedDueToLeak)
  {
    if (UVLampOn.equals("Off"))
    {
      Blynk.virtualWrite(V5, 1);
      valveOff();
      Blynk.logEvent("flostop_event");
    }
    else if (!blynk_data.userUpdate)
    {
      Blynk.virtualWrite(V5, 0);
      valveOn();
    }
  }
}
void displayFlow()
{
  char *msg = new char[255];
  sprintf(msg, "Flowrate: %.3f", flowrate);
  debugln(msg);
  free(msg);
}

void processData()
{
  blynk_data.flowrate = flowrate / 60.0;
  debugln(blynk_data.flowrate);
  blynk_data.cumulativeflow = cumulativeFlow;
  debugln(blynk_data.cumulativeflow);
  blynk_data.irradiance = 10 * (UVVoltage * 3.3 / 4095.0) * 1.515;
  debugln(blynk_data.irradiance);
  blynk_data.pressure1 = 208.33 * ((3.3 * pressureCH1 / 4095.0) - 0.6);
  debugln(blynk_data.pressure1);
  blynk_data.pressure2 = 208.33 * ((3.3 * pressureCH2 / 4095.0) - 0.6);
  debugln(blynk_data.pressure2);
  blynk_data.dosage = calculateUVDosage(&blynk_data.flowrate, &blynk_data.irradiance);
  debugln(blynk_data.dosage);
}
void sendDatatoBlynk()
{
  Blynk.virtualWrite(V0, blynk_data.flowrate);
  Blynk.virtualWrite(V9, blynk_data.cumulativeflow);
  Blynk.virtualWrite(V1, blynk_data.irradiance);
  Blynk.virtualWrite(V2, blynk_data.pressure1);
  Blynk.virtualWrite(V3, blynk_data.pressure2);
  Blynk.virtualWrite(V4, blynk_data.dosage);
}
BLYNK_WRITE(V5)
{
  if (param.asInt() == 1)
  {
    blynk_data.userUpdate = 1;
    valveOff();
  }
  else if (param.asInt() == 0)
  {
    Blynk.resolveEvent("flostop_event");
    Blynk.resolveEvent("leak_detected");
    blynk_data.userUpdate = 0;
    burstData.valveLockedDueToLeak = false;
    burstData.burstDetection = false;
    burstData.leakConfirmed = false;
    valveOn();
  }
}
BLYNK_WRITE(V11)
{
  if (param.asInt() == 1)
  {
    debugln("Resetting Cumulative Flow");
    resetTotalFlow(rstCFlowCommand, sizeof(rstCFlowCommand));
  }
}
BLYNK_WRITE(V12)
{
  if (param.asInt() == 1)
  {
    disableShutoff = 1;
    Blynk.logEvent("auto_flostop_disabled");
  }
  else
  {
    Blynk.resolveEvent("auto_flostop_disabled");
    disableShutoff = 0;
  }
}
BLYNK_WRITE(V13)
{
  float value = param.asFloat();
  flowThreshold = value;
}
void setupTime()
{
  timeClient.begin();
  timeClient.setTimeOffset(46800);
  timeClient.update();
  auto nzTime = timeClient.getEpochTime();
  debugln(timeClient.getFormattedTime());
  rtc.setTime(nzTime);
  flowusage.previousMonth = (uint8_t)rtc.getMonth();
  flowusage.flowLastMonth = cumulativeFlow;
  flowusage.yesterday = (uint8_t)rtc.getDay();
  flowusage.yesterdaysFlow = cumulativeFlow;
  flowusage.previousHour = (uint8_t)rtc.getHour();
  flowusage.flowPreviousHour = cumulativeFlow;

  isTimeSet = true;
}
void sendESPdata()
{
}

void checkdailyFlow()
{
  if (flowusage.today != flowusage.yesterday)
  {
    // Calculate today's flow
    flowusage.todaysFlow = cumulativeFlow - flowusage.yesterdaysFlow;
    Blynk.virtualWrite(V7, flowusage.todaysFlow);

    // Store the cumulative flow for the next day's comparison
    flowusage.yesterdaysFlow = cumulativeFlow;
    flowusage.yesterday = flowusage.today;

    // Debugging
    debugln("Daily Flow: " + String(flowusage.todaysFlow));
    debugln("Cumulative Flow: " + String(cumulativeFlow));
    debugln("Yesterday's Flow: " + String(flowusage.yesterdaysFlow));
  }
}

void checkmonthlyFlow()
{
  if (flowusage.currentMonth != flowusage.previousMonth)
  {
    // Calculate this month's flow
    flowusage.flowThisMonth = cumulativeFlow - flowusage.flowLastMonth;
    Blynk.virtualWrite(V8, flowusage.flowThisMonth);

    // Store the cumulative flow for the next month's comparison
    flowusage.flowLastMonth = cumulativeFlow;
    flowusage.previousMonth = flowusage.currentMonth;

    // Debugging
    debugln("Monthly Flow: " + String(flowusage.flowThisMonth));
    debugln("Cumulative Flow: " + String(cumulativeFlow));
    debugln("Last Month's Flow: " + String(flowusage.flowLastMonth));
  }
}

void checkhourlyFlow()
{
  if (flowusage.currentHour != flowusage.previousHour)
  {
    // Calculate this hour's flow
    flowusage.flowThisHour = cumulativeFlow - flowusage.flowPreviousHour;
    Blynk.virtualWrite(V10, flowusage.flowThisHour);

    // Store the cumulative flow for the next hour's comparison
    flowusage.flowPreviousHour = cumulativeFlow;
    flowusage.previousHour = flowusage.currentHour;

    // Debugging
    debugln("Hourly Flow: " + String(flowusage.flowThisHour));
    debugln("Cumulative Flow: " + String(cumulativeFlow));
    debugln("Previous Hour's Flow: " + String(flowusage.flowPreviousHour));
  }
}

void initFlowThreshold()
{
  if (!isThresholdSet)
  {
    cFlowThreshold = cumulativeFlow;
    isThresholdSet = true;
  }
}

void checkBurst()
{
  if (flowrate > flowThreshold)
  {
    burstData.consecutiveHighFlowCount++;
    debugln("High flow reading. Count: " + String(burstData.consecutiveHighFlowCount));

    if (burstData.consecutiveHighFlowCount >= burstData.requiredCount && !burstData.leakConfirmed)
    {
      burstData.leakConfirmed = true;
      burstData.burstDetection = true;
      burstData.valveLockedDueToLeak = true;

      Blynk.logEvent("leak_detected");
      if (disableShutoff == 0)
      {
        Blynk.virtualWrite(V5, 1);
        valveOff();
      }
    }
  }
  else
  {
    burstData.consecutiveHighFlowCount = 0;
    // leakConfirmed remains true until manually cleared by the user
  }
}