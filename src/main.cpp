#include "main.h"
#include "Arduino.h"

void setup()
{
  Serial.begin(115200);
  Serial1.setRxFIFOFull(32);
  Serial1.begin(115200, SERIAL_8N1);
  EEPROM.begin(512);
  // Initial calibration will happen on first flow reading
  // Initialize ADC first
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  delay(100); // Give ADC time to stabilize

  // Then initialize pressure sensors
  init_pressure_ch1();
  init_pressure_ch2();

  // Take a few initial readings to prime the moving average
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    readPressure_ch1();
    readPressure_ch2();
    delay(10);
  }
  flowThreshold = 30.0;
  BlynkEdgent.begin();
  // enableOTA();
  debugln("Setup complete");

  readFlowSensorData(readflowCommand, sizeof(readflowCommand), flowrate, cumulativeFlow, *pData, sizeof(*pData));
  initFlowThreshold();
  pressureCH1 = readPressure_ch1();
  pressureCH2 = readPressure_ch2();
  // After pressure sensor init
  filterMonitor.calibrate(
      (208.33 * ((3.3 * pressureCH1 / 4095.0) - 0.6)) - (208.33 * ((3.3 * pressureCH2 / 4095.0) - 0.6)),
      float(flowrate / 60.0));
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
    // Reset filters when there's no flow or at heartbeat interval
    // if (flowrate == 0 || now - reportTime >= UPDATE_FREQ)
    // {
    //   resetPressureFilters();
    // }
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
  blynk_data.flowrate = flowrate / 60.0; // Convert L/hr to L/min
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
  // Replace existing blockage detection with:
  auto blockageStatus = filterMonitor.update(
      blynk_data.pressure1,
      blynk_data.pressure2,
      blynk_data.flowrate);

  if (blockageStatus.requiresAttention)
  {
    Blynk.logEvent("filter_blockage", blockageStatus.message);
  }
  Blynk.virtualWrite(V14, blockageStatus.blockagePercentage);
}
void sendDatatoBlynk()
{
  Blynk.virtualWrite(V0, blynk_data.flowrate);
  Blynk.virtualWrite(V9, blynk_data.cumulativeflow);
  Blynk.virtualWrite(V1, blynk_data.irradiance);
  Blynk.virtualWrite(V2, blynk_data.pressure1);
  Blynk.virtualWrite(V3, blynk_data.pressure2);
  Blynk.virtualWrite(V4, blynk_data.dosage);
  Blynk.virtualWrite(V15, filterMonitor.isCalibrated() ? "Calibrated" : "Needs Calibration");
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
  if (value > 0)
  {
    flowThreshold = value;
    debugln("Flow threshold updated to: " + String(flowThreshold) + " L/min");
  }
  else
  {
    debugln("Invalid threshold value received: " + String(value));
  }
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
  static bool initialSettingsReceived = false;

  // Wait for initial settings from Blynk
  if (!initialSettingsReceived)
  {
    if (Blynk.connected())
    {
      Blynk.syncVirtual(V12); // Sync disable shutoff setting
      initialSettingsReceived = true;
    }
    return;
  }

  if (!isFlowAvailable || flowThreshold <= 0)
  {
    return;
  }

  debugln("Current flowrate: " + String(blynk_data.flowrate) + " L/min");
  debugln("Current threshold: " + String(flowThreshold) + " L/min");

  if (blynk_data.flowrate > flowThreshold)
  {
    burstData.consecutiveHighFlowCount++;
    debugln("High flow detected! Count: " + String(burstData.consecutiveHighFlowCount) +
            "/" + String(burstData.requiredCount) +
            " (Flow: " + String(blynk_data.flowrate) + " > Threshold: " + String(flowThreshold) + ")");

    if (burstData.consecutiveHighFlowCount >= burstData.requiredCount && !burstData.leakConfirmed)
    {
      burstData.leakConfirmed = true;
      burstData.burstDetection = true;
      burstData.valveLockedDueToLeak = true;

      String alertMsg = "Flow rate (" + String(blynk_data.flowrate) + ") exceeded threshold (" + String(flowThreshold) + ")";
      debugln("Leak confirmed! " + alertMsg);
      Blynk.logEvent("leak_detected", alertMsg);

      if (disableShutoff == 0)
      {
        Blynk.virtualWrite(V5, 1);
        valveOff();
        debugln("Valve closed due to leak detection");
      }
    }
  }
  else
  {
    if (burstData.consecutiveHighFlowCount > 0)
    {
      debugln("Flow returned to normal: " + String(blynk_data.flowrate) + " <= " + String(flowThreshold));
    }
    burstData.consecutiveHighFlowCount = 0;
  }
}

// Add this handler for the calibration button (V16)
BLYNK_WRITE(V16)
{
  if (param.asInt() == 1)
  {
    debugln("Calibrating filter system...");
    filterMonitor.calibrate(
        blynk_data.pressure1 - blynk_data.pressure2,
        blynk_data.flowrate);
    Blynk.logEvent("filter_calibration", "Filter system has been calibrated");
  }
}