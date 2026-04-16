#pragma once

#include "Arduino.h"
#include "mesh/MeshModule.h"
#include "concurrency/OSThread.h"
#include "sensor_handlers.h"

#ifndef SENSOR_SLOT_SENSE_PIN
  #define SENSOR_SLOT_SENSE_PIN  4   // safe GPIO by default
#endif

#ifndef SENSOR_POLL_MS
  #define SENSOR_POLL_MS 300
#endif

#ifndef SENSOR_DEBOUNCE_COUNT
  #define SENSOR_DEBOUNCE_COUNT 3
#endif

#ifndef SENSOR_RC_TIMEOUT_US
  #define SENSOR_RC_TIMEOUT_US 200000UL // 200 ms
#endif

// Timing thresholds (microseconds). Adjust as you calibrate.
#ifndef SENSOR_MMWAVE_MIN_US
  #define SENSOR_MMWAVE_MIN_US  40LL
#endif
#ifndef SENSOR_MMWAVE_MAX_US
  #define SENSOR_MMWAVE_MAX_US  65LL
#endif

#ifndef SENSOR_VIBRA_MIN_US
  #define SENSOR_VIBRA_MIN_US   1400LL
#endif
#ifndef SENSOR_VIBRA_MAX_US
  #define SENSOR_VIBRA_MAX_US   1600LL
#endif

enum class SensorState {
  NONE = 0,
  MMWAVE,
  VIBRA,
  UNKNOWN
};

class RCSlotModule : public concurrency::OSThread, public MeshModule {
public:
  RCSlotModule();
  virtual ~RCSlotModule();

  void setup() override;
  bool wantPacket(const meshtastic_MeshPacket *p) override;
  int32_t runOnce() override;

private:
  unsigned long lastPollMs_;
  SensorState curState_;
  SensorState lastStableRead_;
  int stableCount_;

  // Startup delay guard
  unsigned long detectReadyMs_;

  int64_t rcTimingRead(uint8_t sensePin, unsigned long timeout_us = SENSOR_RC_TIMEOUT_US);
  SensorState mapTimingToState(int64_t t_us);
  void onStateChanged(SensorState newState, int64_t measured_us);

  // Logging helper (safe)
  void dbgPrintf(const char *fmt, ...);

  // Handler for current sensor type (owns pointer)
  SensorHandler* activeHandler_;
  // helper to switch handler
  void switchHandler(SensorState newState);
};
