#include "sensor_handlers.h"
#include "Arduino.h"

// --- None handler (no-op) ---
class NoneHandler : public SensorHandler {
public:
  void init() override {}
  void onAttach() override {}
  void onDetach() override {}
  void loop() override {}
  const char* name() const override { return "NONE"; }
};

// --- mmWave handler (stub) ---
class MmWaveHandler : public SensorHandler {
public:
  void init() override {
    // Example: prepare any resources needed (I2C, SPI init is done elsewhere)
  }
  void onAttach() override {
    // Called when radar module attaches. Start driver, enable interrupts, etc.
    // Keep it non-blocking: schedule init work or use a small state machine
    // Example debug blink (do not block here)
  }
  void onDetach() override {
    // Tidy up, stop radar, power down if needed
  }
  void loop() override {
    // Called periodically; do non-blocking work here
  }
  const char* name() const override { return "mmWave Radar"; }
};

// --- Vibra handler (stub) ---
class VibraHandler : public SensorHandler {
public:
  void init() override {
    // Example: configure i2c or pwm driver if present
  }
  void onAttach() override {
    // Example: initialize vibra motor driver
  }
  void onDetach() override {
    // shutdown vibra
  }
  void loop() override {
    // update vibra state if needed
  }
  const char* name() const override { return "Vibra Module"; }
};

SensorHandler* createHandlerForType(SensorType t) {
  switch (t) {
    case SensorType::NONE:   return new NoneHandler();
    case SensorType::MMWAVE: return new MmWaveHandler();
    case SensorType::VIBRA:  return new VibraHandler();
    default:                 return new NoneHandler();
  }
}
