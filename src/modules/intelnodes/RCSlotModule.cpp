#include "RCSlotModule.h"
#include "Arduino.h"
#include <stdarg.h>

// include factory
#include "sensor_handlers.h"

RCSlotModule::RCSlotModule()
  : concurrency::OSThread("RCSlot", 8192),
    MeshModule("RCSlot"),
    lastPollMs_(0),
    curState_(SensorState::NONE),
    lastStableRead_(SensorState::NONE),
    stableCount_(0),
    detectReadyMs_(0),
    activeHandler_(nullptr)
{
}

RCSlotModule::~RCSlotModule() {
  if (activeHandler_) delete activeHandler_;
}

void RCSlotModule::setup() {
  pinMode(SENSOR_SLOT_SENSE_PIN, INPUT);
  detectReadyMs_ = millis() + 2000UL; // small startup delay
  // create initial handler (NONE)
  switchHandler(SensorState::NONE);
  dbgPrintf("RCSlotModule: setup complete; sensePin=%d, detectReadyMs=%lu", SENSOR_SLOT_SENSE_PIN, detectReadyMs_);
}

bool RCSlotModule::wantPacket(const meshtastic_MeshPacket *p) {
  (void)p;
  return false;
}

int32_t RCSlotModule::runOnce() {
  // Wait for system ready
  if (millis() < detectReadyMs_) return (int32_t)(detectReadyMs_ - millis());

  unsigned long now = millis();
  if ((now - lastPollMs_) < SENSOR_POLL_MS) return (int32_t)(SENSOR_POLL_MS - (now - lastPollMs_));
  lastPollMs_ = now;

  int64_t t_us = rcTimingRead((uint8_t)SENSOR_SLOT_SENSE_PIN, SENSOR_RC_TIMEOUT_US);
  SensorState detected = mapTimingToState(t_us);

  if (detected == lastStableRead_) stableCount_++; else { stableCount_ = 1; lastStableRead_ = detected; }

  if (stableCount_ >= SENSOR_DEBOUNCE_COUNT && detected != curState_) {
    curState_ = detected;
    onStateChanged(detected, t_us);
  }

  // call handler loop so per-sensor logic can run (non-blocking)
  if (activeHandler_) activeHandler_->loop();

  // short safe debug line
  if (t_us < 0) dbgPrintf("RC read: TIMEOUT => %d (stable=%d)", (int)detected, stableCount_);
  else dbgPrintf("RC read: %lld us => %d (stable=%d)", (long long)t_us, (int)detected, stableCount_);

  return (int32_t)SENSOR_POLL_MS;
}

int64_t RCSlotModule::rcTimingRead(uint8_t sensePin, unsigned long timeout_us) {
  // discharge
  pinMode(sensePin, OUTPUT);
  digitalWrite(sensePin, LOW);
  delayMicroseconds(50);

  pinMode(sensePin, INPUT);
  unsigned long start = micros();
  while (true) {
    unsigned long now = micros();
    unsigned long diff = now - start;
    if (digitalRead(sensePin) == HIGH) return (int64_t)diff;
    if (diff >= timeout_us) return -1;
    yield();
  }
}

SensorState RCSlotModule::mapTimingToState(int64_t t_us) {
  if (t_us < 0) return SensorState::NONE;

  if (t_us >= SENSOR_MMWAVE_MIN_US && t_us <= SENSOR_MMWAVE_MAX_US) {
    return SensorState::MMWAVE;
  }
  if (t_us >= SENSOR_VIBRA_MIN_US && t_us <= SENSOR_VIBRA_MAX_US) {
    return SensorState::VIBRA;
  }
  // allow legacy/short values to map to UNKNOWN
  return SensorState::UNKNOWN;
}

void RCSlotModule::onStateChanged(SensorState newState, int64_t measured_us) {
  // Print friendly name
  const char* sname = "UNK";
  switch (newState) {
    case SensorState::NONE:   sname = "NONE"; break;
    case SensorState::MMWAVE: sname = "mmWave Radar"; break;
    case SensorState::VIBRA:  sname = "Vibra Module"; break;
    case SensorState::UNKNOWN: sname = "UNKNOWN"; break;
  }

  if (measured_us < 0) dbgPrintf("RCSlot: STATE CHANGED -> %s (timeout)", sname);
  else dbgPrintf("RCSlot: STATE CHANGED -> %s (%lld us)", sname, (long long)measured_us);

  // switch handler (deletes previous and creates new one)
  switchHandler(newState);
}

void RCSlotModule::switchHandler(SensorState newState) {
  // If the state didn't change in handler terms, do nothing
  if (activeHandler_) {
    // if same name, skip detach/attach (optional optimization)
    // We'll always call detach->delete->create->attach to be explicit.
    activeHandler_->onDetach();
    delete activeHandler_;
    activeHandler_ = nullptr;
  }

  // create new handler
  SensorType st = SensorType::NONE;
  switch (newState) {
    case SensorState::NONE:   st = SensorType::NONE; break;
    case SensorState::MMWAVE: st = SensorType::MMWAVE; break;
    case SensorState::VIBRA:  st = SensorType::VIBRA; break;
    default:                  st = SensorType::NONE; break;
  }

  activeHandler_ = createHandlerForType(st);
  if (activeHandler_) {
    activeHandler_->init();
    activeHandler_->onAttach();
    dbgPrintf("RCSlot: handler switched to %s", activeHandler_->name());
  }
}

// ---------- safe dbgPrintf ----------
void RCSlotModule::dbgPrintf(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

#ifdef LOG_INFO
  static char logbuf[256];
  vsnprintf(logbuf, sizeof(logbuf), fmt, args);
  va_end(args);
  LOG_INFO("%s", logbuf);
  return;
#else
  static unsigned long lastLogMs = 0;
  const unsigned long LOG_THROTTLE_MS = 150;
  unsigned long now = millis();
  if (now - lastLogMs < LOG_THROTTLE_MS) { va_end(args); return; }

  static char buf[256];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (!Serial) { Serial.begin(115200); delay(5); }
  Serial.write("[RCSlot] ");
  Serial.write(buf);
  Serial.write("\r\n");

  lastLogMs = now;
  return;
#endif
}
