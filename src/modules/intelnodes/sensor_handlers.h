#pragma once

#include <stdint.h>

// Sensor types (externally visible)
enum class SensorType : int {
  NONE = 0,
  MMWAVE,
  VIBRA,
  UNKNOWN
};

// Base class for sensor handlers. Implement each sensor in its own file/class.
class SensorHandler {
public:
  virtual ~SensorHandler() {}
  // Called once when handler object is created (module-level init).
  virtual void init() {}
  // Called when a module of this type is attached.
  virtual void onAttach() {}
  // Called when a module of this type is detached.
  virtual void onDetach() {}
  // Called periodically from the module's thread (non-blocking).
  virtual void loop() {}
  // human readable name
  virtual const char* name() const = 0;
};

// Factory: returns a heap-allocated handler for a given SensorType.
// Caller owns returned pointer and must delete it when done.
SensorHandler* createHandlerForType(SensorType t);
