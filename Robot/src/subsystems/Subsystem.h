#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <Arduino.h>


//Abstract class so robot subsystems can be looped through. This defines the functions that each subsystem is required to include
class Subsystem {
public:
    virtual ~Subsystem() = default;

    // Called once at startup
    virtual void init() = 0;

    // Called periodically from loop()
    virtual void update() = 0;

    // Optional safe-state behavior
    virtual void stop() {}

    // Optional health check
    virtual bool isHealthy() const { return true; }

protected:
    Subsystem() = default;
};

#endif
