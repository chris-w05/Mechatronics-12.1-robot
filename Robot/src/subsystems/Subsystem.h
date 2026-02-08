#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

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
