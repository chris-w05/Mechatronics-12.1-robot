/**
 * @file Subsystem.h
 * @brief Abstract base class for all robot subsystems.
 *
 * Every hardware subsystem (Drive, Miner, Shooter, etc.) inherits from
 * Subsystem so the Robot class can manage them through a common interface.
 * Concrete subclasses must implement init() and update(); all other hooks
 * have safe default implementations.
 */
#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <Arduino.h>

/**
 * @brief Polymorphic interface for a robot subsystem.
 *
 * All subsystems are expected to follow the init / update lifecycle:
 * - `init()` is called once from `Robot::init()` during setup.
 * - `update()` is called every loop iteration from `Robot::update()`.
 *
 * Optionally, `stop()` can place the subsystem in a known safe state
 * (e.g., zero motor power) and `isHealthy()` can report fault conditions.
 */
class Subsystem {
public:
    virtual ~Subsystem() = default;

    /** @brief Initialise hardware and reset internal state. Called once in setup(). */
    virtual void init() = 0;

    /** @brief Update control outputs. Called every loop iteration. */
    virtual void update() = 0;

    /**
     * @brief Place subsystem in a safe, stopped state.
     *
     * Default implementation is a no-op. Override to cut power, retract
     * mechanisms, or otherwise make the subsystem safe on demand.
     */
    virtual void stop() {}

    /**
     * @brief Report whether the subsystem is operating nominally.
     * @return `true` if healthy; `false` if a fault has been detected.
     */
    virtual bool isHealthy() const { return true; }

protected:
    Subsystem() = default;
};

#endif
