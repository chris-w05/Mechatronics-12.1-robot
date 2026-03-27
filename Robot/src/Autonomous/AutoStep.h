/**
 * @file AutoStep.h
 * @brief Abstract interface that every autonomous step must implement.
 *
 * An `AutoStep` represents a single, self-contained action in an autonomous
 * routine (e.g. drive forward, mine a block, fire the shooter).  Steps are
 * sequenced and driven by `AutonomousRoutine` or `ReplanStep`.
 *
 * Lifecycle contract:
 *  1. `start()`     — called once before the step runs (initialise state).
 *  2. `update()`    — called every loop tick until `isFinished()` returns true.
 *  3. `isFinished()` — returns true when the step has completed.
 *  4. `end()`       — called once after the step finishes (cleanup).
 */
// AutoStep.h

//This is a template that all autonomous steps must follow for standardization. 
//All autonomous steps are also AutoSteps. 
#ifndef AUTO_STEP_H
#define AUTO_STEP_H


/**
 * @brief Pure-virtual base class for all autonomous steps.
 */
class AutoStep {
public:
    virtual ~AutoStep() = default;

    /** @brief Initialise any state needed before the step begins running. */
    virtual void start() = 0;

    /** @brief Advance the step by one control-loop tick. */
    virtual void update() = 0;

    /** @brief @return `true` when the step has completed its objective. */
    virtual bool isFinished() const = 0;

    /** @brief Clean up resources or subsystem state when the step ends. */
    virtual void end() = 0;
};

#endif
