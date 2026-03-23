# OOP Crash Course for This Robot Project

This is a practical guide to object-oriented programming (OOP) as it is used in this robot codebase.

Audience:

- If you are new to OOP, this explains the ideas in plain language.
- If you already know OOP, this maps those ideas to concrete project patterns.

## 1) The Big Idea: Why OOP Here?

The robot has many parts that must run together:

- drivetrain
- miner
- shooter
- serial communication

OOP lets us model each part as an object with:

- state (its current data)
- behavior (functions it can run)

Then a top-level coordinator object runs all parts each loop.

Project example:

- [src/main.cpp](src/main.cpp) keeps Arduino code thin.
- [src/Robot.hpp](src/Robot.hpp) owns and coordinates subsystem objects.

---

## 2) Class and Object

Class: a blueprint.
Object: one real instance of that blueprint.

Example in this project:

- Class: `Drive`
- Object: `drive` member inside `Robot`

In [src/Robot.hpp](src/Robot.hpp), `Robot` creates one object for each major subsystem and stores them as members.

Why this helps:

- each subsystem keeps its own internal variables
- you can reason about one mechanism at a time

---

## 3) Encapsulation (Keep internals private)

Encapsulation means hiding implementation details and exposing a clean API.

Good pattern used in this code:

- Subsystem keeps most fields private.
- Other code asks it to do high-level actions.

Example:

- In [src/subsystems/Drive.hpp](src/subsystems/Drive.hpp), external code calls methods like `setSpeed`, `followRadiusAtVelocity`, or `approachDistance`.
- External code does not directly manipulate encoder internals or motor controller math.

Why this helps:

- safer changes later
- fewer accidental side effects
- easier debugging

---

## 4) Abstraction (Work at the right level)

Abstraction means exposing what to do, not every low-level detail of how.

Examples:

- `miner.startMining(10)` means hit the block 10 times.
- Caller does not need to know servo angle timing internals.

See:

- [src/subsystems/Miner.hpp](src/subsystems/Miner.hpp)
- [src/subsystems/Shooter.hpp](src/subsystems/Shooter.hpp)

This is great for team work: one person can improve internals without changing other files.

---

## 5) Inheritance and Interface Contracts

A core OOP pattern here is an abstract base class interface.

Project interface:

- [src/subsystems/Subsystem.h](src/subsystems/Subsystem.h)

`Subsystem` defines required behavior:

- `init()`
- `update()`
- optional `stop()`

Then concrete classes inherit from it:

- `Drive : public Subsystem`
- `Miner : public Subsystem`
- `Shooter : public Subsystem`
- `SerialComs : public Subsystem`

This is a contract: every subsystem can be initialized and updated the same way.

---

## 6) Polymorphism (Treat different classes uniformly)

Polymorphism means using base-class pointers/references to handle different derived classes through one interface.

Project example:

- In [src/Robot.hpp](src/Robot.hpp), robot stores subsystem pointers in an array of `Subsystem*`.
- Then one loop can call `subsystems[i]->update()` no matter which subsystem it is.

This removes repeated code and makes adding subsystems easier.

---

## 7) Composition (Build bigger objects from smaller ones)

Composition means classes contain other objects and use them.

Examples:

- `Robot` contains subsystem objects.
- `Drive` contains encoder, line sensor, distance sensor, odometry, motor controller.
- `Shooter` contains button, encoder, and motor controller.

See:

- [src/Robot.hpp](src/Robot.hpp)
- [src/subsystems/Drive.hpp](src/subsystems/Drive.hpp)
- [src/subsystems/Shooter.hpp](src/subsystems/Shooter.hpp)

This mirrors real hardware structure and keeps logic modular.

---

## 8) State Machines (Object behavior changes by mode)

Several classes are mode/state machines.

Examples:

- `Robot` mode: `AWAIT`, `SERIAL_TEST`, `AUTONOMOUS`
- `Drive` modes: straight, arc, line follow, hardset, distance, etc.
- `Miner` mode: lift, mining, off, store
- `Shooter` mode: off, position, velocity, auto

Each object switches behavior in `update()` based on mode.

Why this is OOP-friendly:

- mode and related timers live inside the object
- external code sets intent, object manages transitions

---

## 9) Autonomous as an OOP Framework

Autonomous uses interface-based design heavily.

Base step interface:

- [src/Autonomous/AutoStep.h](src/Autonomous/AutoStep.h)

Required methods:

- `start()`
- `update()`
- `isFinished()`
- `end()`

Scheduler:

- [src/Autonomous/AutonomousRoutine.hpp](src/Autonomous/AutonomousRoutine.hpp)

Concrete step classes in:

- [src/Autonomous/Steps](src/Autonomous/Steps)

This is polymorphism again: scheduler runs any step that satisfies the `AutoStep` contract.

---

## 10) Beginner-Friendly Example: Build a New Auto Step

Suppose you want a new step called `WaitForDistanceStep`.

Conceptually:

1. Constructor receives `Drive&` and target distance.
2. `start()` sends initial command if needed.
3. `update()` can monitor sensors.
4. `isFinished()` returns true when condition met.
5. `end()` stops or leaves subsystem in desired state.

Skeleton:

```cpp
class WaitForDistanceStep : public AutoStep {
public:
    WaitForDistanceStep(Drive& drive, float target)
        : _drive(drive), _target(target) {}

    void start() override {
        _drive.approachDistance(_target);
    }

    void update() override {
        // optional: debug or adaptive logic
    }

    bool isFinished() const override {
        return fabs(_drive.getDistanceSensorReading() - _target) < 0.5f;
    }

    void end() override {
        _drive.setSpeed(0);
    }

private:
    Drive& _drive;
    float _target;
};
```

This is classic OOP:

- inheritance from interface
- encapsulated state
- reusable object behavior

---

## 11) Device Layer vs Subsystem Layer

A very useful architectural OOP split in this project:

- Device classes in [src/Devices](src/Devices): low-level hardware wrappers
- Subsystems in [src/subsystems](src/subsystems): mechanism-level behavior

Example:

- `EncoderWrapper` reads encoder data.
- `Drive` decides how to use that data for control.

This separation keeps hardware details from leaking into high-level strategy logic.

---

## 12) Common OOP Mistakes to Avoid in This Project

1. Putting robot behavior directly in [src/main.cpp](src/main.cpp)
Keep this file minimal.

2. Exposing too many public member variables
Prefer private fields with clear public methods.

3. Duplicating control logic across files
Put shared logic in the right class once.

4. Blocking inside `update()`
Use timers/state checks rather than long delays.

5. Breaking ownership assumptions for autonomous pointers
Read ownership expectations before adding/deleting step pointers.

---

## 13) Practical Checklist When Writing New OOP Code

1. Decide which class should own the state.
2. Keep data private unless there is a strong reason not to.
3. Expose small, intention-revealing public methods.
4. Make `init`, `update`, and `stop` behavior explicit.
5. Add serial commands only at the command/router layer.
6. Put constants in [include/Config.hpp](include/Config.hpp), not magic literals.
7. Test subsystem in serial mode before using in autonomous.

---

## 14) Summary

The project uses OOP in a practical embedded style:

- interfaces for consistency (`Subsystem`, `AutoStep`)
- polymorphism for generic loops/schedulers
- composition to model real hardware modules
- state machines inside objects for reliable runtime behavior

If you follow these same patterns, your additions will integrate cleanly with the existing robot architecture.
