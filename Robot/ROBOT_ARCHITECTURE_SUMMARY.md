# Robot Architecture Summary

## 1) High-Level Architecture

This codebase is organized around one top-level `Robot` object that owns all major subsystems:

- `Drive`
- `Miner`
- `Shooter`
- `SerialComs`

`main.cpp` is intentionally minimal:

1. `setup()` initializes serial ports and calls `robot.init()`.
2. `loop()` calls `robot.update()` every cycle.

Inside `Robot::update()`:

1. Read inbound commands from both serial channels:
   - USB serial (`Serial`, usually PC terminal)
   - XBee/secondary serial (`Serial2`, usually external controller)
2. Process at most one buffered command from each channel.
3. Call `update()` on every subsystem every loop.
4. If robot mode is `AUTONOMOUS` or `SERIAL_TEST`, call `autonomous.update()`.

This design keeps all timing and behavior deterministic around one central loop.

---

## 2) File and Responsibility Layout

- `src/main.cpp`:
  - Arduino entry points.
- `src/Robot.hpp`:
  - Top-level system wiring, subsystem ownership, mode state machine.
- `src/Robot_Commands.hpp`:
  - Serial command parser + command handlers.
- `include/Config.hpp`:
  - Pin mapping, physical constants, PID gains, and setpoints.
- `src/subsystems/*.hpp`:
  - Each major mechanism as a `Subsystem` implementation.
- `src/Autonomous/*`:
  - Generic autonomous scheduler + steps and planning utilities.
- `src/Devices/*`:
  - Hardware abstraction wrappers (encoders, sensors, motor/servo drivers).
- `src/utils/*`:
  - Math/control/state helpers (PID, odometry, Ramsete, strategy state).

---

## 3) Subsystem Control Model

All subsystems derive from `Subsystem` and follow the same lifecycle:

- `init()` called once at startup.
- `update()` called every loop.
- optional `stop()` for safe state.

### Drive

`Drive` is a mode-based controller with internal states such as:

- `STRAIGHT`, `ARC`, `STOPPED`
- `LINEFOLLOWING`, `LINEFOLLOWING_HARDSET`, `LINEFOLLOWING_DISTANCE`
- `HARDSET`, `DISTANCE`

Key behavior:

- Reads encoder data every cycle.
- Optionally reads line sensor/distance sensor only in modes that need them.
- Uses odometry and closed-loop motor control for distance/arc motion.
- Supports Ramsete correction for trajectory tracking.
- Exposes command-style methods (`setSpeed`, `followRadiusAtVelocity`, `followLineHardset`, `approachDistance`, etc.) to switch modes.

### Miner

`Miner` controls two servos:

- Miner arm servo (press/retract/store angles)
- Ramp servo (store/lift/passive angles)

It runs as a finite state machine (`LIFT`, `MINING`, `OFF`, `STORE`) and supports:

- Finite hit-count mining (`startMining(hits)`)
- Indefinite mining (`startMiningIndefinitely()`)
- Timeout safety to prevent endless run

### Shooter

`Shooter` controls one motor with encoder feedback and a block-detect switch.

Modes include:

- `OFF`
- `POSITION` (position PID-based prime/fire control)
- `VELOCITY`
- `AUTO` (auto-fire state machine)

Auto mode sequences through block detect, settle delay, fire, and re-prime timing.

### SerialComs

`SerialComs` receives newline-terminated command strings from `Serial2` and provides a non-blocking command buffer interface to `Robot`.

---

## 4) Robot Modes and Command Routing

The robot-level mode machine in `Robot` is:

- `AWAIT`: idle/standby
- `SERIAL_TEST`: direct testing mode for mechanism commands
- `AUTONOMOUS`: run autonomous routine

Global commands (available regardless of mode) include:

- `A`: build and start predefined autonomous step sequence
- `a`: reset and restart autonomous
- `S`: enter serial test mode
- `stop`: stop all subsystems and autonomous
- `Help`: print command reference

In `SERIAL_TEST`, additional mechanism commands are enabled (`Drive`, `Linefollow`, `Distance`, `Approach`, `Mine`, `Fire`, `SetPID`, etc.).

---

## 5) Autonomous Framework

### Core interfaces

- `AutoStep` defines a standard step interface:
  - `start()`
  - `update()`
  - `isFinished()`
  - `end()`

- `AutonomousRoutine` holds an ordered list of `AutoStep*`:
  - Runs exactly one current step at a time
  - Calls `start()` once when a step begins
  - Calls `update()` each loop
  - On completion, calls `end()` then starts the next step

### Common step types in this project

- Motion: `DriveDistance`, `DriveRadiusAngle`, `DriveRadiusAtVelocity`, `DriveLineToWallStep`, `FollowLineStep`
- Mechanism actions: `MineBlock`, `DeployRamp`, `FireStep`
- Flow composition: `DelayStep`, `CompositeStep`, `ParallelStep`, `DeferredStep`, `ReplanStep`

### How autonomous currently starts

On command `A`, `Robot` creates a hardcoded sequence of steps and starts the routine, for example:

1. Straight/arc drive steps
2. Align to wall/line
3. Deploy ramp
4. Mine block

This sequence is assembled at runtime by pushing step objects into `AutonomousRoutine`.

### Dynamic planning capability

The codebase also includes `Planner` + `ReplanStep` for planner-driven autonomous execution:

- `Planner::planNext()` returns the next step based on current phase/state.
- `ReplanStep` repeatedly requests, runs, and retires child steps.
- This supports adaptive behavior without hardcoding one fixed sequence.

---

## 6) Configuration and Tuning

`include/Config.hpp` centralizes most robot constants:

- Hardware pins
- Physical conversion constants (ticks-to-distance, geometry)
- Servo setpoint angles
- Timing constants
- PID constants and feedforward functions

This is the first place to update when hardware wiring or drivetrain tuning changes.

---

## 7) Control/Data Flow in One Loop

A useful mental model for one cycle of execution is:

1. Input: read serial command channels.
2. Intent: parse command and set subsystem targets/modes.
3. Control: each subsystem updates from sensors and applies outputs.
4. Coordination: autonomous scheduler advances step state.
5. Repeat.

Because all changes funnel through this loop, behavior remains predictable and easier to debug.

---

## 8) Practical Notes

- `MAX_STEPS` and `MAX_SUBSYSTEMS` limits are compile-time safety limits in `Config.hpp`.
- Some autonomous composition classes own/deallocate child steps; others (like `ReplanStep`) are intentionally non-owning. Ownership consistency matters to avoid leaks or double deletes.
- The project already includes Doxygen setup (`Doxyfile`) for generated API docs in `html/`.
