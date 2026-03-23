# Programmer Guide

This guide is for both:

- experienced embedded/OOP developers who need precise implementation details, and
- new contributors (including people still learning object-oriented programming) who need a clear mental model.

## 1) Quick Start (Build, Upload, Monitor)

From the project root:

```bash
platformio run --environment megaADK
platformio run --target upload --environment megaADK
platformio device monitor --baud 115200
```

Environment configuration is in `platformio.ini` (`env:megaADK`).

---

## 2) How the Program Runs

### Startup sequence

1. `setup()` starts serial ports.
2. `robot.init()` initializes all subsystems.
3. Command help is printed to serial.

### Main loop sequence

Each loop, `robot.update()` does:

1. Read USB and Serial2 command channels.
2. Parse and dispatch command(s).
3. Update every subsystem.
4. Run autonomous scheduler when mode permits.

If you are new to OOP: think of each subsystem as an object that has its own internal state and behavior, but all of them are coordinated by one parent object (`Robot`).

---

## 3) Core Code Map

- `src/main.cpp`: Arduino entrypoints only.
- `src/Robot.hpp`: top-level robot object and loop orchestration.
- `src/Robot_Commands.hpp`: command parser and handlers.
- `src/subsystems/`: high-level mechanism logic.
- `src/Devices/`: low-level hardware wrappers.
- `src/Autonomous/`: scheduler and step abstractions.
- `src/Autonomous/Steps/`: concrete autonomous actions.
- `src/utils/`: control math and strategy helpers.
- `include/Config.hpp`: pins/constants/tuning.

Rule of thumb:

- Put "what the robot should do" in subsystems/autonomous steps.
- Put "how a specific sensor/actuator is read/written" in `Devices`.

---

## 4) Working With Subsystems

Every subsystem inherits `Subsystem` and should implement:

- `init()`
- `update()`
- `stop()` (recommended safe-state behavior)

### Existing subsystems

- `Drive`: drivetrain + line/distance sensor-based modes.
- `Miner`: mining/ramp servo state machine.
- `Shooter`: encoder-based shooter + optional auto-fire logic.
- `SerialComs`: line-based command reception over `Serial2`.

### Adding a new subsystem

1. Create `src/subsystems/NewSubsystem.hpp` inheriting `Subsystem`.
2. Add any low-level I/O wrappers under `src/Devices/` if needed.
3. Add a member in `Robot` and construct it in `Robot` constructor.
4. Register it in the `subsystems[]` list so `init/update/stop` are called.
5. Add any serial commands for it in `Robot_Commands.hpp`.

Keep `update()` non-blocking. Long waits (`delay`) will harm control responsiveness.

---

## 5) Using the Command Interface

Two input channels are supported:

- USB serial (`Serial`)
- Radio/aux serial (`Serial2` through `SerialComs`)

### Modes

- `AWAIT`: default/idle behavior.
- `SERIAL_TEST`: direct manual command testing.
- `AUTONOMOUS`: autonomous routine execution.

### Typical workflow during bring-up

1. Send `S` to enter serial test mode.
2. Verify each subsystem command in isolation.
3. Use `A` to run autonomous.
4. Use `stop` for emergency-safe halt.

### Command style

Parser accepts a command token plus up to two float parameters. Delimiters can be spaces, commas, colons, and tabs.

Examples:

- `Drive 12`
- `Line 24 10`
- `Arc 18 90`
- `SetPID 350 0`

---

## 6) Autonomous System (How to Extend It)

`AutoStep` is the base interface for every action in autonomous:

- `start()` runs once when step begins
- `update()` runs every loop while active
- `isFinished()` tells scheduler when to advance
- `end()` runs once on completion/interruption

`AutonomousRoutine` executes one step at a time in sequence.

### Add a new autonomous step

1. Create a class in `src/Autonomous/Steps/` inheriting `AutoStep`.
2. Pass subsystem references in constructor.
3. Put one-time command in `start()`.
4. Put continuous logic in `update()`.
5. Define robust completion in `isFinished()`.
6. Put cleanup/safe-stop in `end()`.
7. Add step insertion in `Robot_Commands.hpp` (for `A`) or in planner flow.

### Replanning path (advanced)

If you need adaptive behavior (decision after sensor result), use:

- `Planner` to produce next step from state.
- `ReplanStep` to repeatedly ask planner and run results.

This pattern reduces hardcoded giant command chains.

---

## 7) Tuning and Hardware Changes

### Where to change pins

- `include/Config.hpp`

### Where to tune control

- PID values and feedforward functions in `include/Config.hpp`
- mode/control behavior in `src/subsystems/Drive.hpp` and `src/subsystems/Shooter.hpp`

### When hardware changes

1. Update pin constants in `Config.hpp`.
2. Rebuild and verify each subsystem in `SERIAL_TEST` mode.
3. Validate autonomous afterward.

---

## 8) Ownership, Memory, and Safety

The autonomous framework uses raw pointers in several places. Be careful about ownership:

- `AutonomousRoutine` owns/deletes steps added to it.
- `CompositeStep` and `ParallelStep` also own/deallocate children.
- `ReplanStep` is explicitly non-owning.

Because this is a microcontroller project:

- avoid unnecessary dynamic allocation at runtime,
- avoid blocking calls,
- make cleanup explicit in `end()` and `stop()`.

---

## 9) Debugging Checklist

When behavior is wrong, use this order:

1. Confirm wiring/pins in `Config.hpp`.
2. Confirm mode transitions (`S`, `A`, `E`, `stop`) through serial output.
3. Test subsystem in isolation with serial commands.
4. Verify `isFinished()` conditions for autonomous steps.
5. Check whether a step leaves subsystem in a conflicting mode after `end()`.

Helpful principle:

- If motion is wrong, inspect `Drive` mode and target values.
- If mechanism timing is wrong, inspect subsystem state machine timers.
- If autonomous stalls, inspect current step's `isFinished()` condition.

---

## 10) Documentation and Tests

- Generate API docs with Doxygen:

```bash
doxygen Doxyfile
open html/index.html
```

- PlatformIO unit test directory exists at `test/`.
  - Add module-level tests there as project grows.

---

## 11) Contributor Conventions

1. Keep `main.cpp` thin; put behavior in classes.
2. Keep each class focused on one responsibility.
3. Prefer explicit names (`startMiningIndefinitely`) over magic numbers.
4. Keep serial commands stable; update help text when adding commands.
5. Keep constants in `Config.hpp` instead of scattering literals.
6. Ensure every new subsystem/step has a safe stop path.

If you are newer to OOP: this project already uses a solid pattern you can follow. Copy the shape of existing classes (`init`, `update`, `stop`, clear member state) and you will stay consistent with the architecture.
