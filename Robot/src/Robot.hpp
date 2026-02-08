
#include "Autonomous/Steps/DriveDistance.hpp"
#include "Autonomous/Steps/CompositeStep.hpp"
#include "Autonomous/Planner.hpp"
#include "Autonomous/ReplanStep.hpp"
#include "subsystems/Subsystem.h"
#include "subsystems/Drive.hpp"
#include "Autonomous/AutonomousRoutine.hpp"
#include "Autonomous/Planner.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/SerialComs.hpp"
#include "subsystems/Shooter.hpp"
#include "SoftwareSerial.h"
#include "Config.hpp"
#include "Devices/MotorController.hpp"

SoftwareSerial serialMain(SERIAL_RX, SERIAL_TX);

class Robot{

    

    public:
        Robot()
            : drive(
                LEFT_ENCODER_A, LEFT_ENCODER_B,
                RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                LEFT_MOTOR_PWM, LEFT_MOTOR_DIR,
                RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                DISTANCE_SENSOR_PIN
                ),
            miner(MINER_SERVO_PIN),
            shooter(SHOOTER_MOTOR_PWM1, SHOOTER_MOTOR_PWM2, SHOOTER_MOTOR_ENABLE, SHOOTER_MOTOR_ENABLE2, -1, -1, -1, 5.0, MotorController::DriverType::L298N ),
            serialComs(serialMain),
            planner(drive)
        {
            //Sets up subsystems
            subsystems[subsystemCount++] = &drive;
            subsystems[subsystemCount++] = &miner;
            subsystems[subsystemCount++] = &shooter;
            subsystems[subsystemCount++] = &serialComs;

            //Create autonomous routine
            //Autonomous is a queue of AutoSteps
            //For now, its comprised of one ReplanStep, which asks Planner's planNext what the next step it should do should be
            //planNext consults utils/Strategy to find which group of steps should be done next - i.e. a reject sequence, driving and scoring, mining, etc
            
            //Planner can be thought of the implementation of Strategy for autonomous decision making. 
            //It turns objectives from Strategy into sequences of actions for autonomous running

            autonomous.add(new ReplanStep(&planner, &Planner::planThunk));
        }

        void init()
        {
            serialMain.begin(SERIAL_BAUD_RATE);

            for (Subsystem *s : subsystems)
                s->init();

            autonomous.start();
        }

        void update()
        {
            // Always update SerialComs first so commands are captured promptly
            serialComs.update();

            // If a command is available, handle it (global commands are always accepted)
            if (serialComs.hasCommand())
            {
                char cmd = serialComs.getCommandChar();
                if (cmd != 0)
                {
                    handleGlobalCommand(cmd);
                }
            }

            switch (mode)
            {
            case AWAIT:
                // do nothing except wait for the 'S' command (handled in global handler)
                break;

            case AUTONOMOUS:
                // In autonomous mode we want subsystems and the autonomous planner to run
                for (int i = 0; i < subsystemCount; ++i)
                    subsystems[i]->update();

                autonomous.update();
                break;

            case SERIAL_TEST:
                // Update all subsystems so that commands (e.g. miner.mine()) actually take effect
                for (int i = 0; i < subsystemCount; ++i)
                    subsystems[i]->update();

                // In SERIAL_TEST mode we also accept single-character commands.
                // Those are handled in handleSerialTestCommand(); we poll serialComs.hasCommand() above.
                break;
            } // switch
            
        }

    private:
        /**
         * Global commands are handled even from AWAIT state.
         * - 'S' : enter SERIAL_TEST
         * - 'A' : start Autonomous now
         * - others: forwarded to serial test handler when in SERIAL_TEST
         */
        void handleGlobalCommand(char cmd)
        {
            switch (cmd)
            {
            case 'S':
                if (mode != SERIAL_TEST)
                {
                    mode = SERIAL_TEST;
                    serialComs.send("Entered SERIAL_TEST mode. Send 'H' for help.");
                }
                break;

            case 'A':
                // start autonomous immediately
                if (mode != AUTONOMOUS)
                {
                    mode = AUTONOMOUS;
                    autonomous.start();
                    serialComs.send("Autonomous started.");
                }
                break;

            default:
                // If we're already in SERIAL_TEST, forward the command to the SERIAL_TEST handler.
                if (mode == SERIAL_TEST)
                {
                    handleSerialTestCommand(cmd);
                }
                else
                {
                    // Unrecognized in AWAIT/AUTONOMOUS — optionally echo
                    // This keeps the robot responsive if a user accidentally types while waiting.
                    char buf[48];
                    snprintf(buf, sizeof(buf), "Unknown/global cmd '%c' (S start serial, A start auton)", cmd);
                    serialComs.send(buf);
                }
                break;
            }
        }

        /**
         * Commands that are available while in SERIAL_TEST mode.
         *
         * - 'M' : start miner
         * - 'm' : stop miner
         * - 'D' : request drive diagnostic (this is conservative: calls drive.stop() and
         *         prints a hint. See TODO below to implement a real drive pulse)
         * - 'E' : exit SERIAL_TEST back to AWAIT (stops miner)
         * - 'H' : print help
         */
        void handleSerialTestCommand(char cmd)
        {
            switch (cmd)
            {
            case 'M':
                miner.mine();
                serialComs.send("Miner: mine() called.");
                break;

            case 'm':
                miner.stopMining();
                serialComs.send("Miner: stopMining() called.");
                break;

            case 'D':
                // Conservative default: stop drive and prompt user how to implement a diagnostic pulse safely.
                drive.stop(); // safe stop
                serialComs.send("Drive: stop() called. To run a diagnostic pulse, implement Drive::runDiagnosticPulse() and call it here.");
                // If you want a sample diagnostic pulse, add this to your Drive class:
                // void runDiagnosticPulse() { setMotorOpenLoopLeft(100); setMotorOpenLoopRight(100); delay(200); stop(); }
                break;

            case 'd':

                break;
            case 'E':
                // Exit serial testing and go back to awaiting mode (stop subsystems if needed)
                miner.stopMining();
                mode = AWAIT;
                serialComs.send("Exited SERIAL_TEST. Back to AWAIT.");
                break;

            case 'F':
                shooter.fire();
                serialComs.send("Shooter: fire() called.");
                break;
            
            case 'f':
                shooter.stopFiring();
                serialComs.send("Shooter: stopFiring() called.");
                break;

            case 'H':
            default:
            {
                // Print help / list of test commands
                serialComs.send("SERIAL_TEST commands:");
                serialComs.send("  M : start miner");
                serialComs.send("  m : stop miner");
                serialComs.send("  D : drive diagnostic (safe default: stop; see code comments)");
                serialComs.send("  A : start autonomous");
                serialComs.send("  E : exit SERIAL_TEST -> AWAIT");
                serialComs.send("  H : this help");
            }
            break;
            } // switch
        }

        enum RobotMode
        {
            AUTONOMOUS,
            SERIAL_TEST,
            AWAIT
        };

        Drive drive;
        Miner miner;
        Shooter shooter;
        SerialComs serialComs;

        Planner planner;
        AutonomousRoutine autonomous;
        Subsystem *subsystems[MAX_SUBSYSTEMS];
        RobotMode mode = AWAIT;
        int subsystemCount = 0;
};
