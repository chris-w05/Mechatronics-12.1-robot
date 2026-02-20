
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
#include "Autonomous/Steps/FireStep.hpp"
#include "Autonomous/Steps/MineBlock.hpp"
#include "Autonomous/Steps/DelayStep.hpp"


class Robot{

    

    public:

        /**
         * Robot constructor
         */
        Robot()
            : drive(
                  LEFT_ENCODER_A, LEFT_ENCODER_B,
                  RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                  drivePins,
                  DISTANCE_SENSOR_PIN, LINE_SENSOR_START_PIN),
              miner(MINER_SERVO_PIN),
              shooter(SHOOTER_ENCODER_A, SHOOTER_ENCODER_B,
                    shooterPins),
              serialComs(Serial2),
              planner(drive, miner, shooter)
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

            // autonomous.add(new ReplanStep(&planner, &Planner::planThunk));
        }

        void init()
        {
            Serial.println("Starting robot init");
            for (int i = 0; i < subsystemCount; ++i)
            {
                if (subsystems[i])
                {
                    subsystems[i]->init();
                }
            }

            Serial.println("Subsystems initialized");
            // autonomous.start();
        }

        void update()
        {
            // Always update SerialComs first so commands are captured promptly
            // serialComs.update();

            // If a command is available, handle it (global commands are always accepted)

            if (serialComs.hasCommand())
            {
                char cmd = serialComs.getCommandChar();
                
                Serial.println(cmd);
                if (cmd != 0)
                {
                    handleGlobalCommand(cmd);
                }
            }


            //Global states
            switch (mode)
            {
            case AWAIT:
                for (int i = 0; i < subsystemCount; ++i)
                    subsystems[i]->update();
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

                autonomous.update();
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

            // Theser are test states to test individual cases of robot behavior. 
            switch (cmd)
            {
            case 'S':
                if (mode != SERIAL_TEST)
                {
                    mode = SERIAL_TEST;
                    Serial.println("Entered SERIAL_TEST mode. Send 'H' for help.");
                }
                break;

            case 'A':
                // start autonomous immediately
                if (mode != AUTONOMOUS)
                {
                    mode = AUTONOMOUS;
                    autonomous.start();
                    Serial.println("Autonomous started.");
                }
                break;

            case 'a':
                // start autonomous immediately
                mode = AUTONOMOUS;
                autonomous.reset();
                Serial.println("resetting");
                
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
                autonomous.clear();
                autonomous.add(new MineBlockStep(miner, 5));
                autonomous.start();
                Serial.println("Miner: mine() called.");
                break;

            case 'm':
                autonomous.stop();
                miner.store();
                Serial.println("Miner: stopMining() called.");
                break;

            case 'D':
                // Conservative default: stop drive and prompt user how to implement a diagnostic pulse safely.
                // autonomous.clear();
                // autonomous.add(new DriveDistance(drive, 10.0f, 3.0f));
                // autonomous.add(new DriveArc(drive, 2 * PI, .5f, 0.0f, false));
                // autonomous.add(new DriveDistance(drive, -10.0f, -3.0f));
                // autonomous.start();
                drive.hardSetSpeed(100);
                // drive.setSpeed(20);
                Serial.println("Drive: start() called.");
                break;

            case 'L':
                drive.hardSetSpeed(100, -100);
                // drive.setSpeed(20);
                Serial.println("Drive: turn left called.");
                break;

            case 'R':
                drive.hardSetSpeed(-100, 100);
                // drive.setSpeed(20);
                Serial.println("Drive: turn right called.");
                break;

            case 'Q':
                // drive.setSpeed(10.0);
                autonomous.clear();
                // autonomous.add(new DriveDistance(drive, 30.0f, 3.0f));
                autonomous.add(new DriveArc(drive, 2 * PI, .5f, 0.0f, false));
                // autonomous.add(new DriveDistance(drive, -10.0f, -3.0f));
                autonomous.start();
                Serial.println("Drive: Close loop control called.");
                break;
            case 'W':
                drive.followRadiusCCW( .5, 0.0);
                Serial.println("Drive: Close loop turning called.");
                break;
            case 'T':
                break;
            case 'q':
                drive.setSpeed(0.0);
                Serial.println("Closed loop on 0 velocity called.");
                break;
            case 'l':
            case 'r':
            case 'd':
                autonomous.stop();
                drive.hardSetSpeed(0);
                Serial.println("Drive: stop() called.");
                break;
            case 'E':
                // Exit serial testing and go back to awaiting mode (stop subsystems if needed)
                autonomous.stop();
                mode = AWAIT;
                Serial.println("Exited SERIAL_TEST. Back to AWAIT.");
                break;

            case 'F':
                // autonomous.clear();
                // autonomous.add(new FireStep(shooter, 300000, true));
                // autonomous.start();
                shooter.fireHardSet(255);
                Serial.println("Shooter: fire() called.");
                break;
            
            case 'f':
                autonomous.stop();
                Serial.println("Shooter: stopFiring() called.");
                break;

            case 'H':
                // Print help / list of test commands
                Serial.println("SERIAL_TEST commands:");
                Serial.println("  M : start miner");
                Serial.println("  m : stop miner");
                Serial.println("  D : drive diagnostic (safe default: stop; see code comments)");
                Serial.println("  A : start autonomous");
                Serial.println("  E : exit SERIAL_TEST -> AWAIT");
                Serial.println("  H : this help");
                break;
            default:
            {
                
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
