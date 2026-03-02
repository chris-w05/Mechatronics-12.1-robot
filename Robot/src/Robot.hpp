
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
#include "Autonomous/Steps/FollowLineStep.hpp"
#include "Autonomous/Steps/DriveRadiusAtVelocity.hpp"
#include "Autonomous/Steps/DriveLineUntilWall.hpp"


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
                  DISTANCE_SENSOR_PIN, LINE_SENSOR_PINS),
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
            serialComs.update();

            // If a command is available, handle it (global commands are always accepted)
            
            if (serialComs.hasCommand())
            {
                // fetch the full command string (this clears the hasCommand flag)
                const char *raw = serialComs.getCommand();
                Serial.print("RCV: ");
                Serial.println(raw);

                char cmdChar = 0;
                float param = 0.0f;
                bool paramValid = false;

                if (serialComs.parseCmdWithOptionalFloat(raw, cmdChar, param, paramValid))
                {
                    // Handle only true global commands here
                    handleGlobalCommand(cmdChar);

                    // Only forward to serial-test handler when we're actually in SERIAL_TEST
                    if (mode == SERIAL_TEST)
                    {
                        if (paramValid)
                            handleSerialTestCommand(cmdChar, param, paramValid);
                        else
                            handleSerialTestCommand(cmdChar);
                    }
                }
                else
                {
                    serialComs.send("Invalid command format");
                }
            }

            //Global states
            switch (mode)
            {
            case AWAIT:
                for (int i = 0; i < subsystemCount; ++i)
                {
                    subsystems[i]->update();
                }
                break;

            case AUTONOMOUS:
                // In autonomous mode we want subsystems and the autonomous planner to run
                for (int i = 0; i < subsystemCount; ++i)
                {
                    subsystems[i]->update();
                }

                autonomous.update();
                break;

            case SERIAL_TEST:
                // Update all subsystems so that commands (e.g. miner.mine()) actually take effect
                for (int i = 0; i < subsystemCount; ++i){
                    subsystems[i]->update();
                }

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
                    float speed = 20;
                    // autonomous.add(new DriveDistance(drive, 10, speed));
                    // autonomous.add(new DriveRadiusAtVelocity(drive, speed, 30, 30));

                    // autonomous.add(new DriveDistance(drive, 6, 10));
                    autonomous.add(new DriveDistance(drive, 16, speed));
                    autonomous.add(new DriveRadiusAtVelocity(drive, speed, -18, 14.96649)); //was 11
                    autonomous.add(new DriveRadiusAtVelocity(drive, speed, 36, 26));
                    autonomous.add(new DriveDistance(drive, 23.5, speed));
                    autonomous.add(new DriveRadiusAtVelocity(drive, 10, -8, 11));
                    autonomous.add(new DriveLineToWallStep(drive, 10, 200));
                    // autonomous.add(new FollowLineStep(drive, 10, 160));
                    // autonomous.add(new FireStep(shooter, 30000, false));
                    autonomous.start();
                    Serial.println("Autonomous started.");
                }
                break;

            case '=':
                // Stop everything
                for (int i = 0; i < subsystemCount; ++i)
                {
                    subsystems[i]->stop();
                }

                break;

            case 'a':
                // start autonomous immediately
                mode = AUTONOMOUS;
                autonomous.reset();
                autonomous.start();

                break;

            default:
                // Unknown as a global command; caller will forward to serial-test handler when appropriate.
                if (mode != SERIAL_TEST)
                {
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
         * - 'D' : Start driving
         * - 'W' : Follow radius
         * - 'l, r, d' : stop driving
         * - 'E' : Exit to AWAIT state
         * - 'P' : Hold shooter position
         * - 'p' : Hold shooter position (different location)
         * - 'F' : Fire shooter
         * - 'f' : turn off shooter motor
         * - 'H' : print help
         */
        void handleSerialTestCommand(char cmd)
        {
            switch (cmd)
            {
            case 'M':
                miner.startMiningIndefinitely();
                Serial.println("Miner: mine() called.");
                break;

            case 'm':
                autonomous.stop();
                miner.store();
                Serial.println("Miner: stopMining() called.");
                break;

            case 'L':
                // drive.hardSetSpeed(150, -150);
                drive.followRadiusAtVelocity(10, -18);
                // drive.setSpeed(20);
                Serial.println("Drive: turn left called.");
                break;

            case 'R':
                drive.followRadiusAtVelocity(10, 18);
                // drive.setSpeed(20);
                Serial.println("Drive: turn right called.");
                break;

            case 'Q':
                // drive.setSpeed(10.0);
                autonomous.clear();
                // autonomous.add(new DriveDistance(drive, 30.0f, 3.0f));
                // autonomous.add(new DriveArc(drive, 2 * PI, .5f, 0.0f, false));
                // autonomous.add(new DriveDistance(drive, -10.0f, -3.0f));
                // autonomous.start();
                drive.followLine(5);
                Serial.println("Drive: Close loop control called.");
                break;
            case 'W':
                drive.followRadiusCCW( .5, 8);
                Serial.println("Drive: Close loop turning called.");
                break;
            case 'T':
                Serial.println(drive.getAccumulatedHeading());
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

            case 'P':
                shooter.stopFiring();
                shooter.prime();
                Serial.println("Shooter: holdPosition() called.");
                break;

            case 'p':
                shooter.stopFiring();
                shooter.holdPosition(1.1);
                Serial.println("Shooter: holdPosition() called.");
                break;

            case 'F':
                // autonomous.clear();
                // autonomous.add(new FireStep(shooter, 300000, true));
                // autonomous.start();
                shooter.fire();
                Serial.println("Shooter: fire() called.");
                break;
            
            case 'f':
                autonomous.stop();
                Serial.println("Shooter: stopFiring() called.");
                break;
            
            case '1':
                Serial.print("Current distance sensor distance: ");
                Serial.println(drive.getDistanceSensorReading());
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

        /**
         * Overload for handleSerialTestCommand for when a parameter is passed with character
         */
        void handleSerialTestCommand(char cmd, float param, bool paramValid)
        {
            switch (cmd)
            {
            case 'D':
                if (paramValid)
                {
                    // param is the desired speed (units you choose)
                    drive.setSpeed(param);
                    char buf[48];
                    snprintf(buf, sizeof(buf), "Drive: start() at speed %.2f", (double)param);
                    serialComs.send(buf);
                }
                else
                {
                    // no parameter: use existing safe default
                    drive.setSpeed(10);
                    serialComs.send("Drive: start() at default speed 10");
                }
                break;
            case 'I':
                if (paramValid)
                {
                    // param is the desired speed (units you choose)
                    drive.hardSetSpeed(param);
                    char buf[48];
                    snprintf(buf, sizeof(buf), "Drive: hardset speed at speed %.2f", (double)param);
                    serialComs.send(buf);
                }
                else
                {
                    // no parameter: use existing safe default
                    drive.hardSetSpeed(120);
                    serialComs.send("Drive: hardsetspeed at default speed 120");
                }
                break;
            case 'O':
                if (paramValid)
                {
                    drive.followLineHardset(param);
                    serialComs.send("Following line");
                }
                else{
                    drive.followLine(4);
                }
                break;
            case 'C':
                if (paramValid)
                {
                    drive.apporachDistance(param);
                    char buf[48];
                    snprintf(buf, sizeof(buf), "Drive: driving to wall distance of  %.2f", (double)param);
                    serialComs.send(buf);
                }
                else{
                drive.apporachDistance(10.0);
                serialComs.send("Drive: sapproachDistance at default 10.0cm");
            }
            break;

            case 'l':
            case 'r':
            case 'd':
                autonomous.stop();
                drive.hardSetSpeed(0);
                serialComs.send("Drive: stop() called.");
                break;

                // ... keep other cases unchanged, or add param-aware behavior for other commands.

            default:
                // If you want backward compatibility: call old handler if exists
                // handleSerialTestCommand(cmd); // only if you kept original variant
                break;
            }
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
