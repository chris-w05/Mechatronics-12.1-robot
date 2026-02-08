
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
#include "SoftwareSerial.h"
#include "Config.hpp"

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
            serialComs(serialMain),
            planner(drive)
        {
            //Sets up subsystems
            subsystems[subsystemCount++] = &drive;
            subsystems[subsystemCount++] = &miner;
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
            switch (mode){
                case AWAIT:
                    break;
                case AUTONOMOUS:
                    for (Subsystem *s : subsystems)
                        s->update();

                    autonomous.update();
                    break;
                case SERIAL_TEST:
                    break;
            }


            
        }

    private:
        enum RobotMode
        {
            AUTONOMOUS,
            SERIAL_TEST,
            AWAIT
        };

        Drive drive;
        Miner miner;
        SerialComs serialComs;

        Planner planner;
        AutonomousRoutine autonomous;
        Subsystem *subsystems[MAX_SUBSYSTEMS];
        RobotMode mode = AWAIT;
        int subsystemCount = 0;
};
