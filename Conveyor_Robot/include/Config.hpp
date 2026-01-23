#ifndef CONFIG_H
#define CONFIG_H
//----------------------------Pin allocation
// Encoder pins
static const int LEFT_ENCODER_A = 4;
static const int LEFT_ENCODER_B = 5;
static const int RIGHT_ENCODER_A = 6;
static const int RIGHT_ENCODER_B = 7;
static const int ARM_ROT_ENCODER_A = 4;
static const int ARM_ROT_ENCODER_B = 5;
static const int ARM_EXT_ENCODER_A = 6;
static const int ARM_EXT_ENCODER_B = 7;

// Motor pins
static const int LEFT_MOTOR_PWM = 8;
static const int LEFT_MOTOR_DIR = 9;
static const int RIGHT_MOTOR_PWM = 10;
static const int RIGHT_MOTOR_DIR = 11;
static const int ARM_ROT_MOTOR_PWM = 8;
static const int ARM_ROT_MOTOR_DIR = 9;
static const int ARM_EXT_MOTOR_PWM = 10;
static const int ARM_EXT_MOTOR_DIR = 11;

//Servo pins
static const int GRIPPER_SERVO_PIN = 10;
static const int INDEXER_SERVO_PIN = 12;

//Solenoid pins
static const int MINER_SOLENOID_PIN = 10;

//Serial pins
static const int SERIAL_TX = 2;
static const int SERIAL_RX = 3;
static const long SERIAL_BAUD_RATE = 115200;

//Sensor pins
static const int FRONT_LEFT_DIST_SENSOR_PIN = 41;
static const int BACK_LEFT_DIST_SENSOR_PIN = 41;
static const int FRONT_DIST_SENSOR_PIN = 41;
static const int BACK_DIST_SENSOR_PIN = 41;

//----------------------------------Software constants
static const int MAX_STEPS = 10; // Maximum allowed steps in autonomous. This prevents too much memory from being used
static const int MAX_SUBSYSTEMS = 10; 

//----------------------------------Physical constants
//Robot params
static const float DRIVETRAIN_WIDTH = 6;
static const int TICKS_PER_REV = 1024; //Number of ticks per motor revolution
static const float DRIVETRAIN_WHEEL_DIAMETER = 4; //in
static const float ARM_ROT_RATIO = .5; // rev/rotation - arm pos(rotations) = motors rotations * ARM_ROT_RATIO
static const float ARM_EXT_RATIO = 3.14 * 2; // mm/rotation - arm pos(mm) = motors rotations * ARM_EXT_RATIO

//----------------------------------Setpoints
//Arm extension distances
static const float ARM_EXT_DIST_CLOSE = 0;              //mm
static const float ARM_EXT_DIST_MIDDLE = 2.8 * 25.4;    //mm
static const float ARM_EXT_DIST_FAR = 2.8 * 25.4 * 2;   //mm

//Arm roation angles
struct ArmPreset
{
    float angle;      // degrees
    float extension; // whatever units your setPosition expects (encoder ticks or mm or percent)
};

// Named presets
enum ArmPresetId
{
    PRESET_MINE = 0,
    PRESET_SCORE_SHORT,
    PRESET_SCORE_MED,
    PRESET_SCORE_LONG,
    PRESET_COUNT
};


//These are currently meaningless
//Stored as {angle, extension}
static constexpr ArmPreset ARM_PRESETS[PRESET_COUNT] = {
    /*PRESET_STOW*/ {0, 10},
    /*PRESET_SCORE_LOW*/ {.25, 30},
    /*PRESET_SCORE_MED*/ {.4, 60},
    /*PRESET_SCORE_HIGH*/ {.5, 90}};

//Gripper constants
static const int GRIPPER_CLOSED_ANGLE = 90;             //deg
static const int GRIPPER_OPEN_ANGLE = 0;                //deg

//Miner constants
static const int MINER_HIT_RATE = 350;                  //ms
static const int SOLENOID_MAX_ON_TIME = 20;             //ms

//Indexer constants
static const int INDEXER_LEFT_ANGLE = 20;               //deg
static const int INDEXER_MIDDLE_ANGLE = 90;             //deg
static const int INDEXER_RIGHT_ANGLE = 170;             //deg







//----------------------------------PID constants
//drivetrain
static const float DRIVE_L_KP = .1;
static const float DRIVE_L_KI = 0.01;
static const float DRIVE_L_KD = 0;
static const float DRIVE_R_KP = .1;
static const float DRIVE_R_KI = 0.01;
static const float DRIVE_R_KD = 0;


//arm
static const float ARM_ROT_KP = .1;
static const float ARM_ROT_KI = 0.01;
static const float ARM_ROT_KD = 0;
//extensi
static const float ARM_EXT_KP = .1;
static const float ARM_EXT_KI = 0.01;
static const float ARM_EXT_KD = 0;

#endif
