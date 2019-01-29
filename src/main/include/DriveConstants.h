///DRIVE GAINS///
#ifndef SRC_DRIVECONSTANTS_H_
#define SRC_DRIVECONSTANTS_H_

//must all be const

//Teleop

const double K_P_RIGHT_VEL = 0.0; //no gear shift
const double K_D_RIGHT_VEL = 0.0;

const double K_P_LEFT_VEL = 0.0;
const double K_D_LEFT_VEL = 0.0;

const double K_P_YAW_VEL = 0.0;
const double K_D_YAW_VEL = 0.0;

const double K_P_YAW_HEADING_POS = 0.0; //special controllers
const double K_D_VISION_POS = 0.0;

//		Auton

const double K_P_YAW_AU = 0.0; //gets sent to Controller()
const double K_D_YAW_AU = 0.0;

const double K_P_YAW_DIS = 0.0; //used in AutonDrive()
const double K_I_YAW_DIS = 0.0;
const double K_D_YAW_DIS = 0.0;

const double K_P_RIGHT_DIS = 0.0;
const double K_I_RIGHT_DIS = 0.0;
const double K_D_RIGHT_DIS = 0.0;

const double K_P_LEFT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;

///ROBOT DRIVE CONSTANTS///
//TODO: find empirically
const double MAX_FPS = 15.0; //used in auton pathfinder
const double FF_SCALE = 0.7; //auton

const double WHEEL_DIAMETER = 6.0; //inches, for fps for auton
const double TICKS_PER_ROT = 1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders
const double TICKS_PER_FOOT = 1315.0; //auton

const double MAX_Y_RPM = 550.0;
const double ACTUAL_MAX_Y_RPM = 625.0;
const double MAX_YAW_RATE = 20.0;

//misc

const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm

////////////not used this year

//teleop
const double K_P_RIGHT_VEL_LOW = 0.00; //low gear
const double K_D_RIGHT_VEL_LOW = 0.000;

const double K_P_LEFT_VEL_LOW = 0.00;
const double K_D_LEFT_VEL_LOW = 0.000;

const double K_P_YAW_VEL_LOW = 0.0;
const double K_D_YAW_VEL_LOW = 0.0;

const double K_P_RIGHT_VEL_HIGH = 0.00; //high gear
const double K_D_RIGHT_VEL_HIGH = 0.00;

const double K_P_LEFT_VEL_HIGH = 0.00;
const double K_D_LEFT_VEL_HIGH = 0.0;

const double K_P_YAW_VEL_HIGH = 0.0;
const double K_D_YAW_VEL_HIGH = 0.000;

const double K_P_KICK_VEL = 0.00; //HDrive
const double K_D_KICK_VEL = 0.0;
const double K_F_KICK_VEL = 0.0;

//auto
const double K_P_KICKER_DIS = 0.0;
const double K_I_KICKER_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0;

//robot drive constants
const double MAX_Y_RPM_LOW = 550.0;
const double MAX_Y_RPM_HIGH = 1250.0;
const double ACTUAL_MAX_Y_RPM_LOW = 625.0;
const double ACTUAL_MAX_Y_RPM_HIGH = 1300.0;

const double MAX_X_RPM = 0.0; //for HDrive

const double MAX_YAW_RATE_LOW = 20.0; //max angular velocity divided by the max rpm multiplied by set max rpm
const double MAX_YAW_RATE_HIGH = 20.0;

const double MAX_KICK_FPS = ((MAX_X_RPM * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
const int Kv_KICK = (1 / MAX_KICK_FPS);

const double UP_SHIFT_VEL = 0.0; // (24/14) *9370 //RPM
const double DOWN_SHIFT_VEL = 0.0; //will be less than up shift vel (14/56) *9370 //RPM


#endif
