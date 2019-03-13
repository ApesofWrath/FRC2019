///DRIVE GAINS///
#ifndef SRC_DRIVECONSTANTS_H_
#define SRC_DRIVECONSTANTS_H_

//must all be const

//Teleop

const double K_P_RIGHT_VEL = 0.002;//0.0001; //no gear shift
const double K_D_RIGHT_VEL = 0.0;// 0.0005;

const double K_P_LEFT_VEL = 0.002; //voltage compensation //ff //p
const double K_D_LEFT_VEL = 0.0;

const double K_P_YAW_VEL = 0.0;
const double K_D_YAW_VEL = 0.0;

const double K_P_YAW_HEADING_POS = 0.5; //special controllers TODO: change back
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
const double FF_SCALE = 1.0; //auton

const double WHEEL_DIAMETER = 6.0; //inches, for fps for auton
const double TICKS_PER_ROT = 4096;//1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders
const double TICKS_PER_FOOT = 1315.0; //auton

const double MAX_Y_RPM = 550.0;
const double ACTUAL_MAX_Y_RPM = 550.0;
const double MAX_YAW_RATE = 15.0;

const double MAX_Y_FPS_AUTON = MAX_Y_RPM * TICKS_PER_ROT / TICKS_PER_FOOT / 60;

//misc

const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm

///ROBOT DRIVE CONSTANTS///

const double MAX_VEL_VIS = 3.0; //m/s for pathfinder
const double MAX_ACC_VIS = 1.0;
const double MAX_JERK_VIS = 10000.0;

// const double MAX_FPS = 0.0; //used in auton pathfinder
// const double FF_SCALE = 0.0;
//
// const double WHEEL_DIAMETER = 6.0; //inches, for fps for auton
// const double TICKS_PER_ROT = 1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders
// const double TICKS_PER_FOOT = 1315.0; //auton
//
// const double MAX_Y_RPM = 0;
// const double ACTUAL_MAX_Y_RPM = 0;
// const double MAX_YAW_RATE = 0.0;


#endif
