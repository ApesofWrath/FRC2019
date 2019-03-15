#ifndef SRC_DRIVECONSTANTS_H_
#define SRC_DRIVECONSTANTS_H_

#include "Constants.h"

#define PRACTICE 0 //0 uses the consts on the bottom //comp gains work ok on practice bot too

#if PRACTICE //bot still w/o superstructure

//Teleop

const double reverse_output = -1.0;

const double K_P_RIGHT_VEL = 0.003;//0.0001; //no gear shift
const double K_D_RIGHT_VEL = 0.0;// 0.0005;

const double K_P_LEFT_VEL = 0.003; //voltage compensation //ff //p
const double K_D_LEFT_VEL = 0.0;

const double K_P_YAW_VEL = 13.0;
const double K_D_YAW_VEL = 0.0;

const double K_P_YAW_HEADING_POS = 0.0; //special controllers - yaw vision
const double K_D_VISION_POS = 0.0;

//Auton

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

const double MAX_VEL_VIS = 3.0; //m/s for pathfinder
const double MAX_ACC_VIS = 1.0;
const double MAX_JERK_VIS = 10000.0;

//Drive maxes

const double ACTUAL_MAX_Y_RPM_AUTON = 10.0;
const double ACTUAL_MAX_Y_RPM_L_F = 600.0;
const double ACTUAL_MAX_Y_RPM_L_B = 595.0;
const double ACTUAL_MAX_Y_RPM_R_F = 590.0;
const double ACTUAL_MAX_Y_RPM_R_B = 595.0;

const double MAX_Y_RPM = 595.0; //smallest actual max
const double ACTUAL_MAX_Y_RPM = 600.0;
const double MAX_YAW_RATE = 11.4;
const double MAX_FPS = 15.0; //used in auton pathfinder

const double FF_SCALE = 0.7; //auton

#else //1st bot to have superstructure

//Teleop

const double reverse_output = 1.0;

const double K_P_RIGHT_VEL = 0.003;//0.0001; //no gear shift
const double K_D_RIGHT_VEL = 0.0;// 0.0005;

const double K_P_LEFT_VEL = 0.003; //voltage compensation //ff //p
const double K_D_LEFT_VEL = 0.0;

const double K_P_YAW_VEL = 13.0;
const double K_D_YAW_VEL = 0.0;

const double K_P_YAW_HEADING_POS = 0.0; //special controllers - yaw vision
const double K_D_VISION_POS = 0.0;

//Auton

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

const double MAX_VEL_VIS = 3.0; //m/s for pathfinder
const double MAX_ACC_VIS = 1.0;
const double MAX_JERK_VIS = 10000.0;

//Drive maxes

const double ACTUAL_MAX_Y_RPM_AUTON = 500.0;
const double ACTUAL_MAX_Y_RPM_L_F = 610.0;
const double ACTUAL_MAX_Y_RPM_L_B = 570.0;
const double ACTUAL_MAX_Y_RPM_R_F = 580.0;
const double ACTUAL_MAX_Y_RPM_R_B = 580.0;

const double MAX_Y_RPM = 550.0;
const double ACTUAL_MAX_Y_RPM = 550.0;
const double MAX_YAW_RATE = 11.4;
const double MAX_FPS = 10.0;//; //used in auton pathfinder

const double FF_SCALE = 0.7; //auton

#endif //both bots

const double WHEEL_DIAMETER = 6.0; //inches, for fps for auton
const double TICKS_PER_ROT = 4096;//1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders
const double TICKS_PER_FOOT = 1315.0; //auton
const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm

#endif
