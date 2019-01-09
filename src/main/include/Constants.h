//GENERAL//

const double TIME_STEP = 0.02; //ms

//DRIVE GAINS//

//		Teleop
const double K_P_RIGHT_VEL_LOW = 0.001;
const double K_P_LEFT_VEL_LOW = 0.001;
const double K_D_RIGHT_VEL_LOW = 0.000;
const double K_D_LEFT_VEL_LOW = 0.000;
const double K_P_YAW_VEL_LOW = 30.0; //85.0;
const double K_D_YAW_VEL_LOW = 0.0;

const double K_P_RIGHT_VEL_HIGH = 0.001;
const double K_P_LEFT_VEL_HIGH = 0.001;
const double K_D_RIGHT_VEL_HIGH = 0.00;
const double K_D_LEFT_VEL_HIGH = 0.0;
const double K_P_YAW_VEL_HIGH = 10.0; //120.0;
const double K_D_YAW_VEL_HIGH = 0.000;

const double K_P_YAW_HEADING_POS_HD = 0.0;
const double K_D_VISION_POS_HD = 0.0;
const double K_P_YAW_HEADING_POS_WC = 0.01;
const double K_D_VISION_POS_WC = 0.0;

const double K_P_KICK_VEL = 0.00365; //0.00311
const double K_D_KICK_VEL = 0.0;
const double K_F_KICK_VEL = 1.0 / 400.0;

//		Auton
const double K_P_YAW_AU_HD = 0.0; //5.0;
const double K_D_YAW_AU_HD = 0.0; //0.085;
const double K_P_YAW_AU_WC = 0.0; //5.0;
const double K_D_YAW_AU_WC = 0.0; //0.085;

const double K_P_RIGHT_DIS = 0.15; //0.085; //0.1;
const double K_P_LEFT_DIS = 0.15; //0.085; // 0.1;//2.4
const double K_P_KICKER_DIS = 0.280;

const double K_I_RIGHT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_I_KICKER_DIS = 0.0;

const double K_D_RIGHT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0; //f

double K_P_YAW_DIS = 0.5; //0.2;3.3;//3.5; //1.5; //3.0 //was spending too much time making the turn and when it actually got to the shoot the gap part of the profile, the profile was already ahead of it
double K_I_YAW_DIS = 0.0; //3;//1;//0.0
double K_D_YAW_DIS = 0.0; //4.03.2;//4.0; //pd controller on yaw //20, p sum of p and d

///ROBOT DRIVE CONSTANTS///

const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm
double FF_SCALE = 0.7;

const double WHEEL_DIAMETER = 4.0; //inches, for fps for auton
const double TICKS_PER_ROT = 1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders

const double MAX_Y_RPM_LOW = 550.0;
const double MAX_Y_RPM_HIGH = 1250.0;
const double MAX_Y_RPM_HD = 0; //HDrive

double MAX_FPS = 0; //used in auton pathfinder

const double ACTUAL_MAX_Y_RPM_LOW = 625.0; //668 what
const double ACTUAL_MAX_Y_RPM_HIGH = 1300.0;
const double ACTUAL_MAX_Y_RPM_HD = 0;

double DYN_MAX_Y_RPM = 625.0; //for field-centric
const double MAX_X_RPM = 400.0; // for HDrive

const double MAX_YAW_RATE_LOW = 12.0; //max angular velocity divided by the max rpm multiplied by set max rpm
const double MAX_YAW_RATE_HIGH = 28.0;
const double MAX_YAW_RATE_HD = 0.0;

const double MAX_KICK_FPS = ((MAX_X_RPM * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
const int Kv_KICK = (1 / MAX_KICK_FPS);

const double UP_SHIFT_VEL = 375.0; // (24/14) *9370 //RPM
const double DOWN_SHIFT_VEL = 200.0; //will be less than up shift vel (14/56) *9370 //RPM

const double TICKS_PER_FOOT = 1315.0;
const double DRIVE_WAIT_TIME = 0.05; //seconds
