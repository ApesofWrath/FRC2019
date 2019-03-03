#ifndef SRC_ARMCONSTANTS_H_
#define SRC_ARMCONSTANTS_H_

//CTRE settings

const double Kf = 0.4;//0.3 //works like this
const double Kp = 0.0;//0.75; //0.0
const double Ki = 0.0;//0.00085;
const double Kd = 0.0;

const double arb_ff_percent = 0.0;//1.0; //0.1

const double CRUISE_VEL = 0.0;//0.041; //rad/sec
const double ENC_CRUISE_VEL = 2000.0; // conversion incorrect - CRUISE_VEL * (1.0 / (2.0 * 3.14159)) * 4096.0 * 7.5 * (1.0 / 10.0); //ticks/100ms

const double CRUISE_ACC = 0.0; //acceleration, which is in velocity units per second (where velocity units = change in sensor per 100ms)
const double ENC_CRUISE_ACC = 100.0; //CRUISE_ACC * (1.0 / (2.0 * 3.14159)) * 4096.0 * 7.5 * (100.0); //ticks/100ms/sec = ticks/100sec

const int TIMEOUT_MS = 10;
const int FRAME_PERIOD = 10;

//

const double MAX_VEL_ENC = 0.0;
const double TICKS_PER_ROT_A = 4096.0;
const double MAX_VOLTAGE_A = 12.0;
const double MIN_VOLTAGE_A = -12.0;

const double free_speed_a = 18730.0; //rpm
const double G_a = 7.5; //gear ratio
const double ENC_GEAR_RATIO = 7.5; //ENC_GEAR_RATIO encoder rot for each arm rot
const double MAX_THEORETICAL_VELOCITY_A = ((free_speed_a/ G_a)) * (2.0 * 3.14159265)
		/ 60.0; //rad/s
const double Kv_a = 1 / MAX_THEORETICAL_VELOCITY_A;

//for motionProfiler
const double MAX_VELOCITY_A = 1.0; //rad/s
const double MAX_ACCELERATION_A = 2.5; //rad/s/s
const double TIME_STEP_A = 0.01; //sec

const double UP_LIMIT_A = 1.58;
const double BACK_LIMIT_A = 2.0;
const double DOWN_LIMIT_A = 0.0;

const double UP_VOLT_LIMIT_A = 0.0;
const double DOWN_VOLT_LIMIT_A = 0.0;

const double ARM_OFFSET = 1.3;

const double STALL_VEL_A = 0.05;
const double STALL_VOLT_A = 4.0;

#endif
