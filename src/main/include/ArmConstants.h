#ifndef SRC_ARMCONSTANTS_H_
#define SRC_ARMCONSTANTS_H_

#define PI 3.14159265

const double TICKS_PER_ROT_A = 4096.0;
const double MAX_VOLTAGE_A = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_A = -12.0;

const double free_speed_a = 18730.0; //rpm
const double G_a = (831.0 / 1.0); //gear ratio
const double ENC_GEAR_RATIO = (7.5 / 1.0); //7.5 encoder rot for each arm rot
const double MAX_THEORETICAL_VELOCITY_A = ((free_speed_a/ G_a)) * (2.0 * PI)
		/ 60.0; //rad/s
const double Kv_a = 1 / MAX_THEORETICAL_VELOCITY_A;

//for motion motionProfiler
const double MAX_VELOCITY_A = 1.0; //rad/s
const double MAX_ACCELERATION_A = 2.5; //rad/a/a
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
