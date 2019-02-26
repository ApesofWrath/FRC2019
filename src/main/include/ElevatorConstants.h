#ifndef SRC_ELEVATORCONSTANTS_H_
#define SRC_ELEVATORCONSTANTS_H_

//CTRE settings
const double Kf_e = 0.26;
const double Kp_e = 0.25;
const double Ki_e = 0.000032;
const double Kd_e = 0.0;

const double CRUISE_VEL_E = 0;
const double ENC_CRUISE_VEL_E = 10000; //CRUISE_VEL_E * ((1.0 / (3.14 * PULLEY_DIAMETER)) * 4096.0 / 2.0) / 10.0;

const double CRUISE_ACC_E = 0;
const double ENC_CRUISE_ACC_E = 3500; //CRUISE_ACC_E * ((1.0 / (3.14 * PULLEY_DIAMETER)) * 4096.0 / 2.0) / 100.0; //1/sec^2 to 1/100sec^2
//

const double EL_SAFETY_HEIGHT = 0.7; //lowest position for elev in order for arm to move backward
const double ARM_HEIGHT_SAFETY_HEIGHT = 1.0; //how far back the arm needs to be in order for the elevator to need to stay up

const double ff_percent_e = 0.4;
const double PULLEY_DIAMETER = 0.0381;

const double free_speed_e = 18730.0; //rad/s
const double TICKS_PER_ROT_E = 4096.0; //possibly not
const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_E = -10.0;

const double STALLS_TILL_STOP = 3;

#endif
