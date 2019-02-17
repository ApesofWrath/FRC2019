#ifndef SRC_ELEVATORCONSTANTS_H_
#define SRC_ELEVATORCONSTANTS_H_

const double EL_SAFETY_HEIGHT = 0.7; //lowest position for elev in order for arm to move backward
const double ARM_HEIGHT_SAFETY_HEIGHT = 1.0; //how far back the arm needs to be in order for the elevator to need to stay up

// Last year positions for testing on Cornelius
// const double DOWN_POS_E = 0.01;
// const double MID_POS_E = 0.4;//0.668;
// const double UP_POS_E = 0.70;

const double ff_percent_e = 0.4;
const double PULLEY_DIAMETER = 0.0381;

const double free_speed_e = 18730.0; //rad/s
const double TICKS_PER_ROT_E = 4096.0; //possibly not
const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_E = -10.0;

const double STALLS_TILL_STOP = 3;

#endif
