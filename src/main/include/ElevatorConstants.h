#ifndef SRC_ELEVATORCONSTANTS_H_
#define SRC_ELEVATORCONSTANTS_H_

// Last year positions for testing on Cornelius
const double DOWN_POS_E = 0.01;
const double MID_POS_E = 0.4;//0.668;
const double UP_POS_E = 0.70;

// elevator posititons for the differnet states
const double ROCKET_TOP_CARGO_POS = UP_POS_E;
const double ROCKET_MID_CARGO_POS = MID_POS_E;
const double ROCKET_BOTTOM_CARGO_POS = DOWN_POS_E;
const double ROCKET_TOP_HATCH_POS = 0.0;
const double ROCKET_MID_HATCH_POS = 0.0;
const double BOTTOM_HATCH_POS = 0.0;
const double BAY_CARGO_POS = 0.0;

const double ff_percent_e = 0.4;
const double PULLEY_DIAMETER = 0.0381;

const double free_speed_e = 18730.0; //rad/s
const double TICKS_PER_ROT_E = 4096.0; //possibly not
const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_E = -10.0;

const double STALLS_TILL_STOP = 3;

#endif
