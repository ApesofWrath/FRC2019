#ifndef SRC_SUCTION_H_
#define SRC_SUCTION_H_

#include <frc/WPILib.h>
#include <frc/DigitalOutput.h>

class Suction {
public:

  frc::DigitalOutput *suction;

  const int PULL_STATE_H = 0;
  const int PUSH_STATE_H = 1;
  int suction_state = PULL_STATE_H;

  const int SUCTION_CHANNEL = -1;

  Suction();
  void Pull();
  void Push();
  void SuctionStateMachine();

};

#endif
