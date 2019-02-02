#ifndef SRC_SUCTION_H_
#define SRC_SUCTION_H_

#include <frc/WPILib.h>
#include <frc/DigitalOutput.h>
#include <frc/DoubleSolenoid.h>

class HatchPickup {
public:

  frc::DigitalOutput *suction;
  frc::DoubleSolenoid *solenoid;

  const int PULL_STATE_H = 0;
  const int PUSH_STATE_H = 1;

  const int DOWN_STATE_H = 0;
  const int UP_STATE_H = 1;

  int suction_state = PULL_STATE_H;
  int solenoid_state = DOWN_STATE_H;


  const int SUCTION_CHANNEL = -1;

  const int SOLENOID_FORWARD_CHANNEL = -1;
  const int SOLENOID_REVERSE_CHANNEL = -1;


  HatchPickup();

  void Pull();
  void Push();
  void SuctionStateMachine();

  void Up();
  void Down();
  void SolenoidStateMachine();


};

#endif
