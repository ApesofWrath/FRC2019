#ifndef SRC_SUCTION_H_
#define SRC_SUCTION_H_

#include <frc/WPILib.h>
#include <frc/DigitalOutput.h>
#include <frc/DoubleSolenoid.h>

class HatchPickup {
public:

  frc::DigitalOutput *suction;
  frc::DoubleSolenoid *solenoid;

  const int ON_STATE_H = 0;
  const int OFF_STATE_H = 1;

  const int OUT_STATE_H = 0;
  const int IN_STATE_H = 1;

  int suction_state = 0;
  int solenoid_state = 0;


  const int SUCTION_CHANNEL = -1;

  const int SOLENOID_FORWARD_CHANNEL = -1;
  const int SOLENOID_REVERSE_CHANNEL = -1;


  HatchPickup();

  void On();
  void Off();
  void SuctionStateMachine();

  void In();
  void Out();
  void SolenoidStateMachine();


};

#endif
