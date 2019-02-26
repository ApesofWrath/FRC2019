#ifndef SRC_SUCTION_H_
#define SRC_SUCTION_H_

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <frc/DoubleSolenoid.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class HatchPickup {
public:

  TalonSRX *suction1, *suction2;
  frc::DoubleSolenoid *solenoid;

  const int ON_STATE_H = 0;
  const int OFF_STATE_H = 1;

  const int OUT_STATE_H = 0;
  const int IN_STATE_H = 1;

  int suction_state = OFF_STATE_H;
  int solenoid_state = IN_STATE_H;

  const int SUCTION_CHANNEL = 0;

  const int SOLENOID_FORWARD_CHANNEL = 0;
  const int SOLENOID_REVERSE_CHANNEL = 1;


  HatchPickup();

  void On();
  void Off();
  void SuctionStateMachine();

  void In();
  void Out();
  void SolenoidStateMachine();

  bool HaveHatch();
  bool ReleasedHatch();
  bool SeeHatch();


};

#endif
