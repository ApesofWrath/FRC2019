#include "HatchPickup.h"

const int ON_STATE = 0;
const int OFF_STATE = 1;

const int OUT_STATE = 0;
const int IN_STATE = 1;

const int sample_window = 10;
double currents_intake[sample_window] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double avg1 = 0.0;
double avg2 = 0.0;
//int encoder_counter = 0;

int has_counter = 0; // make sure not drawing current for drive

nt::NetworkTableEntry hatchEntry;

HatchPickup::HatchPickup() {

  suction1 = new TalonSRX(12);
  suction2 = new TalonSRX(40);
  suction2->Follow(*suction1);

  solenoid = new frc::DoubleSolenoid(9, SOLENOID_FORWARD_CHANNEL, SOLENOID_REVERSE_CHANNEL);
  // release_solenoid_left = new frc::Solenoid(3, 0);
  //   release_solenoid_right = new frc::Solenoid(3, 1);
 release_solenoid = new frc::DoubleSolenoid(9, -0, -0);
}

void HatchPickup::PistonIn() {

//  alignment_solenoid->Set(frc::DoubleSolenoid::kReverse);

}

void HatchPickup::PistonOut() {

//  alignment_solenoid->Set(frc::DoubleSolenoid::kForward);

}

void HatchPickup::On() {

  suction1->Set(ControlMode::PercentOutput, -0.9); //.3
///  suction2->Set(ControlMode::PercentOutput, 0.3);

}

void HatchPickup::Off() {

  suction1->Set(ControlMode::PercentOutput, 0.0);
//  suction2->Set(ControlMode::PercentOutput, 0.0);

}

void HatchPickup::In() {
  solenoid->Set(frc::DoubleSolenoid::kReverse); //reversed

}

void HatchPickup::Out() {

  solenoid->Set(frc::DoubleSolenoid::kForward); //reversed

}

void HatchPickup::SuctionStateMachine() {

  switch (suction_state) {
      case ON_STATE:
          On();
          frc::SmartDashboard::PutString("SUCTION", "On");
          break;
      case OFF_STATE:
          Off();
          frc::SmartDashboard::PutString("SUCTION", "Off");
          break;
  }
}


void HatchPickup::SolenoidStateMachine() {

  switch (solenoid_state) {
    case OUT_STATE:
      Out();
      frc::SmartDashboard::PutString("SOLENOID", "Out");
      break;

    case IN_STATE:
      In();
      frc::SmartDashboard::PutString("SOLENOID", "In");
      break;
  }

}

bool HatchPickup::HaveHatch() {
  frc::SmartDashboard::PutNumber("have hatch counter", has_counter);
  if ((suction1->GetOutputCurrent() < 3.5 && suction1->GetOutputCurrent() > 0.5) && (suction2->GetOutputCurrent() < 3.5 && suction2->GetOutputCurrent() > 0.5)) {
    has_counter++;
    if (has_counter > 7) { //there are downspikes in current when driving
    //   frc::SmartDashboard::PutNumber("have", 1);
       has_counter = 0;
      return true;
     }

  } else {
     has_counter = 0;
    // frc::SmartDashboard::PutNumber("have", 0);
    return false;
  }

}

bool HatchPickup::ReleasedHatch() {

  if ((suction1->GetOutputCurrent() > 3.5) || (suction2->GetOutputCurrent() > 3.5)) {
    frc::SmartDashboard::PutNumber("released",1);
    return true;
  } else {
    frc::SmartDashboard::PutNumber("released",0);
  return false;
  }

}

bool HatchPickup::SeeHatch() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetBoolean("hatch", false);

}
