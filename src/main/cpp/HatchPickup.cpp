#include "HatchPickup.h"

const int ON_STATE = 0;
const int OFF_STATE = 1;

const int OUT_STATE = 0;
const int IN_STATE = 1;

const int sample_window = 10;
int currents_intake[sample_window] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double avg1 = 0.0;
double avg2 = 0.0;
//int encoder_counter = 0;

HatchPickup::HatchPickup() {

  suction1 = new TalonSRX(12);
  suction2 = new TalonSRX(40);
  suction2->Follow(*suction1);

  solenoid = new frc::DoubleSolenoid(9, SOLENOID_FORWARD_CHANNEL, SOLENOID_REVERSE_CHANNEL);
}

void HatchPickup::On() {

  suction1->Set(ControlMode::PercentOutput, -1.0); //.3
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

  for (int i = 0; i < (sample_window - 2); i++) { //to index 18
		currents_intake[i] = currents_intake[i + 1];
	}

	currents_intake[sample_window - 1] =
			suction1->GetOutputCurrent();

  for (int i = 0; i < 4; i++) {
    avg1++;
  }
  avg1 /= sample_window / 2;

  for (int i = 5; i < (sample_window - 1); i++) {
    avg2++;
  }
  avg2 /= sample_window / 2;

  if ((avg1 - avg2) > 0.5) {
    return true;
  }
  return false;

}

bool HatchPickup::ReleasedHatch() {
  return false;
}
