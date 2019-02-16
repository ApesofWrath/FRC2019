/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

double targetPos;
	void Robot::RobotInit() {

		/* Hardware */
		talon1 = new TalonSRX(33);
		talon2 = new TalonSRX(0);
		joy = new frc::Joystick(0);

		/* create some followers */
	//	talon2 *talon2 = new talon2SPX(2);
		talon2->Follow(*talon1);

		/* Factory default hardware to prevent unexpected behavior */
		talon1->ConfigFactoryDefault();
		talon2->ConfigFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		talon1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);//configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											// Constants.kPIDLoopIdx,
											// 30);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		//talon->SetSensorPhase(true);
		talon1->SetInverted(true);
		talon2->SetInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		talon1->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
		talon1->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

		/* Set the peak and nominal outputs */
		talon1->ConfigNominalOutputForward(0, 10);
		talon1->ConfigNominalOutputReverse(0, 10);
		talon1->ConfigPeakOutputForward(1, 10);
		talon1->ConfigPeakOutputReverse(-1, 10);

		/* Set Motion Magic gains in slot0 - see documentation */
		talon1->SelectProfileSlot(0, 0);
		talon1->Config_kF(0, .31, 10);
		talon1->Config_kP(0, 0.1, 10);
		talon1->Config_kI(0, 0, 10); //middle number is the gain
		talon1->Config_kD(0, 0, 10);

		/* Set acceleration and vcruise velocity - see documentation */
		talon1->ConfigMotionCruiseVelocity(1000, 10);
		talon1->ConfigMotionAcceleration(1000, 10);

		/* Zero the sensor */
		talon1->SetSelectedSensorPosition(0, 0, 10);
	}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
	/**
	 * This function is called periodically during operator control
	 */
	void Robot::TeleopPeriodic() {
		/* Get gamepad axis - forward stick is positive */

		if (joy->GetRawButton(1)) {
			/* Motion Magic */

			/*4096 ticks/rev * 10 Rotations in either direction */
			targetPos = 4.5 * 4096;
			talon1->Set(ControlMode::MotionMagic, targetPos);

	} else {
		talon1->Set(ControlMode::PercentOutput, joy->GetY());
	}

	frc::SmartDashboard::PutNumber("enc vel", talon1->GetSelectedSensorVelocity(0));
	frc::SmartDashboard::PutNumber("enc pos", talon1->GetSelectedSensorPosition(0));
		frc::SmartDashboard::PutNumber("enc targ", targetPos);
		frc::SmartDashboard::PutNumber("enc error",targetPos - talon1->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("cur", talon1->GetOutputCurrent());
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
