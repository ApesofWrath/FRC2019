/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

	void Robot::RobotInit() {

		/* Hardware */
		talon = new TalonSRX(1);
		joy = new frc::Joystick(0);

		/* create some followers */
		VictorSPX *victor = new VictorSPX(2);
		victor->Follow(*talon);

		/* Factory default hardware to prevent unexpected behavior */
		talon->ConfigFactoryDefault();
		victor->ConfigFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		talon->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);//configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											// Constants.kPIDLoopIdx,
											// 30);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		talon->SetSensorPhase(true);
	//	talon.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 30);
		talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 30);

		/* Set the peak and nominal outputs */
		talon->ConfigNominalOutputForward(0, 30);
		talon->ConfigNominalOutputReverse(0, 30);
		talon->ConfigPeakOutputForward(1, 30);
		talon->ConfigPeakOutputReverse(-1, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		talon->SelectProfileSlot(0, 0);
		talon->Config_kF(0, .001, 30);
		talon->Config_kP(0, 0, 30);
		talon->Config_kI(0, 0, 30); //middle number is the gain
		talon->Config_kD(0, 0, 30);

		/* Set acceleration and vcruise velocity - see documentation */
		talon->ConfigMotionCruiseVelocity(15000, 30);
		talon->ConfigMotionAcceleration(6000, 30);

		/* Zero the sensor */
		talon->SetSelectedSensorPosition(0, 0, 30);
	}

	/**
	 * This function is called periodically during operator control
	 */
	void Robot::TeleopPeriodic() {
		/* Get gamepad axis - forward stick is positive */

		if (joy->GetRawButton(1)) {
			/* Motion Magic */

			/*4096 ticks/rev * 10 Rotations in either direction */
			double targetPos = 99 * 4096 * 10.0;
			talon->Set(ControlMode::MotionMagic, targetPos);

	}
}

int main() { return frc::StartRobot<Robot>(); }
