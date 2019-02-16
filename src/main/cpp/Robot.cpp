/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

class Robot extends TimedRobot {
	/* Hardware */
	TalonSRX talon = new TalonSRX(1);
	Joystick joy = new Joystick(0);

	/* create some followers */
	BaseMotorController _follower1 = new TalonSRX(0);
	BaseMotorController _follower2 = new VictorSPX(0);
	BaseMotorController _follower3 = new VictorSPX(1);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	void robotInit() {
		/* setup some followers */
		_follower1.configFactoryDefault();
		_follower2.configFactoryDefault();
		_follower3.configFactoryDefault();
		_follower1.follow(_talon);
		_follower2.follow(_talon);
		_follower3.follow(_talon);

		/* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx,
											Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(true);
		_talon.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	void teleopPeriodic() {
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY();
		if (Math.abs(leftYstick) < 0.10) { leftYstick = 0;} /* deadband 10% */

		/* Get current Talon SRX motor output */
		double motorOutput = _talon.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		/**
		 * Peform Motion Magic when Button 1 is held,
		 * else run Percent Output, which can be used to confirm hardware setup.
		 */
		if (_joy.getRawButton(1)) {
			/* Motion Magic */

			/*4096 ticks/rev * 10 Rotations in either direction */
			double targetPos = leftYstick * 4096 * 10.0;
			_talon.set(ControlMode.MotionMagic, targetPos);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent Output */

			_talon.set(ControlMode.PercentOutput, leftYstick);
		}

		/* Instrumentation */
		//Instrum.Process(_talon, _sb);
	}
}
