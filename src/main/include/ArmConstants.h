class ArmConstants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	static int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	static int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	static int kTimeoutMs = 30;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
  static Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

	double kP;
	double kI;
	double kD;
	double kF;
	int kIzone;
	double kPeakOutput;

}

void Gains (double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {

kP = _kP;
kI = _kI;
kD = _kD;
kF = _kF;
kIzone = _kIzone;
kPeakOutput = _kPeakOutput;

}
