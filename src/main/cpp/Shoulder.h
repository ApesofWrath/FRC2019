/*
 * Shoulder.h
 *
 *  Created on: Jan 7, 2019
 *      Author: Kaya
 */

 #ifndef SRC_SHOULDER_H_
 #define SRC_SHOULDER_H_

 #include <frc/WPILib.h>
 #include "ctre/Phoenix.h"
 #include <Timer.h>
 #include <thread>
 #include <chrono>
 #include <vector>
 #include <cmath>
 #include <list>
 #include <DigitalInput.h>
 #include <ShoulderMotionProfiler.h>

class Shoulder {
public:

  TalonSRX *talonShoulder;

  DigitalInput *hallEffectShoulder; //for bottom

  std::thread ShoulderThread;

	int zeroing_counter_s = 0;

	bool is_init_shoulder = false; //is shoulder initialized

  const int INIT_STATE_H = 0;
	const int UP_STATE_H = 1; //shoulder state machine
  const int HIGH_STATE_H = 2;
	const int MID_STATE_H = 3;
	const int DOWN_STATE_H = 4;
  const int STOP_SHOULDER_STATE_H = 5;
  int shoulder_state = INIT_STATE_H;

  Shoulder(PowerDistributionPanel *pdp,
      ShoulderMotionProfiler *shoulder_profiler)

  void InitializeShoulder();

  void Rotate(std::vector<std::vector<double> > ref_shoulder);
	double GetAngularVelocity();
	double GetAngularPosition();

	bool IsAtBottomShoulder();
  bool EncodersRunning();

	void SetVoltageShoulder(double voltage_s);

	void ZeroEnc();
	void ManualShoulder(Joystick *joyOpShoulder);

  void StopShoulder();
  void ShoulderStateMachine();

  void StartShoulderThread();
	void EndShoulderThread();
	static void ShoulderWrapper(Shoulder *shoulder);

};

#endif /* SRC_SHOULDER_H_ */
