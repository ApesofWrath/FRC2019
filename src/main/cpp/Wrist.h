/*
 * Wrist.h
 *
 *  Created on: Jan 11, 2019
 *      Author: Kaya
 */

 #ifndef SRC_WRIST_H_
 #define SRC_WRIST_H_

 #include <frc/WPILib.h>
 #include "ctre/Phoenix.h"
 #include <Timer.h>
 #include <thread>
 #include <chrono>
 #include <vector>
 #include <cmath>
 #include <list>
 #include <DigitalInput.h>
 #include <WristMotionProfiler.h>

 class Wrist {
 public:

  TalonSRX *talonWrist;

  DigitalInput *hallEffectWrist; //for bottom

  std::thread WristThread;

  int zeroing_counter_w = 0;

  bool is_init_wrist = false; //is wrist initialized

  const int INIT_STATE_H = 0;
  const int UP_STATE_H = 1; //wrist state machine;
  const int MID_STATE_H = 2;
  const int DOWN_STATE_H = 3;
  int wrist_state = INIT_STATE_H;

  Wrist(PowerDistributionPanel *pdp,
      WristMotionProfiler *wrist_profiler)

  void InitializeWrist();

  void Rotate(std::vector<std::vector<double> > ref_wrist);
  double GetAngularVelocity();
  double GetAngularPosition();

  bool IsAtBottomWrist();
  bool EncodersRunning();

  void SetVoltageWrist(double voltage_w);

  void ZeroEnc();
  void ManualWrist(Joystick *joyOpWrist);

  void StopWrist();
  void WristStateMachine();

  void StartWristThread();
  void EndWristThread();
  static void WristWrapper(Wrist *wrist);

 };

 #endif /* SRC_WRIST_H_ */
