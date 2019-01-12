/*
 * Wrist.cpp
 *
 *  Created on: Jan 11, 2019
 *      Author: Kaya
 */

 #include "Wrist.h"

 #define PI 3.14159265

 const int INIT_STATE = 0;
 const int UP_STATE = 1;
 const int MID_STATE = 2;
 const int DOWN_STATE = 3;

 const int HALL_EFF_WRIST_ID = 0;

 const double TICKS_PER_ROT_W = 4096.0;
 const double MAX_VOLTAGE_W = 12.0; //CANNOT EXCEED abs(10)
 const double MIN_VOLTAGE_W = -12.0;

 const double free_speed_w = 18730.0; //rpm
 const double G_w = (831.0 / 1.0); //gear ratio
 const double MAX_THEORETICAL_VELOCITY_W = ((free_speed_w/ G_w)) * (2.0 * PI)
 		/ 60.0; //rad/s
 const double Kv_w = 1 / MAX_THEORETICAL_VELOCITY_W;

 //for motion motionProfiler
 const double MAX_VELOCITY_W = 1.0; //rad/w
 const double MAX_ACCELERATION_W = 2.5; //rad/w/w
 const double TIME_STEP_W = 0.01; //sec

 const double UP_LIMIT_W = 1.58;
 const double DOWN_LIMIT_W = 0.0;

 const double UP_VOLT_LIMIT_W = 0.0;
 const double DOWN_VOLT_LIMIT_W = 0.0;

 const double WRIST_OFFSET = 1.3;

 const double STALL_VEL_W = 0.05;
 const double STALL_VOLT_W = 4.0;

 const double DOWN_ANGLE = 0.1;
 const double MID_ANGLE = 0.6;
 const double UP_ANGLE = 1.3; //starting pos

 double current_pos = 0.0;
 double current_vel = 0.0;
 double goal_pos = 0.0;
 double goal_vel = 0.0;
 double wrist_offset = 0.0;

 int last_wrist_state = 0; //cannot equal the first state or profile will not set the first time

 double u_w = 0; //this is the input in volts to the motor
 const double v_bat_w = 12.0; //needs to be separate from the max and min voltages, which may change

 std::vector<std::vector<double> > K_w;
 std::vector<std::vector<double> > K_down_w = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
 		{ 10.32, 0.063 } };
 std::vector<std::vector<double> > K_up_w = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
 		{ 10.32, 0.063 } };

 std::vector<std::vector<double> > X_w = { { 0.0 }, //state matrix filled with the states of the system //not used
 		{ 0.0 } };

 std::vector<std::vector<double> > error_w = { { 0.0 }, { 0.0 } };

 Timer *intakeTimer = new Timer();

 PowerDistributionPanel *pdp_w;

 WristMotionProfiler *wrist_profiler;

 bool is_at_bottom = false;
 bool first_time_at_bottom = false;
 bool voltage_safety = false;

 int init_counter_w = 0;
 int counter_w = 0;
 int w = 0;
 int encoder_counter = 0;

 Wrist::Wrist(PowerDistributionPanel *pdp,
 		WristMotionProfiler *wrist_profiler_){

   talonWrist = new TalonSRX(0);

   talonWrist->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
 	 //talonWrist->EnableCurrentLimit(true);
 	 talonWrist->ConfigPeakCurrentLimit(20, 0); //for now
 	 talonWrist->ConfigContinuousCurrentLimit(20, 0); //for now

   wrist_profiler = wrist_profiler_;

   wrist_profiler->SetMaxAccWrist(MAX_ACCELERATION_W);
 	 wrist_profiler->SetMaxVelWrist(MAX_VELOCITY_W);
 	 hallEffectWrist = new DigitalInput(HALL_EFF_WRIST_ID);

   pdp_w = pdp;

 }

 void Wrist::InitializeWrist() {

 	//ZeroEnc();

 	//if (!IsAtBottomWrist()) { //don't see hall effect
 	if (!is_init_wrist) { //this has to be here for some reason
 		SetVoltageWrist(-1.5); //double wrist_volt = (2.0 / pdp_i->GetVoltage()) * 1.0; //down//SetVoltageWrist(-1.0);
 		//talonWrist->Set(ControlMode::PercentOutput, wrist_volt);
 	}

 	//double up_volt = (0.2 * -1.0) / pdp_e->GetVoltage(); //to not crash down
 	//talonElevator1->Set(ControlMode::PercentOutput, up_volt);
 	//talonElevator2->Set(ControlMode::PercentOutput, up_volt);
 }

 void Wrist::ManualWrist(Joystick *joyOpWrist) {

 	//SmartDashboard::PutNumber("WRIST CUR", talonWrist->GetOutputCurrent());
 	//double enc_wrist = GetAngularPosition();
 	//SmartDashboard::PutNumber("WRIST ENC",
 	//		talonWrist->GetSensorCollection().GetQuadraturePosition());

 	//SmartDashboard::PutNumber("WRIST POS", GetAngularPosition()); //left is negative, right is positive

 	double output = joyOpWrist->GetY() * 0.5 * 1.0;

 	output *= MAX_VOLTAGE_W;

 	//SmartDashboard::PutNumber("WRIST OUTPUT", output);

 	SetVoltageWrist(output);

 }

 void Wrist::Rotate(std::vector<std::vector<double> > ref_wrist) {

 	//top is position, bottom is velocity

 	current_pos = GetAngularPosition();
 	current_vel = GetAngularVelocity();
 	goal_pos = ref_wrist[0][0];
 	goal_vel = ref_wrist[1][0];

 	SmartDashboard::PutNumber("WRIST POS", current_pos);
 	SmartDashboard::PutNumber("WRIST VEL", current_vel);

 	SmartDashboard::PutNumber("WRIST REF POS", goal_pos);
 	SmartDashboard::PutNumber("WRIST REF VEL", goal_vel);

 	error_w[0][0] = goal_pos - current_pos;
 	error_w[1][0] = goal_vel - current_vel;

 	SmartDashboard::PutNumber("WRIST ERR POS", error_w[0][0]);
 	SmartDashboard::PutNumber("WRIST ERR VEL", error_w[1][0]);

 	if (wrist_profiler->final_goal_w < current_pos) { //must use final ref, to account for getting slightly ahead of the profiler
 		K_w = K_down_w;
 	} else {
 		K_w = K_up_w;
 	}

 //	if (IsAtBottomWrist() && std::abs(error_a[0][0]) > 0.4) { //shaking
 //		wrist_state = DOWN_STATE;
 //	}

 	wrist_offset = WRIST_OFFSET * (double) cos(current_pos); //counter gravity when needed because of slack on the wrist

 	u_w = (K_w[0][0] * error_w[0][0]) + (K_w[0][1] * error_w[1][0]) //u_sis in voltage, so * by v_bat_a
 			+ (Kv_w* goal_vel * v_bat_w) + wrist_offset;

 	SmartDashboard::PutNumber("WRIST CONT VOLT", u_w);

 	SetVoltageWrist(u_w);

 }

 void Wrist::SetVoltageWrist(double voltage_w) {

 	is_at_bottom = IsAtBottomWrist(); //hall effect returns 0 when at bottom. we reverse it here
 	SmartDashboard::PutNumber("WRIST HALL EFF", is_at_bottom); //actually means not at bottom //0 means up// 1 means dow

 	SmartDashboard::PutString("WRIST SAFETY", "none");

 	//soft limit top
 	if (GetAngularPosition() >= (UP_LIMIT_W) && voltage_w > UP_VOLT_LIMIT_W) { //at max height and still trying to move up
 		voltage_w = 0.0; //shouldn't crash
 		SmartDashboard::PutString("WRIST SAFETY", "top soft limit");
 	}

 	//soft limit bottom
 	if (is_at_bottom) {
 		ZeroEnc(); //will not run after 2nd time (first time is in teleop init)
 		if (GetAngularPosition() < (DOWN_LIMIT_W) && voltage_w < DOWN_VOLT_LIMIT_W) { //but hall effect goes off at 0.35
 			voltage_w = 0.0;
 			SmartDashboard::PutString("WRIST SAFETY", "bot hall eff");
 		}
 	}

 //	} else {
 //		counter_s= 0;
 //	}

 	//voltage limit
 	if (voltage_w > MAX_VOLTAGE_W) {
 		voltage_w = MAX_VOLTAGE_W;
 		SmartDashboard::PutString("WRIST SAFETY", "clipped");
 	} else if (voltage_w < MIN_VOLTAGE_W) {
 		voltage_w = MIN_VOLTAGE_W;
 		SmartDashboard::PutString("WRIST SAFETY", "clipped");
 	}

 	//stall
 	if (std::abs(GetAngularVelocity()) <= STALL_VEL_W && std::abs(voltage_w) > STALL_VOLT_W) { //outputting current, not moving, should be moving
 		encoder_counter++;
 	} else {
 		encoder_counter = 0;
 	}
 	if (encoder_counter > 10) {
 		voltage_safety = true;
 		voltage_w = 0.0;
 		SmartDashboard::PutString("WRIST SAFETY", "stall");
 	}

 	SmartDashboard::PutNumber("WRIST VOLTAGE", voltage_w);

 	//scale
 	voltage_w /= v_bat_w; //scale from -1 to 1 for the talon // max voltage is positive

 	//reverse
 	voltage_w *= -1.0; //set AT END

 	talonWrist->Set(ControlMode::PercentOutput, voltage_w);

 }

 double Wrist::GetAngularVelocity() {

 	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
 	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
 	double ang_vel =
 			(talonWrist->GetSensorCollection().GetQuadratureVelocity()
 					/ (TICKS_PER_ROT_W)) * (2.0 * PI) * (10.0) * -1.0;
 	//double ang_vel = 0.0;

 	return ang_vel;

 }

 double Wrist::GetAngularPosition() {

 	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
 	//2pi radians per rotations
 	double ang_pos =
 			(talonWrist->GetSensorCollection().GetQuadraturePosition()
 					/ (TICKS_PER_ROT_W)) * (2.0 * PI) * -1.0;
 	//double ang_pos = 0.0;

 	double offset_pos = .35; //amount that the wrist will stick up in radians

 	return ang_pos + offset_pos;

 }

 bool Wrist::IsAtBottomWrist() {
 	if (!hallEffectWrist->Get()) {
 		return true;
 	} else {
 		return false;
 	}
 }

 void Wrist::StopWrist() {

 	talonWrist->Set(ControlMode::PercentOutput, 0.0);

 }

 void Wrist::WristStateMachine(){

   SmartDashboard::PutNumber("WRIST ENC",
 			talonWrist->GetSensorCollection().GetQuadraturePosition())

   switch (wrist_state){

     case INIT_STATE:
     SmartDashboard::PutString("WRIST", "INIT");
     InitializeWrist();
     if (is_init_wrist) {
 			wrist_state = UP_STATE;
 		}
 		last_wrist_state = INIT_STATE;
     break;

     case UP_STATE:
     SmartDashboard::PutString("WRIST", "UP");
     if (last_wrist_state != UP_STATE) { //first time in state
       wrist_profiler->SetMaxAccWrist(MAX_ACCELERATION_W); //these must be reset in each state
       wrist_profiler->SetMaxVelWrist(MAX_VELOCITY_W);
       wrist_profiler->SetFinalGoalWrist(UP_ANGLE);
 			 wrist_profiler->SetInitPosWrist(GetAngularPosition());
 		 }
 		last_wrist_state = UP_STATE;
     break;

     case MID_STATE:
     SmartDashboard::PutString("WRIST", "MID");
     if (last_wrist_state != MID_STATE) {
       wrist_profiler->SetMaxAccWrist(MAX_ACCELERATION_W); //these must be reset in each state
       wrist_profiler->SetMaxVelWrist(MAX_VELOCITY_W);
 			wrist_profiler->SetFinalGoalIntake(MID_ANGLE);
 			wrist_profiler->SetInitPosIntake(GetAngularPosition());
 		}
 		last_wrist_state = MID_STATE;
     break;

     case DOWN_STATE:
     SmartDashboard::PutString("WRIST", "DOWN");
     if (last_wrist_state != DOWN_STATE) {
       wrist_profiler->SetMaxAccWrist(MAX_ACCELERATION_W); //these must be reset in each state
       wrist_profiler->SetMaxVelWrist(MAX_VELOCITY_W);
 			wrist_profiler->SetFinalGoalWrist(DOWN_ANGLE);
 			wrist_profiler->SetInitPosWrist(GetAngularPosition());
 		}
 		last_wrist_state = DOWN_STATE;
     break;

   }

   }

   bool Wrist::EncodersRunning() { //will stop the controller from run //or stalled //MOVED INTO SET VOLTAGE

   //	double current_pos = GetAngularPosition(); //radians
   //	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

   	return true;
   }

   void Wrist::ZeroEnc() { //called in Initialize() and in SetVoltage()

     if (zeroing_counter_w < 2) {
       talonWrist->GetSensorCollection().SetQuadraturePosition(0, 0);
       zeroing_counter_s++;
     } else {
       is_init_wrist = true;
     }
   }

   void Wrist::WristWrapper(Wrist *wrist) {

   	wristTimer->Start();

   	while (true) {
   		while (frc::RobotState::IsEnabled()) {
   			std::this_thread::sleep_for(
   					std::chrono::milliseconds(WRIST_SLEEP_TIME));

   			if (wristTimer->HasPeriodPassed(WRIST_WAIT_TIME)) {

   				std::vector<std::vector<double>> profile_wrist =
   						wrist_profiler->GetNextRefWrist();

   				if (wrist->wrist_state != STOP_WRIST_STATE
   						&& wrist->wrist_state != INIT_STATE) {
   					wrist->Rotate(profile_wrist);
   				}

   				wristTimer->Reset();

   			}
   		}
   	}

   }

   void Wrist::StartWristThread() {

   	Wrist *wrist_ = this;

   	WristThread = std::thread(&Wrist::WristWrapper, wrist_);
   	WristThread.detach();

   }

   void Wrist::EndWristThread() {

   	WristThread.~thread();

   }

 }
