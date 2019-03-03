#include "../../include/Drive/DriveBase.h"

using namespace std::chrono;

//teleop drive sm
const int REGULAR = 0;
const int VISION_DRIVE = 1;
const int ROTATION_CONTROLLER = 2;
int teleop_drive_state = REGULAR;

//vision drive sm
const int CREATE_VIS_PROF = 0;
const int FOLLOW_VIS_PROF = 1;
const int RESET_VIS = 2;
int vision_drive_state = CREATE_VIS_PROF;

//auton drive sm
const int CREATE_AUTON_PROF = 0;
const int FOLLOW_AUTON_PROF = 1;
const int RESET_AUTON = 2;
const int TELEOP_DRIVE = 3;
int auton_drive_state = CREATE_AUTON_PROF;

std::vector<std::vector<double>> vision_profile; //runautondrive looks at auton_row size
std::vector<std::vector<double> > auton_profile(1500, std::vector<double>(5)); //rows stacked on rows, all points // can't be in .h for some reason

//WestCoast, 2-speed transmission option
DriveBase::DriveBase(int l1, int l2, int l3, int l4,
		int r1, int r2, int r3, int r4, int pcm, int f_channel, int r_channel, bool two_speed) {

	k_p_yaw_au = K_P_YAW_AU; //these get sent from AutonDrive to Controller, not used in AutonDrive
	k_d_yaw_au = K_D_YAW_AU;

	max_vel_vis = MAX_VEL_VIS; //m/s for pathfinder
	max_acc_vis = MAX_ACC_VIS;
	max_jerk_vis = MAX_JERK_VIS;

	k_p_yaw_heading_pos = K_P_YAW_HEADING_POS;
	k_d_vision_pos = K_D_VISION_POS;

	k_p_yaw_dis = K_P_YAW_DIS;
	k_i_yaw_dis = K_I_YAW_DIS;
	k_d_yaw_dis = K_D_YAW_DIS;

	ff_scale = FF_SCALE;

	DYN_MAX_Y_RPM = max_y_rpm;

	if (two_speed) { //StartLow()

		// max_y_rpm = MAX_Y_RPM_LOW;
		// max_yaw_rate = MAX_YAW_RATE_LOW;
		// actual_max_y_rpm = ACTUAL_MAX_Y_RPM_LOW;
		// max_yaw_rate = (max_yaw_rate / actual_max_y_rpm) * max_y_rpm;
		//
		// k_p_right_vel = K_P_RIGHT_VEL_LOW;
		// k_p_left_vel = K_P_LEFT_VEL_LOW;
		// k_p_yaw_vel = K_P_YAW_VEL_LOW;
		// k_d_yaw_vel = K_D_YAW_VEL_LOW;
		// k_d_right_vel = K_D_RIGHT_VEL_LOW;
		// k_d_left_vel = K_D_LEFT_VEL_LOW;
		//
		// k_f_left_vel = 1.0 / actual_max_y_rpm;
		// k_f_right_vel = 1.0 / actual_max_y_rpm;
		//
		// is_low_gear = true;

	} else { //regular constants

		max_y_rpm = MAX_Y_RPM;
		max_yaw_rate = MAX_YAW_RATE;
		actual_max_y_rpm = ACTUAL_MAX_Y_RPM;
		Kv = 1 / ACTUAL_MAX_Y_RPM;
		//max_yaw_rate = (max_yaw_rate / actual_max_y_rpm) * max_y_rpm;

		k_p_right_vel = K_P_RIGHT_VEL;
		k_p_left_vel = K_P_LEFT_VEL;
		k_p_yaw_vel = K_P_YAW_VEL;
		k_d_yaw_vel = K_D_YAW_VEL;
		k_d_right_vel = K_D_RIGHT_VEL;
		k_d_left_vel = K_D_LEFT_VEL;

		k_f_left_vel = 1.0 / actual_max_y_rpm;
		k_f_right_vel = 1.0 / actual_max_y_rpm;

	}

  actual_max_y_rpm_auton = ACTUAL_MAX_Y_RPM_AUTON;
  actual_max_y_rpm_l_f = ACTUAL_MAX_Y_RPM_L_F;
  actual_max_y_rpm_l_b = ACTUAL_MAX_Y_RPM_L_B;
  actual_max_y_rpm_r_f = ACTUAL_MAX_Y_RPM_R_F;
  actual_max_y_rpm_r_b = ACTUAL_MAX_Y_RPM_R_B;

	LF = l1;
	L2 = l2;
	L3 = l3;
	LR = l4;
	RF = r1;
	R2 = r2;
	R3 = r3;
	RR = r4;

	canTalonLeft1 = new TalonSRX(LF);
	canTalonLeft1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	canTalonLeft2 = new VictorSPX(L2);
  canTalonLeft2->Follow(*canTalonLeft1);
	canTalonLeft3 = new VictorSPX(L3);
  canTalonLeft3->Follow(*canTalonLeft1);

	canTalonRight1 = new TalonSRX(RF);
	canTalonRight1->SetInverted(true);
	canTalonRight1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	canTalonRight2 = new VictorSPX(R2);
  canTalonRight2->Follow(*canTalonRight1);
	canTalonRight3 = new VictorSPX(R3);
  canTalonRight3->Follow(*canTalonRight1);

  //talon settings

	canTalonLeft1->ConfigPeakCurrentLimit(30, 0);
	canTalonRight1->ConfigPeakCurrentLimit(30, 0);

	 canTalonLeft1->ConfigContinuousCurrentLimit(30, 0);
	 canTalonRight1->ConfigContinuousCurrentLimit(30, 0);

	 canTalonLeft1->ConfigPeakCurrentDuration(10, 0);
	 canTalonRight1->ConfigPeakCurrentDuration(10, 0);

	canTalonLeft1->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft2->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft3->ConfigOpenloopRamp(0.15, 0);
	canTalonRight1->ConfigOpenloopRamp(0.15, 0);
	canTalonRight2->ConfigOpenloopRamp(0.15, 0);
	canTalonRight3->ConfigOpenloopRamp(0.15, 0);

	canTalonLeft1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft1->ConfigVelocityMeasurementWindow(5, 0);

	canTalonRight1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight1->ConfigVelocityMeasurementWindow(5, 0);

	canTalonLeft1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
	canTalonLeft1->SetStatusFramePeriod(
			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

  canTalonRight1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
  canTalonRight1->SetStatusFramePeriod(
    			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

  canTalonLeft1->ConfigVoltageCompSaturation(12.0);
  canTalonLeft1->EnableVoltageCompensation(true);
  canTalonRight1->ConfigVoltageCompSaturation(12.0);
  canTalonRight1->EnableVoltageCompensation(true);

  canTalonLeft2->ConfigVoltageCompSaturation(12.0);
  canTalonLeft2->EnableVoltageCompensation(true);
  canTalonRight2->ConfigVoltageCompSaturation(12.0);
  canTalonRight2->EnableVoltageCompensation(true);

  canTalonLeft3->ConfigVoltageCompSaturation(12.0);
  canTalonLeft3->EnableVoltageCompensation(true);
  canTalonRight3->ConfigVoltageCompSaturation(12.0);
  canTalonRight3->EnableVoltageCompensation(true);

  canTalonLeft1->EnableCurrentLimit(true); //still not true from tuner
  canTalonRight1->EnableCurrentLimit(true);

	ahrs = new AHRS(SerialPort::kUSB);
	visionDrive = new Vision();

  //shifter
	if (pcm != -1) solenoid = new DoubleSolenoid(PCM, F_CHANNEL, R_CHANNEL);

}

void DriveBase::SetAutonGains(bool same_side_scale) {

	if (same_side_scale) {
		// K_P_YAW_DIS = 1.68;
		// K_I_YAW_DIS = 0.001;
		//FF_SCALE = 0.7;
		//zero wait counter = 50
	} else { //if need another one

	}

}

//PD on left and right
//P on yaw
void DriveBase::TeleopWCDrive(Joystick *JoyThrottle, //finds targets for the Controller()
		Joystick *JoyWheel) {

	double target_l, target_r, target_yaw_rate;

	double throttle = JoyThrottle->GetY();

	double reverse_y = 1.0;

	if (throttle > 0.0) {
		reverse_y = -1.0;
	} else {
		reverse_y = 1.0;
	}

	double forward = (throttle) * (throttle); //squared and will always be positive, so we need reverse_y

	target_l = reverse_y * forward * max_y_rpm; //scale to velocity

	target_r = target_l;

	double reverse_x = 1.0;

	double wheel = JoyWheel->GetX(); //not take time to get wheel/throttle values multiple times

	if (wheel < 0.0) {
		reverse_x = -1.0;
	} else {
		reverse_x = 1.0;
	}

	double joy_wheel_val = reverse_x * wheel * wheel;

//	if (!is_low_gear) { //squrare wheel in high gear
//		joy_wheel_val *= reverse_x * JoyWheel->GetX();
//	}

	if (std::abs(joy_wheel_val) < .02) {
		joy_wheel_val = 0.0;
	}

	target_yaw_rate = -1.0 * (joy_wheel_val) * max_yaw_rate; //Left will be positive

	if (target_l > max_y_rpm) {
		target_l = max_y_rpm;
	} else if (target_l < -max_y_rpm) {
		target_l = -max_y_rpm;
	}

	if (target_r > max_y_rpm) {
		target_r = max_y_rpm;
	} else if (target_r < -max_y_rpm) {
		target_r = -max_y_rpm;
	}

	Controller(0.0, target_r, target_l, target_yaw_rate, k_p_right_vel,
			k_p_left_vel, 0.0, k_p_yaw_vel, 0.0, k_d_left_vel, k_d_right_vel, 0.0,
			0.0, 0.0, 0.0);

}

void DriveBase::RotationController(Joystick *JoyWheel) {

	double target_heading = init_heading
			+ (-1.0 * JoyWheel->GetX() * (90.0 * PI / 180.0)); //scaling, conversion to radians,left should be positive

	double current_heading = -1.0 * ahrs->GetYaw() * ( PI / 180.0); //degrees to radians, left should be positive

	double error_heading_h = target_heading - current_heading;

	double total_heading_h = k_p_yaw_heading_pos * error_heading_h;

	if (total_heading > max_yaw_rate) {
		total_heading = max_yaw_rate;
	} else if (total_heading < -max_yaw_rate) {
		total_heading = -max_yaw_rate;
	}

	Controller(0.0, 0.0, 0.0, total_heading_h, k_p_right_vel, k_p_left_vel,
			0.0, k_p_yaw_h_vel, 0.0, k_d_right_vel, k_d_left_vel,
		0.0, 0.0, 0.0, 0.0);

}

//only to be used in teleop; auton will already have this essentially
void DriveBase::GenerateVisionProfile(double dist_to_target, double yaw_to_target) {

	ZeroAll(true);

	int length;
	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	p1 = {0.0, 0.0, 0.0};
	p2 = {dist_to_target, 0.0, d2r(yaw_to_target)}; //y, x, yaw

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, TIME_STEP, max_vel_vis, max_acc_vis, max_jerk_vis, &candidate);

	length = candidate.length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	 vision_profile.resize(length, std::vector<double>(5));

	for (int i = 0; i < length; i++) {

		Segment sl = leftTrajectory[i];
		Segment sr = rightTrajectory[i];

		vision_profile.at(i).at(0) = ((double) sl.heading); //pathfinder doesn't give yaw vel targ
		vision_profile.at(i).at(1) = ((double) sl.position);
		vision_profile.at(i).at(2) = ((double) sr.position);
		vision_profile.at(i).at(3) = ((double) sl.velocity);
		vision_profile.at(i).at(4) = ((double) sr.velocity);

	}

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

	SetAutonRefs(vision_profile);

}

//position controlller
//auton targets, actually just pd
void DriveBase::AutonDrive() {

	double pf_yaw_dis = auton_row.at(0); //reversed in Generate - rad
	double pf_l_dis = auton_row.at(1); //feet
	double pf_r_dis = auton_row.at(2); //feet
	double pf_l_vel = auton_row.at(3); //fps
	double pf_r_vel = auton_row.at(4); //fps

	if (pf_yaw_dis > PI) { //get negative half and positive half on circle - left half is positive
		pf_yaw_dis -= (2 * PI);
	}

	//fps //not needed besides check for jitter
	// double r_current = -((double) canTalonRight1->GetSelectedSensorVelocity(0)
	// 		/ (double) TICKS_PER_FOOT) * MINUTE_CONVERSION / 60;
	// double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
	// 		/ (double) TICKS_PER_FOOT) * MINUTE_CONVERSION / 60;

	//frc::SmartDashboard::PutNumber("Actual left", l_current);

	//feet
	double r_dis = GetRightPosition();//-((double) canTalonRight1->GetSelectedSensorPosition(0) //empirically determined ticks per foot
//	/ TICKS_PER_FOOT);
	double l_dis = GetLeftPosition();//((double) canTalonLeft1->GetSelectedSensorPosition(0)
	//		/ TICKS_PER_FOOT);

 //frc::SmartDashboard::PutNumber("actualLeftDis", l_dis);
// frc::SmartDashboard::PutNumber("actualRightDis", r_dis);
// frc::SmartDashboard::PutNumber("actualLeftVel", l_current);
// frc::SmartDashboard::PutNumber("actualRightVel", r_current);

	double y_dis = -1.0 * ahrs->GetYaw() * (double) (PI / 180); //current theta (yaw) value

//	frc::SmartDashboard::PutNumber("Target Heading", refYaw);
//	frc::SmartDashboard::PutNumber("Actual Heading", y_dis);

	l_error_dis_au = pf_l_dis - l_dis; //feet
	r_error_dis_au = pf_r_dis - r_dis; //feet

	y_error_dis_au = pf_yaw_dis - y_dis;  //rad

	//frc::SmartDashboard::PutNumber("Heading error", y_error_dis_au);

	// if (std::abs(tarVelLeft - tarVelRight) < .05 && (std::abs(r_current) < 10)
	// 		&& (std::abs(l_current) < 10)) { //initial jitter
	//
	// }

	i_right += (r_error_dis_au);
	d_right = (r_error_dis_au - r_last_error);

	i_left += (l_error_dis_au);
	d_left = (l_error_dis_au - l_last_error);

	d_kick = (k_error_dis_au - kick_last_error);

	i_yaw += y_error_dis_au;

	P_RIGHT_DIS = K_P_RIGHT_DIS * r_error_dis_au;
	I_RIGHT_DIS = K_I_RIGHT_DIS * i_right;
	D_RIGHT_DIS = K_D_RIGHT_DIS * d_right;

	P_LEFT_DIS = K_P_LEFT_DIS * l_error_dis_au;
	I_LEFT_DIS = K_I_LEFT_DIS * i_left;
	D_LEFT_DIS = K_D_LEFT_DIS * d_left;

	P_YAW_DIS = k_p_yaw_dis * y_error_dis_au; //position
	I_YAW_DIS = k_i_yaw_dis * i_yaw;
	D_YAW_DIS = k_d_yaw_dis * (y_error_dis_au - yaw_last_error);

	// frc::SmartDashboard::PutNumber("P", P_YAW_DIS);
	// frc::SmartDashboard::PutNumber("I", I_YAW_DIS);
	// frc::SmartDashboard::PutNumber("D", D_YAW_DIS);

	double total_right = P_RIGHT_DIS + I_RIGHT_DIS + D_RIGHT_DIS;
	double total_left = P_LEFT_DIS + I_LEFT_DIS + D_LEFT_DIS;

	double total_yaw = P_YAW_DIS + I_YAW_DIS + D_YAW_DIS;

	//frc::SmartDashboard::PutNumber("TOTAL", total_yaw);

	//scaling - vel output depending on dis PID
	double target_fps_yaw_change = total_yaw * MAX_FPS;
	double target_fps_right = total_right * MAX_FPS;
	double target_fps_left = total_left * MAX_FPS;

	target_fps_right += (pf_r_vel + target_fps_yaw_change);
	target_fps_left += (pf_l_vel - target_fps_yaw_change);

	//std::cout << "FB: " << target_rpm_right << std::endl;

	if (target_fps_left > MAX_FPS) {
		target_fps_left = MAX_FPS;
	} else if (target_fps_left < -MAX_FPS) {
		target_fps_left = -MAX_FPS;
	}
//target rpm right is in fps
	if (target_fps_right > MAX_FPS) {
		target_fps_right = MAX_FPS;
	} else if (target_fps_right < -MAX_FPS) {
		target_fps_right = -MAX_FPS;
	}

	Controller(0.0, 0.0, 0.0, 0.0, k_p_right_vel_au, k_p_left_vel_au,
			0.0, k_p_yaw_au, k_d_yaw_au, k_d_left_vel_au, k_d_right_vel_au, 0.0, //sends all 0.0 gains
			target_fps_left, target_fps_right, 0.0);

	l_last_error = l_error_dis_au;
	r_last_error = r_error_dis_au;
	kick_last_error = k_error_dis_au;
	yaw_last_error = y_error_dis_au;

}

void DriveBase::Controller(double ref_kick,
		double ref_right, //first parameter refs are for teleop
		double ref_left, double ref_yaw, double k_p_right, double k_p_left,
		double k_p_kick, double k_p_yaw, double k_d_yaw, double k_d_right,
		double k_d_left, double k_d_kick, double target_vel_left, //50
		double target_vel_right, double target_vel_kick) { //last parameter targets are for auton

	double yaw_rate_current = -1.0 * (double) ahrs->GetRate(); //might be rad/ss
		//	* (double) ((PI) / 180.0); //left should be positive

	 // frc::SmartDashboard::PutNumber("yaw vel", yaw_rate_current);
	 // frc::SmartDashboard::PutNumber("yaw pos", ahrs->GetYaw());
	/// frc::SmartDashboard::PutNumber("max_y_rpm", max_y_rpm);
	// frc::SmartDashboard::PutNumber("max_yaw_rate", max_yaw_rate);

	double target_yaw_rate = ref_yaw;

	ref_left = ref_left - (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //left should be positive
	ref_right = ref_right + (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //ff

	double yaw_error = target_yaw_rate - yaw_rate_current;

//	frc::SmartDashboard::PutNumber("yaw vel error", yaw_error);

//	if(yaw_rate_current == 0.0) {
//		k_p_yaw = 0.0;
//		k_d_yaw = 0.0;
//	}

	if (std::abs(yaw_error) < .3) { //TODO: maybe get rid of this
		yaw_error = 0.0;
	}

	d_yaw_dis = yaw_error - yaw_last_error;

	double yaw_output = ((0.0 * yaw_error) + (k_d_yaw * d_yaw_dis)); //pd for auton, p for teleop //fb //hardly any
//frc::SmartDashboard::PutNumber("yaw p", yaw_output);
	ref_right += yaw_output; //left should be positive
	ref_left -= yaw_output;

	if (std::abs(ref_kick) < 25) {
		ref_kick = 0;
	}

	if (ref_left > max_y_rpm) {
		ref_left = max_y_rpm;
	} else if (ref_left < -max_y_rpm) {
		ref_left = -max_y_rpm;
	}

	if (ref_right > max_y_rpm) {
		ref_right = max_y_rpm;
	} else if (ref_right < -max_y_rpm) {
		ref_right = -max_y_rpm;
	}

  if (frc::RobotState::IsAutonomous()) { //only want the feedforward based off the motion profile during autonomous. The root generated ones (in the if() statement) //should already be 0 during auton because we send 0 as refs
    feed_forward_r = 0;	// will be close to 0  (low error between profile points) for the most part but will get quite aggressive when an error builds,
    feed_forward_l = 0;			//the PD controller should handle it itself
    feed_forward_k = 0;

  } else {

    if (ref_right < 0.0) {
      k_f_right_vel = 1.0 / actual_max_y_rpm_r_b;
    } else {
      k_f_right_vel = 1.0 / actual_max_y_rpm_r_f;
    }

    if (ref_left < 0.0) {
      k_f_left_vel = 1.0 / actual_max_y_rpm_l_b;
    } else {
      k_f_left_vel = 1.0 / actual_max_y_rpm_l_f;
    }

    feed_forward_r = k_f_right_vel * ref_right; //teleop only, controlled
  	feed_forward_l = k_f_left_vel * ref_left;
  	feed_forward_k = 0.0 * ref_kick;//kf kick vel
  }


	// frc::SmartDashboard::PutNumber("kf r", k_f_right_vel);
	// frc::SmartDashboard::PutNumber("kf l", k_f_left_vel);
	//
	// frc::SmartDashboard::PutNumber("ff r", feed_forward_r * MAX_Y_RPM); //max rpm
	// frc::SmartDashboard::PutNumber("ff l", feed_forward_l * MAX_Y_RPM);

	//conversion to RPM from native unit
	double l_current = GetLeftVel(); //-((double) canTalonLeft1->GetSelectedSensorVelocity(0)
	//		/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
	double r_current = GetRightVel(); //((double) canTalonRight1->GetSelectedSensorVelocity(0)
	//		/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
//	double kick_current = ((double) canTalonKicker->GetSelectedSensorVelocity(0) //will timeout, taking too much time
//			 (double) TICKS_PER_ROT) * MINUTE_CONVERSION; //going right is positive

// frc::SmartDashboard::PutNumber("l position", GetLeftPosition());
// frc::SmartDashboard::PutNumber("r position", GetRightPosition());

	l_error_vel_t = ref_left - l_current;
	r_error_vel_t = ref_right - r_current;
	//kick_error_vel = ref_kick - kick_current;


  // frc::SmartDashboard::PutNumber("l current", l_current);
  // frc::SmartDashboard::PutNumber("r current", r_current);
	//
  // 	frc::SmartDashboard::PutNumber("l vel targ", ref_left);
  // 	frc::SmartDashboard::PutNumber("r vel targ", ref_right);

	d_left_vel = (l_error_vel_t - l_last_error_vel);
	d_right_vel = (r_error_vel_t - r_last_error_vel);
	d_kick_vel = (kick_error_vel - kick_last_error_vel);

  // frc::SmartDashboard::PutNumber("l vel error", l_error_vel_t);
  // frc::SmartDashboard::PutNumber("r vel error", r_error_vel_t);
	//
  // frc::SmartDashboard::PutNumber("l vel k", k_p_left);
  // frc::SmartDashboard::PutNumber("r vel k", k_p_right);
	//
	// frc::SmartDashboard::PutNumber("k p l", k_p_left);
	//   frc::SmartDashboard::PutNumber("k p r", k_p_right);
	//  frc::SmartDashboard::PutNumber("R2",canTalonRight2->GetOutputCurrent());

	P_LEFT_VEL = k_p_left * l_error_vel_t;
	P_RIGHT_VEL = k_p_right * r_error_vel_t;
	P_KICK_VEL = k_p_kick * kick_error_vel;

	D_LEFT_VEL = k_d_left * d_left_vel;
	D_RIGHT_VEL = k_d_right * d_right_vel;
	D_KICK_VEL = k_d_kick * d_kick_vel;

   // frc::SmartDashboard::PutNumber("L2", drive_controller->canTalonLeft2->GetOutputCurrent());
  ///frc::SmartDashboard::PutNumber("R1", canTalonRight1->GetOutputCurrent());
  // //  frc::SmartDashboard::PutNumber("R1", drive_controller->GetRightVel());
  //  frc::SmartDashboard::PutNumber("R2",canTalonRight2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("R3", canTalonRight3->GetOutputCurrent());
  //
  //  frc::SmartDashboard::PutNumber("L1", canTalonLeft1->GetOutputCurrent());
  // //  frc::SmartDashboard::PutNumber("R1", drive_controller->GetRightVel());
  //  frc::SmartDashboard::PutNumber("L2", canTalonLeft2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("L3", canTalonLeft3->GetOutputCurrent());

  // frc::SmartDashboard::PutNumber("D r Vel", D_RIGHT_VEL *550.0);
  // frc::SmartDashboard::PutNumber("P r Vel", P_RIGHT_VEL*550.0);
  // frc::SmartDashboard::PutNumber("D l Vel", D_LEFT_VEL*550.0);
  // frc::SmartDashboard::PutNumber("P l Vel", P_LEFT_VEL*550.0);


	double total_right = D_RIGHT_VEL + P_RIGHT_VEL + feed_forward_r
			+ (Kv * target_vel_right * ff_scale); //Kv only in auton, straight from motion profile
	double total_left = D_LEFT_VEL + P_LEFT_VEL + feed_forward_l
			+ (Kv * target_vel_left * ff_scale);
//	double total_kick = D_KICK_VEL + P_KICK_VEL + feed_forward_k
//			+ (Kv_KICK * target_vel_kick);

	if (total_right > 1.0) {
		total_right = 1.0;
	} else if (total_right < -1.0) {
		total_right = -1.0;
	}
	if (total_left > 1.0) {
		total_left = 1.0;
	} else if (total_left < -1.0) {
		total_left = -1.0;
	}

	// frc::SmartDashboard::PutNumber("% OUT LEFT", total_left);
	// frc::SmartDashboard::PutNumber("% OUT RIGHT", total_right);

  canTalonLeft1->Set(ControlMode::PercentOutput, reverse_output * total_left);
	canTalonRight1->Set(ControlMode::PercentOutput, -total_right);

	yaw_last_error = yaw_error;
	l_last_error_vel = l_error_vel_t;
	r_last_error_vel = r_error_vel_t;
	kick_last_error_vel = kick_error_vel;
	l_last_current = l_current;


}

void DriveBase::ZeroAll(bool stop_motors) {

	if (stop_motors) {
		StopAll();
	}

	ZeroI();
	ZeroEncs();
	ZeroYaw();

	zeroing_counter++;

}

//will stop all driven motors in the drive controller
void DriveBase::StopAll() {

	canTalonLeft1->Set(ControlMode::PercentOutput, 0.0);
	canTalonRight1->Set(ControlMode::PercentOutput, 0.0);

}

//sets the position of all the drive encoders to 0
void DriveBase::ZeroEncs() {

	canTalonRight1->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft1->SetSelectedSensorPosition(0, 0, 0);

}

void DriveBase::ZeroYaw() {

	ahrs->ZeroYaw();

}

double DriveBase::GetLeftVel() { //550 left back //590 left forward

	double l_current = -1.0 * ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	return l_current;
}

double DriveBase::GetRightVel() { //580 right back //580 right forward

	double r_current = -1.0 * ((double) canTalonRight1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	return r_current;
}

double DriveBase::GetYawPos() {

	double y_dis = -1.0 * ahrs->GetYaw() * (double) (PI / 180);
	return y_dis;

}

//Zeros the accumulating I
void DriveBase::ZeroI() {

	i_right = 0;
	i_left = 0;
	i_yaw = 0;

	y_error_dis_au = 0;
	l_error_dis_au = 0;
	r_error_dis_au = 0;

}

//profile input from the autonomous routine selector, most likely in robot.cpp
void DriveBase::SetAutonRefs(std::vector<std::vector<double>> profile) {

	set_profile = true;
	auton_profile = profile;
	row_index = 0;
	zeroing_counter = 0;

}

void DriveBase::SetMaxRpm(double rpm) {

	max_y_rpm = rpm;

}

double DriveBase::GetMaxRpm() {

	return max_y_rpm;

}

void DriveBase::StopProfile(bool stop_profile) {

	continue_profile = !stop_profile;

}

double DriveBase::GetLeftPosition() {

	double l_dis = -1.0 * ((double) canTalonLeft1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

	return l_dis;

}

double DriveBase::GetRightPosition() {

	double r_dis = -1.0 * ((double) canTalonRight1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

	return r_dis;

}

//returns rad
double DriveBase::GetRoll() {

	return (ahrs->GetRoll() * 3.14159 / 180.0);

}

bool DriveBase::IsLastIndex() {

	return is_last_index;

}

int DriveBase::GetDriveIndex() {

	return row_index;

}

void DriveBase::SetZeroingIndex(std::vector<int> zero_index) {

	zeroing_index = zero_index;

}

std::vector<std::vector<double> > DriveBase::GetAutonProfile() {

	return vision_profile;

}

//Increments through target points of the motion profile
//Pre: SetAutonRefs()
//		row_index = 0, vision_profile filled, zeroing_indeces filled if needed, zero_counter = 0 if needed, continue_profile set as needed
void DriveBase::RunAutonProfile() {

  //fill next point horizontally
	for (int i = 0; i < vision_profile[0].size(); i++) {
		auton_row.at(i) = vision_profile.at(row_index).at(i);
	}

	//zeroing profile for next segment
	if (zeroing_index.size() > 0) {
		next_zero_index = zeroing_index.at(zero_counter);
	}
	if (row_index == next_zero_index) {
		StopAll();
		if (zero_wait_counter < 10) {
			ZeroAll(true);
			zero_wait_counter++;
		} else {
			if (zero_counter < (zeroing_index.size() - 1)) {
				zero_counter++;
			}
			zero_wait_counter = 0; //for the next time we need to zero
			row_index++; //break out of this if
		}
	} else {
		if (continue_profile && row_index < vision_profile.size()) {
			AutonDrive();
			row_index++;
		} else {
			StopAll();
		}
	}

}

//Increments through target points of the motion profile
void DriveBase::RunVisionProfile() {

	//fill next point
	for (int i = 0; i < vision_profile[0].size(); i++) { //can reuse after auto
		auton_row.at(i) = vision_profile.at(row_index).at(i);
	}

	AutonDrive();
	row_index++;

}

void DriveBase::RunTeleopDrive(Joystick *JoyThrottle,
	Joystick *JoyWheel, bool is_regular, bool is_vision, bool is_rotation) {

		if (is_regular) {
			teleop_drive_state = REGULAR;
		} else if (is_vision) {
			teleop_drive_state = VISION_DRIVE;
		} else if (is_rotation) {
			teleop_drive_state = ROTATION_CONTROLLER;
		}

		switch (teleop_drive_state) {
			case REGULAR:
			frc::SmartDashboard::PutString("DRIVE", "reg");
			TeleopWCDrive(JoyThrottle, JoyWheel);
			break;
			case VISION_DRIVE:
			frc::SmartDashboard::PutString("DRIVE", "vis");
			is_vision_done = VisionDriveStateMachine();
			if (is_vision_done) {
				teleop_drive_state = REGULAR;
			}
			break;
			case ROTATION_CONTROLLER:
			frc::SmartDashboard::PutString("DRIVE", "rot");
			RotationController(JoyWheel);
			break;
		}

}


void DriveBase::RunAutonDrive(Joystick *JoyThrottle,
	Joystick *JoyWheel, bool is_regular, bool is_vision, bool is_rotation) {

  switch (auton_drive_state) {

    case CREATE_AUTON_PROF:
		frc::SmartDashboard::PutString("DRIVE", "create prof");
    if (set_profile) {
      auton_drive_state = FOLLOW_AUTON_PROF;
    }
    break;
    case FOLLOW_AUTON_PROF:
		frc::SmartDashboard::PutString("DRIVE", "follow prof");
    RunAutonProfile();
    break;
    case RESET_AUTON:
		frc::SmartDashboard::PutString("DRIVE", "reset auton");
    //zeroall(true) ?
    break;
    case TELEOP_DRIVE: //clear old auton

    RunTeleopDrive(JoyThrottle, JoyWheel, is_regular, is_vision, is_rotation);
    break;

  }

}

bool DriveBase::VisionDriveStateMachine() {

	switch (vision_drive_state) {

		case CREATE_VIS_PROF:
			frc::SmartDashboard::PutString("VIS DRIVE", "create prof");
		  GenerateVisionProfile(visionDrive->GetDepthToTarget(), visionDrive->GetYawToTarget());
			if (set_profile) {
				vision_drive_state = FOLLOW_VIS_PROF;
			}
			return false;
		break;
		case FOLLOW_VIS_PROF: //TODO: add button for user to end visionDrive
			frc::SmartDashboard::PutString("VIS DRIVE", "follow prof");
			RunVisionProfile();
			if (row_index >= vision_profile.size()) {
				vision_drive_state = RESET_VIS;
			}
			return false;
		break;
		case RESET_VIS:
			frc::SmartDashboard::PutString("VIS DRIVE", "reset prof");
			StopAll();
		  return true;
		break;

	}

}
