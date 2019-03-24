#include "Vision.h"

nt::NetworkTableEntry yawEntry;
nt::NetworkTableEntry depthEntry;

Vision::Vision() {


}

double Vision::GetYawToTarget() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetNumber("yaw", -1.0);
}

double Vision::GetDepthToTarget() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetNumber("depth", -1.0) * 3.28084; // convert from meters to feet

}

double Vision::GetRobotExitAngle() {
  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetNumber("exit_angle", -1.0);
}



	// Controller(0.0, 0.0, 0.0, total_heading_h, k_p_right_vel, k_p_left_vel,
	// 		0.0, k_p_yaw_h_vel, 0.0, k_d_right_vel, k_d_left_vel,
	// 		0.0, 0.0, 0.0, 0.0);
