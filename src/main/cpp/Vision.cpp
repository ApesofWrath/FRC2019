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
