#include "../../include/Drive/Vision.h"

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

  return table->GetNumber("depth", -1.0);

}
