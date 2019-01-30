#include "Vision.h"

nt::NetworkTableEntry yawEntry;
nt::NetworkTableEntry depthEntry;

Vision::Vision() {


}

auto Vision::GetYawToTarget() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetEntry("yaw");

}

auto Vision::GetDepthToTarget() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetEntry("depth"); //TODO: cast to double

}
