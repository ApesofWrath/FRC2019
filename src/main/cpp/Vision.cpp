#include "Vision.h"

nt::NetworkTableEntry yawEntry;
nt::NetworkTableEntry depthEntry;

Vision::Vision() {


}

//return rad
double Vision::GetYawToTarget() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetNumber("yaw", -1.0);

}

//return feet
double Vision::GetDepthToTarget() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  return table->GetNumber("depth", -1.0);

}
