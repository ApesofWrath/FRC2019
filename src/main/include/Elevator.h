#include <WPILib.h>


class Elevator {

public:
  Elevator();
  void Start();
  void Stop();

private:
  void ElevatorStateMachine();
};
