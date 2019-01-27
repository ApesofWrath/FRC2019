const int INIT_STATE_H = 0;
const int STOP_STATE_H = 1;
const int IN_BALL_STATE_H = 2;
const int OUT_BALL_STATE_H = 3;

const int STOP_SPEED = 0.2; //placeholder #s put in correct ones
const int IN_SPEED = 0.95;
const int STOP_SPEED = 0.95;

int intake_state = STOP_STATE;

class Ballintake {
  public:
  Ballintake();
  void Init();
  void Stop();
  void In();
  void Out();
  void IntakeStateMachine();

}
