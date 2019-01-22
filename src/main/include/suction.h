#include <WPILib.h>


class Suction {
    public:
      DigitalOutput *suction_out;

      Suction(int channel);
      void Pull();
      void Push();
      void SuctionStateMachine();

}
