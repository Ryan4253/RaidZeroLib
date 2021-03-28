#include "main.h"

enum intakeState{
  iOff, iIn, iOut, iAutoIndex
};

enum rollerState{
  rOff, rIn, rOut, rAutoIndex, rEject
};

class Intakes{
  public:
    struct Roller{


      rollerState state;
      rollerState getState();
      void setState(rollerState i);
      void eject();

      Motor top;
      Motor bottom;
    };

    struct Intake{


      void setMotor(Motor l, Motor r){
        left = l, right = r;
      }

      intakeState state;
      intakeState getState();
      void setState(intakeState i);

      Motor left;
      Motor right;
    };

    Intakes(Motor rollerTop, Motor RollerBottom, Motor intakeLeft, Motor intakeRight);
    void autoIndex();
    Roller roller;
    Intake intake;
    void intakeTask(void* ptr);
};

extern Intakes intakeSystem;
