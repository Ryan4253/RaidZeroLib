#include "main.h"

class Roller{
  public:
    enum State{
      In, Out, Eject, Autoindex, Off
    };

    Roller(Motor a, Motor b);
    State getState();
    void setState(State s);
    void rollerTask(void *ptr);

  protected:
    Motor top, bottom;

  private:
    State rollerState;
    void eject();
    void autoindex();
    void updateState();
    void run();

    friend class Intake;
};

class Intake{
  public:
    enum State{
      Off, In, Out, Autoindex
    };

    Intake(Motor a, Motor b);
    State getState();
    void setState(State s);
    void intakeTask(void *ptr);

  protected:
    Motor left, right;

  private:
    State intakeState;
    void updateState();
    void run();
    friend void Roller::autoindex();
};

extern Roller roller;
extern Intake intake;
