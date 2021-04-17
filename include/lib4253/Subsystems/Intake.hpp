#pragma once
#include "main.h"

class Roller{
  public:
    enum State{
      IN, OUT, EJECT, AUTOINDEX, OFF
    };

    Roller(int tPort, int bPort);
    State getState();
    void setState(State s);
    void rollerTask(void *ptr);

  protected:
    Motor top, bottom;

  private:
    State rollerState = OFF;
    void eject();
    void autoindex();
    void updateState();
    void run();

    friend class Intake;
};

class Intake{
  public:
    enum State{
      OFF, IN, OUT, AUTOINDEX
    };

    Intake(int lPort, int rPort);
    State getState();
    void setState(State s);
    void intakeTask(void *ptr);

  protected:
    Motor left, right;

  private:
    State intakeState = OFF;
    void updateState();
    void run();
    friend class Roller;
};
