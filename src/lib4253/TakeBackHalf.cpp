#include "main.h"

class TakeBackHalf{
  double targetRPM;
  double currentRPM, prevRPM;
  double error, prevError;
  double power, prevPower;
  double TBHGain;

  void setGain(double gain){
    TBHGain = gain;
  }

  void setRPM(double target){
    targetRPM = target;
  }

  void initialize(){
    
  }

  double step(double rpm){

  }


};
