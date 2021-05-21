#include "XDrive.hpp"
#include "Robot.hpp"

XDrive::XDrive(Motor leftFront, Motor leftBack, Motor rightFront, Motor rightBack);{
      base[0] = leftFront; base[1] = leftBack;
      base[2] = rightFront; base[3] = rightBack;
}

XDrive& XDrive::withOdometry(CustomOdometry* tracker){
      odom = tracker;
      return *this;
}

XDrive& withDrivePID(std::tuple<double, double, double> gain) {
      drivePID.setGain(std::get<0>(gain), std::get<1>(gain), std::get<2>(gain));
}

XDrive& withConfig(double motorRPM, brakeType brake){
      this->motorRPM = motorRPM;
      for(int i : this->base) {
            switch(brake){
                  case COAST: 
                  base[i].setBrakeMode(okapi::brakeMode::coast);
                  break;

                  case HOLD: 
                  base[i].setBrakeMode(okapi::brakeMode::hold);
                  break;

                  case BRAKE: 
                  base[i].setBrakeMode(okapi::brakeMode::brake);
                  break;
            }
      }
}

double XDrive::map(double value, double prevMin, double prevMax, double targetMin, double targetMax) {
      return targetMin + (targetMax - targetMin) * ((value - prevMin) / (prevMax - prevMin));
}

void XDrive::setDriveVel(std::array<int, 4> vel) {
      for (int i : vel) {
            double newVel = map(vel[i], -1, 1, -motorRPM, motorRPM);
            base[i].moveVelocity(newVel);
      }
}

void XDrive::setDriveVolt(std::array<int, 4> volt){
      for (int i : volt) {
            double newVolt = map(volt[i], -1, 1, -12000, 12000);
            base[i].moveVoltage(newVolt);
      }
}