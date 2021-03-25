#include "main.h"

class Drive{
  public:
    std::vector<pros::Motor> left;
    std::vector<pros::Motor> right;
    PID drivePID;
    PID turnPID;
    SlewController driveSlew;
    PurePursuitFollower PPTenshi;

    std::shared_ptr<ChassisController> chassis;
    std::shared_ptr<AsyncMotionProfileController> profileController;
    /*
     = ChassisControllerBuilder()
      .withMotors({9, 10}, {7, 8})
      .withDimensions(AbstractMotor::gearset::green, {{4.32_in, 12.25_in}, imev5GreenTPR})
      .withSensors(ADIEncoder{'A', 'B', true}, ADIEncoder{'C', 'D'})
      .build();
      */

    Drive(std::vector<pros::Motor> l, std::vector<pros::Motor> r);

    void resetEncoders();
    void resetIMU();
    double getDistance();
    double getAngle();
    Vector scaleSpeed(double linear, double turn, double turnScale);

    void moveDistance(double distance, QTime timeLimit);
    void moveTo(Vector target, double turnScale, QTime timeLimit);
    void turnAngle(double angle, QTime timeLimit);
    void turnToAngle(double angle, QTime timeLimit);

    void driverControl();
    static void taskFnc(void *bruh);
};

//extern int lmao;
extern Drive drive;
