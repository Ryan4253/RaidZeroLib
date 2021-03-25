#include "main.h"

extern pros::Controller master;

extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB;

extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder rightEncoder;
extern pros::ADIEncoder midEncoder;
extern pros::ADIButton leftAutonSelector;
extern pros::ADIButton rightAutonSelector;

extern pros::Imu imuTop;
extern pros::Imu imuBottom;

extern std::vector<pros::Motor> base;
extern std::vector<pros::Motor> baseLeft;
extern std::vector<pros::Motor> baseRight;

class Robot{
  public:
    static void setPower(std::vector<pros::Motor> motor, double power);
    static void setBrakeMode(std::vector<pros::Motor> motor, std::string mode);

    static void startTask(std::string name, void (*func)(void *), void *param);
    static bool taskExists(std::string name);
    static void endTask(std::string name);

    static void displayPosition(void *ptr);

    static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
};
