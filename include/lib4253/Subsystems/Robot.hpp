#include "main.h"

extern Controller master;

extern Motor LF;
extern Motor LB;
extern Motor RF;
extern Motor RB;

extern ADIEncoder leftEncoder;
extern ADIEncoder rightEncoder;
extern ADIEncoder midEncoder;
extern ADIButton leftAutonSelector;
extern ADIButton rightAutonSelector;

extern pros::Imu imuTop;
extern pros::Imu imuBottom;

extern std::vector<Motor> base;
extern std::vector<Motor> baseLeft;
extern std::vector<Motor> baseRight;

class Robot{
  public:
    static void setPower(std::vector<Motor> motor, double power);
    static void setBrakeMode(std::vector<Motor> motor, std::string mode);

    static void startTask(std::string name, void (*func)(void *), void *param);
    static bool taskExists(std::string name);
    static void endTask(std::string name);

    static void displayPosition(void *ptr);

    static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
};

enum competition{
  OPCONTROL, AUTONOMOUS, INITIALIZE, DISABLED
};

extern competition matchState;
