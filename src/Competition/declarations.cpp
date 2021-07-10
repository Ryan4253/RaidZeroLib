#include "main.h"
#include "declarations.hpp"
using namespace lib4253;
using namespace okapi::literals;

okapi::Controller master(okapi::ControllerId::master);

okapi::Motor LF(10, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor LB(9, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor RF(8, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor RB(7, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);


okapi::ADIEncoder leftEncoder('A', 'B', true);
okapi::ADIEncoder rightEncoder('E', 'F', false);
okapi::ADIEncoder midEncoder('C', 'D', false);

pros::Imu imuBottom(11);
pros::Imu imuTop(12);

okapi::ADIButton leftAutonSelector('G');
okapi::ADIButton rightAutonSelector('H');

std::shared_ptr<lib4253::Motor> leftBack;
std::shared_ptr<lib4253::Motor> leftFront;
std::shared_ptr<lib4253::Motor> leftTop;
std::shared_ptr<lib4253::Motor> rightBack;
std::shared_ptr<lib4253::Motor> rightFront;
std::shared_ptr<lib4253::Motor> rightTop;

std::shared_ptr<lib4253::Odometry> odom;
std::shared_ptr<lib4253::Chassis> chassis;
std::shared_ptr<lib4253::OdomController> odomController;

void initSubsystems(){
    leftBack = std::make_shared<lib4253::Motor>(1, AbstractMotor::gearset::blue, 1, std::tuple<double, double, double>{1, 1, 1});
    leftFront = std::make_shared<lib4253::Motor>(1, AbstractMotor::gearset::blue, 1, std::tuple<double, double, double>{1, 1, 1});
    leftTop = std::make_shared<lib4253::Motor>(1, AbstractMotor::gearset::blue, 1, std::tuple<double, double, double>{1, 1, 1});
    rightBack = std::make_shared<lib4253::Motor>(1, AbstractMotor::gearset::blue, 1, std::tuple<double, double, double>{1, 1, 1});
    rightFront = std::make_shared<lib4253::Motor>(1, AbstractMotor::gearset::blue, 1, std::tuple<double, double, double>{1, 1, 1});
    rightTop = std::make_shared<lib4253::Motor>(1, AbstractMotor::gearset::blue, 1, std::tuple<double, double, double>{1, 1, 1});

    odom = std::make_shared<ThreeWheelOdometry>(
        std::make_shared<ADIEncoder>('A', 'B', true), 
        std::make_shared<ADIEncoder>('C', 'D', false),
        std::make_shared<ADIEncoder>('E', 'F', false), 
        OdomDimension(3.389_in, 5.748_in, 3.389_in)
    );

    chassis = std::make_shared<Chassis>(
        std::initializer_list<std::shared_ptr<lib4253::Motor>>{leftFront, leftBack, leftTop},
        std::initializer_list<std::shared_ptr<lib4253::Motor>>{rightFront, rightBack, leftTop},
        ChassisScales({4.14_in, 12.44_in}, 360),
        std::make_shared<IMU>(3),
        std::move(std::make_unique<SlewController>(9, 256)),
        std::move(std::make_unique<PID>(0, 0, 0)),
        std::move(std::make_unique<PID>(0, 0, 0)),
        std::move(std::make_unique<PID>(0, 0, 0))
    );

    odomController = std::make_shared<OdomController>(
        chassis, odom, 3_in,
        std::move(std::make_unique<PID>(0, 0, 0)),
        std::move(std::make_unique<PID>(0, 0, 0)),
        std::move(std::make_unique<PID>(0, 0, 0)),
        std::move(std::make_unique<SlewController>(9, 256))
    );

    odom->reset();
    chassis->initialize();

    chassis->moveDistance(25_in, Settler::makeSettler().withMaxTime(5_s).withMaxError(0.5).withMaxDeriv(0).wait(200_ms));
    odomController->moveToPoint({3_in, 6_in}, 1.5, Settler::makeSettler().withMaxTime(40000_ms).withMaxError(1).wait(200_ms));
}

void startTask(){
    chassis->startTask();
    odom->startTask();
}

/*
  drive
    .withDimensions({4.35}, {36, 84}, {12})
    .withDrivePID({0, 0, 0}, {1, 1}, {1})
    .withTurnPID({0, 0, 0}, {1, 1}, {1})
    .withPurePursuit({6}, {2}, {1, 1})
    .withSlew(256, 9)
    .initialize();
    */

void initPaths(){
    //Robot::addPath("10", tenTile);
}