#include<gtest/gtest.h>
#include<cmath>
#include "RaidZeroLib/api/Geometry/Rotation.hpp"
using namespace okapi;

TEST (RotationTest, constructor) { 
    rz::Rotation angle(3_ft, 3_ft);
    ASSERT_EQ(angle.Theta().convert(radian), ((M_PI)/2));
}


