#pragma once
#include "lib4253/Splines/Point2D.hpp" // done
#include "lib4253/Splines/SimplePath.hpp" // done
#include "lib4253/Splines/Trajectory.hpp" // done

#include "lib4253/Utility/Math.hpp" // done
#include "lib4253/Utility/Matrix.hpp"
#include "lib4253/Utility/TaskWrapper.hpp"

#include "lib4253/Filter/Filter.hpp" 
#include "lib4253/Filter/EMA.hpp" // done
#include "lib4253/Filter/DEMA.hpp"
#include "lib4253/Filter/SMA.hpp" // done
#include "lib4253/Filter/Kalman.hpp"
#include "lib4253/Filter/Biquad.hpp" // done

#include "lib4253/Controller/MotorVelocity.hpp"
#include "lib4253/Controller/PID.hpp" // done
#include "lib4253/Controller/Slew.hpp" // done
#include "lib4253/Controller/PurePursuit.hpp"
#include "lib4253/Controller/BangBang.hpp" // done
#include "lib4253/Controller/TakeBackHalf.hpp" // done
#include "lib4253/Controller/LinearMotionProfile.hpp" // done

#include "lib4253/Subsystems/Motor.hpp"
#include "lib4253/Subsystems/Odometry.hpp" // done
#include "lib4253/Subsystems/Drive.hpp" // done

#include "auton.hpp" // done
