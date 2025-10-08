cmake_minimum_required(VERSION 3.6)
project(RaidZeroLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20 -Wall -Wextra -Wshadow=local -Werror=shadow=local -Wnull-dereference -Wno-psabi -Wno-unused-function -pthread -g -O0 -fprofile-arcs -ftest-coverage -D THREADS_STD")

enable_testing()

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in external-dependencies/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external-dependencies )
if(result)
    message(FATAL_ERROR "CMake step for dependencies failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external-dependencies )
if(result)
    message(FATAL_ERROR "Build step for dependencies failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

include_directories(include)

add_executable(RaidZeroLib
        include/okapi/api/chassis/controller/chassisController.hpp
        include/okapi/api/chassis/controller/chassisControllerIntegrated.hpp
        include/okapi/api/chassis/controller/chassisControllerPid.hpp
        include/okapi/api/chassis/controller/chassisScales.hpp
        include/okapi/api/chassis/controller/odomChassisController.hpp
        include/okapi/api/chassis/controller/defaultOdomChassisController.hpp
        include/okapi/api/chassis/model/chassisModel.hpp
        include/okapi/api/chassis/model/hDriveModel.hpp
        include/okapi/api/chassis/model/readOnlyChassisModel.hpp
        include/okapi/api/chassis/model/skidSteerModel.hpp
        include/okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp
        include/okapi/api/chassis/model/threeEncoderXDriveModel.hpp
        include/okapi/api/chassis/model/xDriveModel.hpp
        include/okapi/api/control/async/asyncController.hpp
        include/okapi/api/control/async/asyncLinearMotionProfileController.hpp
        include/okapi/api/control/async/asyncMotionProfileController.hpp
        include/okapi/api/control/async/asyncPosIntegratedController.hpp
        include/okapi/api/control/async/asyncPositionController.hpp
        include/okapi/api/control/async/asyncPosPidController.hpp
        include/okapi/api/control/async/asyncVelIntegratedController.hpp
        include/okapi/api/control/async/asyncVelocityController.hpp
        include/okapi/api/control/async/asyncVelPidController.hpp
        include/okapi/api/control/async/asyncWrapper.hpp
        include/okapi/api/control/iterative/iterativeController.hpp
        include/okapi/api/control/iterative/iterativeMotorVelocityController.hpp
        include/okapi/api/control/iterative/iterativePositionController.hpp
        include/okapi/api/control/iterative/iterativePosPidController.hpp
        include/okapi/api/control/iterative/iterativeVelocityController.hpp
        include/okapi/api/control/iterative/iterativeVelPidController.hpp
        include/okapi/api/control/util/controllerRunner.hpp
        include/okapi/api/control/util/flywheelSimulator.hpp
        include/okapi/api/control/util/pathfinderUtil.hpp
        include/okapi/api/control/util/pidTuner.hpp
        include/okapi/api/control/util/settledUtil.hpp
        include/okapi/api/control/closedLoopController.hpp
        include/okapi/api/control/controllerInput.hpp
        include/okapi/api/control/controllerOutput.hpp
        include/okapi/api/control/offsettableControllerInput.hpp
        include/okapi/api/device/button/abstractButton.hpp
        include/okapi/api/device/button/buttonBase.hpp
        include/okapi/api/device/motor/abstractMotor.hpp
        include/okapi/api/device/rotarysensor/continuousRotarySensor.hpp
        include/okapi/api/device/rotarysensor/rotarySensor.hpp
        include/okapi/api/filter/averageFilter.hpp
        include/okapi/api/filter/composableFilter.hpp
        include/okapi/api/filter/demaFilter.hpp
        include/okapi/api/filter/ekfFilter.hpp
        include/okapi/api/filter/emaFilter.hpp
        include/okapi/api/filter/filter.hpp
        include/okapi/api/filter/filteredControllerInput.hpp
        include/okapi/api/filter/medianFilter.hpp
        include/okapi/api/filter/passthroughFilter.hpp
        include/okapi/api/filter/velMath.hpp
        include/okapi/api/odometry/odometry.hpp
        include/okapi/api/odometry/twoEncoderOdometry.hpp
        include/okapi/api/odometry/odomMath.hpp
        include/okapi/api/odometry/threeEncoderOdometry.hpp
        include/okapi/api/units/QAcceleration.hpp
        include/okapi/api/units/QAngle.hpp
        include/okapi/api/units/QAngularAcceleration.hpp
        include/okapi/api/units/QAngularJerk.hpp
        include/okapi/api/units/QAngularSpeed.hpp
        include/okapi/api/units/QArea.hpp
        include/okapi/api/units/QForce.hpp
        include/okapi/api/units/QFrequency.hpp
        include/okapi/api/units/QJerk.hpp
        include/okapi/api/units/QLength.hpp
        include/okapi/api/units/QMass.hpp
        include/okapi/api/units/QPressure.hpp
        include/okapi/api/units/QSpeed.hpp
        include/okapi/api/units/QTime.hpp
        include/okapi/api/units/QTorque.hpp
        include/okapi/api/units/QVolume.hpp
        include/okapi/api/units/RQuantity.hpp
        include/okapi/api/util/abstractRate.hpp
        include/okapi/api/util/logging.hpp
        include/okapi/api/util/timeUtil.hpp
        include/okapi/api/util/abstractTimer.hpp
        include/okapi/api/util/mathUtil.hpp
        include/okapi/api/util/supplier.hpp
        include/okapi/api/coreProsAPI.hpp
        include/okapi/api/odometry/point.hpp
        include/okapi/api/odometry/stateMode.hpp
        include/okapi/api/odometry/odomState.hpp
        
        # include/RaidZeroLib/api/Control/Async/AsyncVelBangBangController.hpp
        # include/RaidZeroLib/api/Control/Async/AsyncVelTBHController.hpp
        # include/RaidZeroLib/api/Control/Iterative/IterativeVelTBHController.hpp
        # include/RaidZeroLib/api/Control/Iterative/IterativeVelBangBangController.hpp
        # include/RaidZeroLib/api/Control/Feedforward/FeedforwardController.hpp
        # include/RaidZeroLib/api/Control/Feedforward/SimpleMotorFeedforward.hpp
        # include/RaidZeroLib/api/Chassis/AdaptivePurePursuitController.hpp
        # include/RaidZeroLib/api/Utility/StateMachine.hpp
        # include/RaidZeroLib/api/Utility/Util.hpp
        # include/RaidZeroLib/api/Utility/CrossPlatformThread.hpp
        # include/RaidZeroLib/api/Utility/CrossPlatformMutex.hpp
        include/RaidZeroLib/api/Utility/Math.hpp
        # include/RaidZeroLib/api/Pathing/DiscretePath.hpp
        # include/RaidZeroLib/api/Pathing/ParametricPath.hpp
        # include/RaidZeroLib/api/Pathing/CubicBezier.hpp
        # include/RaidZeroLib/api/Pathing/PiecewiseCubicBezier.hpp
        # include/RaidZeroLib/api/Trajectory/MotionProfile/TrapezoidalMotionProfile.hpp
        # include/RaidZeroLib/api/Trajectory/MotionProfile/MotionProfile.hpp
        # include/RaidZeroLib/api/Filter/SlewRate.hpp
        include/RaidZeroLib/api/Geometry/Point.hpp
        include/RaidZeroLib/api/Geometry/Rotation.hpp
        include/RaidZeroLib/api/Geometry/Transform.hpp
        include/RaidZeroLib/api/Geometry/Twist.hpp
        include/RaidZeroLib/api/Geometry/Pose.hpp
        # include/RaidZeroLib/api/Units/Units.hpp
        # include/RaidZeroLib/api.hpp
        
        # src/RaidZeroLib/api/Control/Async/AsyncVelTBHController.cpp
        # src/RaidZeroLib/api/Control/Async/AsyncVelBangBangController.cpp
        # src/RaidZeroLib/api/Control/Iterative/IterativeVelTBHController.cpp
        # src/RaidZeroLib/api/Control/Iterative/IterativeVelBangBangController.cpp
        # src/RaidZeroLib/api/Control/Feedforward/SimpleMotorFeedforward.cpp
        # src/RaidZeroLib/api/Chassis/AdaptivePurePursuitController.cpp
        # src/RaidZeroLib/api/Utility/CrossPlatformMutex.cpp
        src/RaidZeroLib/api/Utility/Math.cpp
        # src/RaidZeroLib/api/Utility/CrossPlatformThread.cpp
        # src/RaidZeroLib/api/Pathing/ParametricPath.cpp
        # src/RaidZeroLib/api/Pathing/DiscretePath.cpp
        # src/RaidZeroLib/api/Pathing/CubicBezier.cpp
        # src/RaidZeroLib/api/Pathing/PiecewiseCubicBezier.cpp
        # src/RaidZeroLib/api/Filter/SlewRate.cpp
        src/RaidZeroLib/api/Geometry/Twist.cpp
        src/RaidZeroLib/api/Geometry/Point.cpp
        src/RaidZeroLib/api/Geometry/Transform.cpp
        src/RaidZeroLib/api/Geometry/Rotation.cpp
        src/RaidZeroLib/api/Geometry/Pose.cpp
        
        # test/test/Control/Feedforward/SimpleMotorFeedforwardTest.cpp
        # test/test/Control/Iterative/IterativeVelBangBangControllerTest.cpp

        test/test/Geometry/RotationTest.cpp
        test/test/Geometry/PointTest.cpp
        test/test/Geometry/TwistTest.cpp
        test/test/Geometry/TransformTest.cpp
        test/test/Geometry/PoseTest.cpp

        # test/test/Pathing/DiscretePathTest.cpp
        # test/test/Pathing/ParametricPathTest.cpp
        # test/test/Pathing/CubicBezierTest.cpp
        # test/test/Pathing/PiecewiseCubicBezierTest.cpp

        # test/test/Units/UnitsTest.cpp

        # test/test/Utility/MathTest.cpp
        # test/test/Utility/StateMachineTest.cpp

        test/include/mocks.hpp
        test/src/mocks.cpp)

# Link against gtest and okapi
target_link_libraries(RaidZeroLib ${CMAKE_SOURCE_DIR}/test/firmware/OkapiLibTest.a gtest_main)