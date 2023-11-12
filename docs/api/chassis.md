# Chassis API

This API includes all the motion control algorithms that can be used for your robot's chassis in autonomous mode. They are all async, meaning they can be run concurrently with other parts of your robot. Make sure to not have two motion controllers control the chassis at the same time.
- [AdaptivePurePursuitController](@ref rz::AdaptivePurePursuitController)
