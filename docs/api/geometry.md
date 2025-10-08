# Geometry API
The Geometry API provides unit-safe 2D geometric primitives and transformations.
It defines a cohesive set of mathematical types for representing and manipulating planar rigid-body motion in SE(2), with all quantities expressed using the au unit library for compile-time dimensional safety.

## Key Features
- Fully unit-safe operations with compile-time dimensional analysis
- Right-handed, counter-clockwise (+Z) coordinate convention
- All angular quantities normalized to (-π, π] radians
- Geometric operations compatible with SE(2) algebra (pose composition, inversion, exponential/log maps)
- Numerical stability for small-angle approximations (< 1e-9 radians)

## References
- [Rotation](@ref rz::Rotation)
- [Point](@ref rz::Point)
- [Pose](@ref rz::Pose)
- [Transform](@ref rz::Transform)
- [Twist](@ref rz::Twist)
