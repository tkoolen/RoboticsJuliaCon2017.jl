module RoboticsJuliaCon2017

using RigidBodyDynamics
using StaticArrays

function create_fourbar{T}(::Type{T})
    # gravitational acceleration
    g = -9.81

    # link lengths
    l_0 = 1.10
    l_1 = 0.5
    l_2 = 1.20
    l_3 = 0.75

    # link masses
    m_1 = 0.5
    m_2 = 1.0
    m_3 = 0.75

    # link center of mass offsets from the preceding joint axes
    c_1 = 0.25
    c_2 = 0.60
    c_3 = 0.375

    # moments of inertia about the center of mass of each link
    I_1 = 0.333
    I_2 = 0.537
    I_3 = 0.4

    # Rotation axis: negative y-axis
    axis = SVector(zero(T), -one(T), zero(T))

    # Construct the world rigid body and create a new mechanism.
    world = RigidBody{T}("world")
    fourbar = Mechanism(world; gravity = SVector(zero(T), zero(T), g))

    # Construct the spanning tree of the mechanism, consisting of bodies 1, 2,
    # and 3 connected by joints 1, 2, and 3.
    # link1 and joint1
    joint1 = Joint("joint1", Revolute(axis))
    inertia1 = SpatialInertia(CartesianFrame3D("inertia1_centroidal"), I_1*axis*axis', zeros(SVector{3, T}), m_1)
    link1 = RigidBody(inertia1)
    beforeJoint1ToWorld = eye(Transform3D, frame_before(joint1), default_frame(world))
    c1_to_joint = Transform3D(inertia1.frame, frame_after(joint1), SVector(c_1, 0, 0))
    attach!(fourbar, world, joint1, beforeJoint1ToWorld, link1, c1_to_joint)

    # link2 and joint2
    joint2 = Joint("joint2", Revolute(axis))
    inertia2 = SpatialInertia(CartesianFrame3D("inertia2_centroidal"), I_2*axis*axis', zeros(SVector{3, T}), m_2)
    link2 = RigidBody(inertia2)
    beforeJoint2ToAfterJoint1 = Transform3D(frame_before(joint2), frame_after(joint1), SVector(l_1, 0., 0.))
    c2_to_joint = Transform3D(inertia2.frame, frame_after(joint2), SVector(c_2, 0, 0))
    attach!(fourbar, link1, joint2, beforeJoint2ToAfterJoint1, link2, c2_to_joint)

    # link3 and joint3
    joint3 = Joint("joint3", Revolute(axis))
    inertia3 = SpatialInertia(CartesianFrame3D("inertia3_centroidal"), I_3*axis*axis', zeros(SVector{3, T}), m_3)
    link3 = RigidBody(inertia3)
    beforeJoint3ToWorld = Transform3D(frame_before(joint3), default_frame(world), SVector(l_0, 0., 0.))
    c3_to_joint = Transform3D(inertia3.frame, frame_after(joint3), SVector(c_3, 0, 0))
    attach!(fourbar, world, joint3, beforeJoint3ToWorld, link3, c3_to_joint)

    # Add loop joint between link2 and link3
    joint4 = Joint("joint4", Revolute(axis))
    beforeJoint4ToJoint2 = Transform3D(frame_before(joint4), frame_after(joint2), SVector(l_2, 0., 0.))
    joint3ToAfterJoint4 = Transform3D(frame_after(joint3), frame_after(joint4), SVector(-l_3, 0., 0.))
    attach!(fourbar, link2, joint4, beforeJoint4ToJoint2, link3, joint3ToAfterJoint4)

    fourbar
end


end # module
