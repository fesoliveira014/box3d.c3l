module b3;

<*
 Links two bodies with a ball and socket: local frame B's origin pinned to local frame A's, the
 orientation about it bounded only by whichever of the cone limit, twist limit, spring and motor
 the definition enables. Both limited angles measure the frames' relative orientation, never their
 position, so a joint swung as a pendulum leaves both at 0. The definition must come from
 default_spherical_joint_def() and name two bodies of this world, the guard create_filter_joint
 describes; the constructor silently clamps the cone angle and orders the twist pair.

 @param [&in] def : "a definition obtained from default_spherical_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn SphericalJointId? WorldId.create_spherical_joint(self, SphericalJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_spherical_joint_raw(self, def).checked()!;
    return (SphericalJointId)joint;
}

<*
 Narrows a joint to the handle its spherical accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the spherical handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn SphericalJointId? JointId.as_spherical(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.SPHERICAL) return WRONG_JOINT_TYPE~;
    return (SphericalJointId)self;
}
