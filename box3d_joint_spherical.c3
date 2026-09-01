module b3;

<*
 Links two bodies with a ball and socket: the origin of local frame B is pinned to the origin of
 local frame A and the orientation about that point is left free, bounded only by whichever of the
 cone limit, twist limit, spring and motor the definition enables. Both limited angles measure the
 two frames' relative orientation and never their position, as SphericalJointDef describes, so a
 joint built here and swung as a pendulum leaves both at 0. The definition must come from
 default_spherical_joint_def() and must name two bodies of this world, the same guard on the same
 embedded base that create_filter_joint describes, stopping in the same place — box3d's own checks
 on the cone angle and the target rotation are compiled out of this build and are not restored,
 and the constructor silently clamps that angle and orders the twist pair.

 @param [&in] def : "a definition obtained from default_spherical_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn SphericalJointId? WorldId.create_spherical_joint(self, SphericalJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_spherical_joint_raw(self, def).checked()!;
    return (SphericalJointId)joint;
}

<*
 Narrows a joint to the handle its spherical accessors take. box3d resolves each of those through
 a joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union instead — a getter hands back nonsense and a setter writes
 over the other type's state. This conversion is where that is caught; an explicit
 (SphericalJointId) cast asserts the type rather than checking it, which is what an explicit cast
 means in C3. The handle is checked live before its type is read at all, because reading the type
 of a destroyed joint resolves through an assertion compiled out the same way and reports whatever
 has since taken the slot.

 @return "the spherical handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn SphericalJointId? JointId.as_spherical(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.SPHERICAL) return WRONG_JOINT_TYPE~;
    return (SphericalJointId)self;
}
