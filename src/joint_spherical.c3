module b3;

<*
 Links two bodies with a ball and socket: local frame B's origin pinned to local frame A's, the
 orientation about it bounded only by whichever of the cone limit, twist limit, spring and motor
 the definition enables. Both limited angles measure the frames' relative orientation, never their
 position, so a joint swung as a pendulum leaves both at 0. The definition must come from
 default_spherical_joint_def() and name two bodies of this world — the guard check_joint_def
 describes; the constructor silently clamps the cone angle and orders the twist pair.

 @param [&in] def : `a definition from default_spherical_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn SphericalJointId? create_spherical_joint(WorldId world, SphericalJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(world, &def.base)!;
    JointId joint = create_spherical_joint_raw(world, def).checked()!;
    return (SphericalJointId)joint;
}

<*
 Narrows a joint to the handle its spherical accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the spherical handle`
 @return? check_joint_type!
*>
fn SphericalJointId? JointId.as_spherical(self) {
    check_joint_type(self, JointType.SPHERICAL)!;
    return (SphericalJointId)self;
}
