module b3;

<*
 Links two bodies with a hinge: their local frames are held together and left free to turn about
 the frame's z axis, with an optional spring, limit and motor. The definition must come from
 default_revolute_joint_def() and name two bodies of this world — the guard check_joint_def
 describes.

 @param [&in] def : `a definition from default_revolute_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn RevoluteJointId? create_revolute_joint(WorldId world, RevoluteJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(world, &def.base)!;
    JointId joint = create_revolute_joint_raw(world, def).checked()!;
    return (RevoluteJointId)joint;
}

<*
 Narrows a joint to the handle its revolute accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the revolute handle`
 @return? check_joint_type!
*>
fn RevoluteJointId? JointId.as_revolute(self) {
    check_joint_type(self, JointType.REVOLUTE)!;
    return (RevoluteJointId)self;
}
