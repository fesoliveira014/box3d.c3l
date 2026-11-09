module b3;

<*
 Links two bodies with a planar velocity drive: body B is pushed towards linear_velocity relative
 to body A, in the xz plane of local frame B, with at most max_force newtons. A default definition
 drives nothing. That default is also required, naming two bodies of this world — the guard
 check_joint_def describes.

 @param [&in] def : `a definition from default_mover_joint_def(), naming two bodies of this world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn MoverJointId? create_mover_joint(WorldId world, MoverJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(world, &def.base)!;
    JointId joint = create_mover_joint_raw(world, def).checked()!;
    return (MoverJointId)joint;
}

<*
 Narrows a joint to the handle its mover accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the mover handle`
 @return? check_joint_type!
*>
fn MoverJointId? JointId.as_mover(self) {
    check_joint_type(self, JointType.MOVER)!;
    return (MoverJointId)self;
}
