module b3;

<*
 Links two bodies with a slider: body B is left free to translate along the x axis of local frame
 A and nothing else, with an optional spring, limit and motor along it. The definition must come
 from default_prismatic_joint_def() and name two bodies of this world — the guard check_joint_def
 describes.

 @param [&in] def : `a definition from default_prismatic_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn PrismaticJointId? create_prismatic_joint(WorldId world, PrismaticJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(world, &def.base)!;
    JointId joint = create_prismatic_joint_raw(world, def).checked()!;
    return (PrismaticJointId)joint;
}

<*
 Narrows a joint to the handle its prismatic accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the prismatic handle`
 @return? check_joint_type!
*>
fn PrismaticJointId? JointId.as_prismatic(self) {
    check_joint_type(self, JointType.PRISMATIC)!;
    return (PrismaticJointId)self;
}
