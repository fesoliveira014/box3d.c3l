module b3;

<*
 Links two bodies by a spring between the z axis of local frame A and the z axis of local frame B,
 holding those parallel and holding nothing else — the gate on max_torque that ParallelJointDef
 describes turns even that off. The definition must come from default_parallel_joint_def() and
 name two bodies of this world — the guard check_joint_def describes.

 @param [&in] def : `a definition from default_parallel_joint_def(), naming two bodies of this
   world`
 @return `the joint`
 @return? check_joint_def!
 @return? INVALID_HANDLE : `the world is locked, mid step or inside a callback box3d runs during
   one`
*>
fn ParallelJointId? create_parallel_joint(WorldId world, ParallelJointDef* def) {
    if (def == null) return INVALID_DEFINITION~;
    check_joint_def(world, &def.base)!;
    JointId joint = create_parallel_joint_raw(world, def).checked()!;
    return (ParallelJointId)joint;
}

<*
 Narrows a joint to the handle its parallel accessors take — see check_joint_type
 for the hazard the two guards close.

 @return `the parallel handle`
 @return? check_joint_type!
*>
fn ParallelJointId? JointId.as_parallel(self) {
    check_joint_type(self, JointType.PARALLEL)!;
    return (ParallelJointId)self;
}
