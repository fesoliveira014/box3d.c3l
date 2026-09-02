module b3;

<*
 Links two bodies by a spring between the z axis of local frame A and the z axis of local frame B,
 holding those parallel and holding nothing else — the gate on max_torque that ParallelJointDef
 describes turns even that off. The definition must come from default_parallel_joint_def() and
 must name two bodies of this world, the same guard on the same embedded base that
 create_filter_joint describes, stopping in the same place.

 @param [&in] def : "a definition obtained from default_parallel_joint_def(), naming two bodies of this world"
 @return "the joint, or the faults create_filter_joint documents"
*>
fn ParallelJointId? WorldId.create_parallel_joint(self, ParallelJointDef* def) {
    check_joint_def(self, &def.base)!;
    JointId joint = create_parallel_joint_raw(self, def).checked()!;
    return (ParallelJointId)joint;
}

<*
 Narrows a joint to the handle its parallel accessors take. Those accessors resolve through a
 joint-type assertion this release build compiles out, so one reaching a joint of another type
 reinterprets that joint's solver union — nonsense from a getter, an overwrite from a setter.
 Liveness is checked first: a destroyed joint's type resolves through the same compiled-out check.

 @return "the parallel handle, INVALID_HANDLE when the joint no longer exists, or WRONG_JOINT_TYPE when this joint is of another type"
*>
fn ParallelJointId? JointId.as_parallel(self) {
    if (!self.is_valid()) return INVALID_HANDLE~;
    if (self.type() != JointType.PARALLEL) return WRONG_JOINT_TYPE~;
    return (ParallelJointId)self;
}
