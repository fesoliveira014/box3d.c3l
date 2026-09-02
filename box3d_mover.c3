module b3;

<*
 Moves target_delta out of the planes it violates and reports where it settled.

 A linear slop is added to each separation, so a delta the planes had to move comes to rest a slop
 inside them rather than on them, and each plane's push is rewritten from zero, not added to.
 iteration_count is the zero-based index of the pass that converged, so twenty is the ceiling and
 means none did, the delta then being whatever the twentieth pass left. An empty slice is answered.
*>
fn PlaneSolverResult solve_planes(Vec3 target_delta, CollisionPlane[] planes) {
    return solve_planes_raw(target_delta, planes.ptr, (int)planes.len);
}

<*
 Removes from vector the component running into each plane, leaving one running away whole.

 A plane is skipped where its clip_velocity is false or its push is zero, and a solve writes a push
 only for a plane it had to move the delta out of — so a wall a completed solve never leaned on
 clips nothing, as does a slice no solve has been run over at all.
*>
fn Vec3 clip_vector(Vec3 vector, CollisionPlane[] planes) {
    return clip_vector_raw(vector, planes.ptr, (int)planes.len);
}
