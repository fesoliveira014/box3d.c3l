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

 A plane is skipped where its clip_velocity is false or its push is zero, and a solve writes a
 push only for a plane it had to move the delta out of — so a wall a completed solve never leaned
 on clips nothing, as does a slice no solve has been run over at all.
*>
fn Vec3 clip_vector(Vec3 vector, CollisionPlane[] planes) {
    return clip_vector_raw(vector, planes.ptr, (int)planes.len);
}

<*
 Collides a capsule mover with this body alone, under a transform the caller supplies. The mover
 and the planes are relative to origin, and one plane at most is written per shape.

 Only spheres, capsules and hulls are considered, a mesh, height field or compound on the body
 being skipped. A short buffer is filled and the rest dropped rather than refused, every prefix
 being a real answer, so one filled to its length may have had more. A locked world fills none.

 @return `the prefix of buffer that was filled`
*>
fn BodyPlaneResult[] BodyId.collide_mover(
  self,
  BodyPlaneResult[] buffer,
  Pos origin,
  Capsule* mover,
  QueryFilter filter,
  WorldTransform body_transform,
) {
    int written = body_collide_mover_raw(
      body_id: self,
      body_planes: buffer.ptr,
      plane_capacity: (int)buffer.len,
      origin: origin,
      mover: mover,
      filter: filter,
      body_transform: body_transform,
    );
    return buffer[:written];
}

<*
 Applies the definition's radial impulse to every dynamic body with a shape in reach; static and
 kinematic never move, and a locked world does nothing. A mesh shape on a dynamic body is worse
 than passed over: box3d builds it a proxy of no points and reads the first anyway before culling
 by distance, so one whose broad-phase box meets an axis-aligned box of radius plus falloff — wider
 than the impulse's reach — crashes unless mask_bits excludes it or its explosion_scale is zero.

 @return? INVALID_DEFINITION : `the position is not finite, radius or falloff is negative or not
   finite, or impulse_per_area is not finite — box3d asserts all four and this release build
   compiles those assertions out`
*>
fn void? WorldId.explode(self, ExplosionDef def) {
    if (!is_valid_position(def.position)) return INVALID_DEFINITION~;
    if (!is_valid_float(def.radius) || !(def.radius >= 0.0f)) return INVALID_DEFINITION~;
    if (!is_valid_float(def.falloff) || !(def.falloff >= 0.0f)) return INVALID_DEFINITION~;
    if (!is_valid_float(def.impulse_per_area)) return INVALID_DEFINITION~;

    world_explode_raw(self, &def);
}
