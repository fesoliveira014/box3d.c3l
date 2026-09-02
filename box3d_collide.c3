module b3;

// box3d's own degeneracy threshold, 1000 * FLT_MIN, which C3 0.8.3 cannot spell. Mirrored rather
// than approximated because below it box3d hands back a zero plane normal, and a triangle with no
// plane culls nothing and can answer with a NaN point.
const float TRIANGLE_AREA_FLOOR @private = 1.17549435e-35f;

fn bool triangle_has_area(Triangle* triangle) @private {
    Vec3 edge1 = (*triangle)[1] - (*triangle)[0];
    Vec3 edge2 = (*triangle)[2] - (*triangle)[0];
    Vec3 cross = {
      edge1[1] * edge2[2] - edge1[2] * edge2[1],
      edge1[2] * edge2[0] - edge1[0] * edge2[2],
      edge1[0] * edge2[1] - edge1[1] * edge2[0],
    };
    return cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2] > TRIANGLE_AREA_FLOOR;
}

<*
 The contact between this sphere and another placed by transform_b_to_a, in this sphere's frame,
 the normal pointing from this sphere towards the other.

 A point arrives while the two touch or overlap, so its separation is never positive — exactly
 zero where the surfaces meet, negative once they overlap. Coincident centres report minus the
 summed radii under a +y normal, box3d's fallback direction rather than a measured one.
*>
fn CollisionManifold Sphere.collide_sphere(&self, Sphere other, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_spheres_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      sphere_a: self,
      sphere_b: &other,
      transform_b_to_a: transform_b_to_a,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this capsule and a sphere placed by transform_b_to_a, in the capsule's frame,
 the normal pointing from the capsule towards the sphere.

 A point arrives while the two touch or overlap, so its separation is never positive — zero where
 the surfaces meet. A capsule whose centres coincide is collided as the sphere they name, and a
 sphere on the capsule's axis reports a +y normal, box3d's fallback where there is none to measure.
*>
fn CollisionManifold Capsule.collide_sphere(&self, Sphere sphere, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_capsule_and_sphere_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      capsule_a: self,
      sphere_b: &sphere,
      transform_b_to_a: transform_b_to_a,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this hull and a sphere placed by transform_b_to_a, in the hull's frame, the
 normal pointing from the hull towards the sphere.

 A four-slop speculative window, so a point can arrive with a positive separation for two shapes
 not yet touching. A sphere centre inside the hull is answered from the nearest face — box3d takes
 the greatest separation, which inside is the least negative.
*>
fn CollisionManifold HullData.collide_sphere(&self, Sphere sphere, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    SimplexCache warm_start;
    collide_hull_and_sphere_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      hull_a: self,
      sphere_b: &sphere,
      transform_b_to_a: transform_b_to_a,
      cache: &warm_start,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this capsule and another placed by transform_b_to_a, in this capsule's frame,
 the normal pointing towards the other. Two points arrive only when they are near parallel, overlap
 along their axes and touch; anything else gives one, and the four-slop window reaches only the
 one-point branch. Nothing at all is reported when either capsule is shorter than a linear slop, or
 the axes pass within a hundredth of one of each other — coaxial as well as crossing.
*>
fn CollisionManifold Capsule.collide_capsule(&self, Capsule other, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_capsules_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      capsule_a: self,
      capsule_b: &other,
      transform_b_to_a: transform_b_to_a,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this hull and a capsule placed by transform_b_to_a, in the hull's frame, the
 normal pointing from the hull towards the capsule. A four-slop speculative window, so a point can
 arrive with a positive separation.

 Two points come from clipping the capsule's segment onto a face, which box3d admits on either end
 alone and then reports both of, so the second can be far from contact — measured at +0.95.
*>
fn CollisionManifold HullData.collide_capsule(&self, Capsule capsule, Transform transform_b_to_a) {
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    SimplexCache warm_start;
    collide_hull_and_capsule_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      hull_a: self,
      capsule_b: &capsule,
      transform_b_to_a: transform_b_to_a,
      cache: &warm_start,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this hull and another placed by transform_b_to_a, in this hull's frame, the
 normal pointing towards the other. box3d reduces the clipped polygon to its own manifold size, so
 MAX_MANIFOLD_POINTS is a real bound here rather than a truncation. A four-slop speculative window,
 so a point can arrive with a positive separation — exactly zero where the two faces meet.

 @param other : "the second hull, which box3d reads before testing anything"
 @return "the contact, or INVALID_DEFINITION when other is null"
*>
fn CollisionManifold? HullData.collide_hull(&self, HullData* other, Transform transform_b_to_a) {
    if (other == null) return INVALID_DEFINITION~;

    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    SatCache warm_start;
    collide_hulls_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      hull_a: self,
      hull_b: other,
      transform_b_to_a: transform_b_to_a,
      cache: &warm_start,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this triangle and a sphere in the same frame, the normal pointing from the
 triangle towards the sphere. One point at most, and the cull reads the sphere's own centre, so a
 sphere reaching past the plane still meets nothing while its centre is behind. A four-slop window,
 so the separation is positive for a sphere not yet reached and zero where the surfaces meet.

 @return "the contact, or DEGENERATE_GEOMETRY when the triangle has no area"
*>
fn CollisionManifold? Triangle.collide_sphere(&self, Sphere sphere) {
    if (!triangle_has_area(self)) return DEGENERATE_GEOMETRY~;
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    collide_triangle_and_sphere_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      triangle_a: (Vec3*)self,
      sphere_b: &sphere,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this triangle and a capsule in the same frame, the normal pointing from the
 triangle towards the capsule. Two points at most, from clipping the segment onto the face, and one
 of them can be far from contact. The cull reads the midpoint of the two centres, so a capsule with
 one end well in front of the plane but its midpoint behind meets nothing.

 @return "the contact, or DEGENERATE_GEOMETRY when the triangle has no area"
*>
fn CollisionManifold? Triangle.collide_capsule(&self, Capsule capsule) {
    if (!triangle_has_area(self)) return DEGENERATE_GEOMETRY~;
    CollisionManifold contact;
    LocalManifold manifold = { .points = &contact.points[0] };
    SimplexCache warm_start;
    collide_triangle_and_capsule_raw(
      manifold: &manifold,
      capacity: MAX_MANIFOLD_POINTS,
      triangle_a: (Vec3*)self,
      capsule_b: &capsule,
      cache: &warm_start,
    );
    contact.normal = manifold.normal;
    contact.point_count = manifold.point_count;
    return contact;
}

<*
 The contact between this triangle and a hull in the same frame, the normal running from triangle
 to hull and the points written into buffer. Alone among the collide pairs its count is bounded by
 that buffer: box3d truncates the clipped polygon rather than reducing it, so four slots keep a
 clip-order prefix of a contact reaching seven. The cull reads the hull's centroid against a linear
 slop, so a hull a slop behind still answers where one further back does not.

 @param enable_speculative : "whether to report points the two have not reached, out to four linear slops"
 @param buffer : "where the points go; under MAX_MANIFOLD_POINTS box3d reports no contact at all, which is why that is refused"
 @return "the contact, DEGENERATE_GEOMETRY when the triangle has no area, INVALID_DEFINITION when hull is null, or BUFFER_TOO_SMALL when buffer holds fewer than MAX_MANIFOLD_POINTS"
*>
fn CollisionManifoldView? Triangle.collide_hull(
  &self,
  HullData* hull,
  bool enable_speculative,
  LocalManifoldPoint[] buffer,
) {
    if (!triangle_has_area(self)) return DEGENERATE_GEOMETRY~;
    if (hull == null) return INVALID_DEFINITION~;
    if (buffer.len < (usz)MAX_MANIFOLD_POINTS) return BUFFER_TOO_SMALL~;

    LocalManifold manifold = { .points = buffer.ptr };
    SatCache warm_start;
    Vec3* vertices = (Vec3*)self;
    collide_triangle_and_hull_raw(
      manifold: &manifold,
      capacity: (int)buffer.len,
      v1: vertices[0],
      v2: vertices[1],
      v3: vertices[2],
      // box3d discards this today, and the cull it once fed is `edgeFlags & flags`, so zero is
      // the value that stays inert if the line is ever restored.
      triangle_flags: 0,
      hull_b: hull,
      cache: &warm_start,
      enable_speculative: enable_speculative,
    );
    return { .normal = manifold.normal, .points = buffer[:manifold.point_count] };
}

<*
 The contact points this manifold carries — the first point_count of its slots. The slice borrows
 the manifold it was called on, so one taken straight from the call that produced a manifold
 outlives it: store the manifold first.

 @return "the points the pair produced, between none and MAX_MANIFOLD_POINTS of them"
*>
fn LocalManifoldPoint[] CollisionManifold.contact_points(&self) {
    return self.points[:self.point_count];
}
