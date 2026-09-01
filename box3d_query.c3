module b3;

<*
 Fires a ray into the world and reports the closest shape it struck.

 The ray runs from origin to origin + translation, so the returned fraction multiplies
 translation, not a normalised direction. Initial overlap is ignored: a ray starting inside a
 shape does not report that shape. This is the convenience form and does no custom filtering
 beyond the query filter — WorldId.cast_ray() with a callback is the general one.

 @param filter : "obtained from default_query_filter(); an unmodified filter considers every shape"
 @return "the closest hit, or NO_HIT when the ray struck nothing"
*>
fn RayResult? WorldId.cast_ray_closest(self, Pos origin, Vec3 translation, QueryFilter filter) {
    RayResult result = cast_ray_closest_raw(
      world_id: self,
      origin: origin,
      translation: translation,
      filter: filter,
    );
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Fires a ray at this shape alone, skipping the broad phase. The shape's current transform is
 used, so this is the query to reach for when the shape is known and the world does not need
 searching.

 @return "the hit, or NO_HIT when the ray missed the shape"
*>
fn CastOutput? ShapeId.ray_cast(self, Pos origin, Vec3 translation) {
    CastOutput result = shape_ray_cast_raw(self, origin, translation);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Fires a ray at this body alone using a transform the caller supplies, which need not be the
 body's current one — passing a future transform is how a cast against a predicted pose is done.

 @param max_fraction : "the portion of translation to consider, normally 1"
 @return "the closest hit on this body, or NO_HIT when the ray missed it"
*>
fn BodyCastResult? BodyId.cast_ray(
  self,
  Pos origin,
  Vec3 translation,
  QueryFilter filter,
  float max_fraction,
  WorldTransform body_transform,
) {
    BodyCastResult result = body_cast_ray_raw(
      body_id: self,
      origin: origin,
      translation: translation,
      filter: filter,
      max_fraction: max_fraction,
      body_transform: body_transform,
    );
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps a convex proxy at this body alone using a transform the caller supplies. The proxy's
 points are borrowed for the duration of the call and its count must not exceed
 MAX_SHAPE_CAST_POINTS. The reported fraction stops short of true contact by box3d's speculative
 margin, so an exact geometric expectation will not match.

 @param can_encroach : "allows the cast to start touching; only meaningful when the proxy radius is above zero"
 @return "the closest hit on this body, or NO_HIT when the sweep missed it"
*>
fn BodyCastResult? BodyId.cast_shape(
  self,
  Pos origin,
  ShapeProxy* proxy,
  Vec3 translation,
  QueryFilter filter,
  float max_fraction,
  bool can_encroach,
  WorldTransform body_transform,
) {
    BodyCastResult result = body_cast_shape_raw(
      body_id: self,
      origin: origin,
      proxy: proxy,
      translation: translation,
      filter: filter,
      max_fraction: max_fraction,
      can_encroach: can_encroach,
      body_transform: body_transform,
    );
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 The point on this body nearest a world target, with the distance to it.

 box3d computes this with shape radii excluded, so for a sphere or a capsule the point lies on
 the centre line and the distance is larger than the gap to the surface by the radius. Only
 spheres, capsules and hulls are considered; a body carrying none of them reports its own origin
 at the largest finite float distance, not at zero. A body whose world is locked reports the zero
 vector at a distance of zero.
*>
fn ClosestPoint BodyId.closest_point(self, Vec3 target) {
    Vec3 point;
    float distance = body_closest_point_raw(self, &point, target);
    return { .point = point, .distance = distance };
}

struct CastCollector @private {
    CastHit[] buffer;
    usz count;
}

struct OverlapCollector @private {
    ShapeId[] buffer;
    usz count;
}

fn float collect_cast_hit(
  ShapeId shape_id,
  Pos point,
  Vec3 normal,
  float fraction,
  ulong user_material_id,
  int triangle_index,
  int child_index,
  void* context,
) @private {
    CastCollector* collector = (CastCollector*)context;
    if (collector.count >= collector.buffer.len) return 0.0f;

    collector.buffer[collector.count] = {
      .shape_id = shape_id,
      .point = point,
      .normal = normal,
      .fraction = fraction,
      .user_material_id = user_material_id,
      .triangle_index = triangle_index,
      .child_index = child_index,
    };
    collector.count++;

    return collector.count < collector.buffer.len ? 1.0f : 0.0f;
}

fn bool collect_overlap(ShapeId shape_id, void* context) @private {
    OverlapCollector* collector = (OverlapCollector*)context;
    if (collector.count >= collector.buffer.len) return false;

    collector.buffer[collector.count] = shape_id;
    collector.count++;

    return collector.count < collector.buffer.len;
}

<*
 Fires a ray into the world and collects every shape along it into the caller's buffer, in no
 particular order — the ray is not clipped, so a shape behind another is still reported.

 The buffer is the only limit: once it is full the cast ends, so a result as long as the buffer
 means there may have been more. Nothing is allocated and the returned slice points into the
 buffer, so it lives exactly as long as the buffer does.

 @return "the prefix of buffer that was filled"
*>
fn CastHit[] WorldId.cast_ray_all(
  self,
  Pos origin,
  Vec3 translation,
  QueryFilter filter,
  CastHit[] buffer,
) {
    CastCollector collector = { .buffer = buffer, .count = 0 };
    self.cast_ray(
      origin: origin,
      translation: translation,
      filter: filter,
      fcn: &collect_cast_hit,
      context: &collector,
    );
    return buffer[:collector.count];
}

<*
 Sweeps a convex proxy through the world and collects every shape it reaches into the caller's
 buffer. The proxy's points are borrowed for the call and its count must not exceed
 MAX_SHAPE_CAST_POINTS. Buffer behaviour matches cast_ray_all; reported fractions stop short of
 true contact by box3d's speculative margin.

 @return "the prefix of buffer that was filled"
*>
fn CastHit[] WorldId.cast_shape_all(
  self,
  Pos origin,
  ShapeProxy* proxy,
  Vec3 translation,
  QueryFilter filter,
  CastHit[] buffer,
) {
    CastCollector collector = { .buffer = buffer, .count = 0 };
    self.cast_shape(
      origin: origin,
      proxy: proxy,
      translation: translation,
      filter: filter,
      fcn: &collect_cast_hit,
      context: &collector,
    );
    return buffer[:collector.count];
}

<*
 Collects the shapes whose broad-phase bounds overlap the given box. This is the broad-phase
 test, so a reported shape may not truly intersect the box. Buffer behaviour matches
 cast_ray_all.

 @return "the prefix of buffer that was filled"
*>
fn ShapeId[] WorldId.overlap_aabb_all(self, Aabb aabb, QueryFilter filter, ShapeId[] buffer) {
    OverlapCollector collector = { .buffer = buffer, .count = 0 };
    self.overlap_aabb(
      aabb: aabb,
      filter: filter,
      fcn: &collect_overlap,
      context: &collector,
    );
    return buffer[:collector.count];
}

<*
 Collects the shapes that overlap a convex proxy placed at origin. The proxy's points are
 borrowed for the call and its count must not exceed MAX_SHAPE_CAST_POINTS. Buffer behaviour
 matches cast_ray_all.

 @return "the prefix of buffer that was filled"
*>
fn ShapeId[] WorldId.overlap_shape_all(
  self,
  Pos origin,
  ShapeProxy* proxy,
  QueryFilter filter,
  ShapeId[] buffer,
) {
    OverlapCollector collector = { .buffer = buffer, .count = 0 };
    self.overlap_shape(
      origin: origin,
      proxy: proxy,
      filter: filter,
      fcn: &collect_overlap,
      context: &collector,
    );
    return buffer[:collector.count];
}

<*
 The contact manifolds behind this contact.

 The slice borrows box3d's own memory: the next step() recomputes it and destroying the world
 frees it, so anything that must outlive either is copied out. The durable handle is the contact
 id, which fetches the geometry again through ContactId.data().

 @return "the manifolds, empty when the contact carries none"
*>
fn Manifold[] ContactData.contact_manifolds(self) {
    return self.manifolds[:self.manifold_count];
}

<*
 The valid contact points of this manifold — the first point_count of its four slots.

 @return "the points that carry contact geometry, between none and MAX_MANIFOLD_POINTS of them"
*>
fn ManifoldPoint[] Manifold.contact_points(&self) {
    return self.points[:self.point_count];
}

<*
 Writes this body's touching contacts into the caller's buffer.

 Only touching contacts are reported, which is why contact_capacity() is an upper bound rather
 than a count. Nothing is allocated: the returned slice is the filled prefix of the buffer, and a
 result as long as the buffer means there may have been more. Each ContactData carries manifolds
 that borrow live state — see ContactData.contact_manifolds().

 @return "the prefix of buffer that was filled"
*>
fn ContactData[] BodyId.contact_data(self, ContactData[] buffer) {
    int written = body_contact_data_raw(self, buffer.ptr, (int)buffer.len);
    return buffer[:written];
}

<*
 Writes the touching contacts involving this shape into the caller's buffer. This shape is
 shape_id_a or shape_id_b on every contact reported. A sensor reports none. Buffer behaviour
 matches BodyId.contact_data().

 @return "the prefix of buffer that was filled"
*>
fn ContactData[] ShapeId.contact_data(self, ContactData[] buffer) {
    int written = shape_contact_data_raw(self, buffer.ptr, (int)buffer.len);
    return buffer[:written];
}
