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
 the centre line and the distance is larger than the gap to the surface by the radius. Shapes
 box3d cannot reduce to a convex proxy are skipped, and a body with none of them reports its own
 origin at a distance of zero.
*>
fn ClosestPoint BodyId.closest_point(self, Vec3 target) {
    Vec3 point;
    float distance = body_closest_point_raw(self, &point, target);
    return { .point = point, .distance = distance };
}
