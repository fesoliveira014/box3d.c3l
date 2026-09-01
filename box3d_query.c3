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
