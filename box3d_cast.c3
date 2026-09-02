module b3;

<*
 Casts a ray at this sphere as a solid body, in the sphere's own frame, reporting a fraction of
 the ray's translation.

 A ray starting strictly inside reports a hit at its own origin with fraction 0 and a zero normal,
 which a caller normalises into a NaN, and a ray leaving the surface it starts on reports nothing.
 box3d asserts RayCastInput.is_valid() here and this build compiles that assertion out.

 @return "where the ray meets the sphere, or NO_HIT when it does not"
*>
fn CastOutput? Sphere.ray_cast(&self, RayCastInput input) {
    CastOutput result = sphere_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this sphere's surface alone, in the sphere's own frame, reporting the nearer of
 the two walls it reaches and the far one when the near one is behind the origin.

 box3d normalises the translation here and never divides the answer back down, so the fraction is
 a distance in the sphere's own units and max_fraction bounds that distance rather than a portion
 of the translation. A translation one unit long is what makes the two agree.

 @return "where the ray meets the shell, or NO_HIT when it does not"
*>
fn CastOutput? Sphere.ray_cast_hollow(&self, RayCastInput input) {
    CastOutput result = hollow_sphere_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this capsule in the capsule's own frame, reporting a fraction of the ray's
 translation. A capsule whose two centres nearly coincide is cast at as the sphere they average to.

 A ray starting strictly inside reports a hit at its own origin with fraction 0 and a zero normal,
 which a caller normalises into a NaN. box3d asserts RayCastInput.is_valid() here and this build
 compiles that assertion out.

 @return "where the ray meets the capsule, or NO_HIT when it does not"
*>
fn CastOutput? Capsule.ray_cast(&self, RayCastInput input) {
    CastOutput result = capsule_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this hull in the hull's own frame, reporting a fraction of the ray's translation.

 A ray starting inside every face plane reports a hit at its own origin with fraction 0 and a zero
 normal, which a caller normalises into a NaN. box3d asserts RayCastInput.is_valid() here and this
 build compiles that assertion out.

 @return "where the ray enters the hull, or NO_HIT when it does not"
*>
fn CastOutput? HullData.ray_cast(&self, RayCastInput input) {
    CastOutput result = hull_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this compound in the compound's own frame, reporting the nearest child it reaches
 in child_index and the material that child names in material_index.

 The fraction is a true fraction of the translation — every child dispatches to the solid cast for
 its kind, never the hollow one. A ray starting inside a child reports fraction 0 and a zero
 normal, which rotating by the child's placement leaves zero, so a caller normalising it gets NaN.

 @return "where the ray meets the nearest child, NO_HIT when it meets none, or INVALID_DEFINITION when max_fraction is above 1"
*>
fn CastOutput? CompoundData.ray_cast(&self, RayCastInput input) {
    if (input.max_fraction > 1.0) return INVALID_DEFINITION~;
    CastOutput result = compound_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this mesh's triangles, scaled by the mesh's own scale. A triangle answers from one
 side only: a ray meeting it from behind its winding reports nothing.

 A max_fraction above 1 is refused, because box3d's triangle solver returns 1 as its no-hit value
 and the search keeps whatever beats max_fraction — so a miss would come back as a hit at exactly
 1, carrying a real triangle index.

 @return "where the ray meets a triangle, NO_HIT when it meets none, or INVALID_DEFINITION when max_fraction is above 1"
*>
fn CastOutput? Mesh.ray_cast(&self, RayCastInput input) {
    if (input.max_fraction > 1.0) return INVALID_DEFINITION~;
    CastOutput result = mesh_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this height field's triangles, reporting a fraction of the translation and the
 triangle struck. Nothing is reported from underneath or over a hole cell, and a max_fraction
 above 1 is refused — the same solver as Mesh.ray_cast, refused for the same reason.

 box3d compresses the heights into 16 bits across the range the definition named, so the surface a
 cast meets sits under the heights it was built from by up to one step, scaled by the y scale.

 @return "where the ray meets the field, NO_HIT when it does not, or INVALID_DEFINITION when max_fraction is above 1"
*>
fn CastOutput? HeightFieldData.ray_cast(&self, RayCastInput input) {
    if (input.max_fraction > 1.0) return INVALID_DEFINITION~;
    CastOutput result = height_field_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps the input's proxy at this sphere in the sphere's own frame, reporting where the two first
 touch. The sphere lends its radius to the proxy's and box3d closes at one linear slop or that sum
 less the slop, whichever is greater — so the stop lands short of true contact below a total
 radius of one slop, on it at one, and past above, a full slop past from two. A pair beginning in
 contact reports fraction 0 under a zero normal, a NaN once normalised, unless can_encroach is set.

 @return "where the proxy first touches the sphere, NO_HIT when it never does, or INVALID_DEFINITION when the proxy carries no points"
*>
fn CastOutput? Sphere.shape_cast(&self, ShapeCastInput input) {
    check_shape_proxy(input.proxy)!;
    CastOutput result = sphere_shape_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps the input's proxy at this capsule in the capsule's own frame, reporting where the two
 first touch. The capsule lends its radius to the proxy's and box3d closes on the formula
 Sphere.shape_cast states, so an ordinary capsule radius alone carries the stop a full slop past
 true contact and reaches can_encroach's window with a proxy of no radius at all. A proxy that
 begins in contact reports fraction 0 under a zero normal unless that flag is set.

 @return "where the proxy first touches the capsule, NO_HIT when it never does, or INVALID_DEFINITION when the proxy carries no points"
*>
fn CastOutput? Capsule.shape_cast(&self, ShapeCastInput input) {
    check_shape_proxy(input.proxy)!;
    CastOutput result = capsule_shape_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps the input's proxy at this hull in the hull's own frame, reporting the fraction of the
 translation at which the two first touch.

 A hull lends no radius of its own, so the sum box3d closes on is the proxy's alone and a proxy of
 radius zero never reaches can_encroach's window: such a proxy starting inside reports fraction 0
 with a zero normal whatever that flag says.

 @return "where the proxy first touches the hull, NO_HIT when it never does, or INVALID_DEFINITION when the proxy carries no points"
*>
fn CastOutput? HullData.shape_cast(&self, ShapeCastInput input) {
    check_shape_proxy(input.proxy)!;
    CastOutput result = hull_shape_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps the input's proxy at this compound in the compound's own frame, reporting the nearest
 child it reaches in child_index and that child's material in material_index.

 Each child lends whatever radius it carries, so the fraction sits off that child's contact as
 casting at it alone would. A max_fraction above 1 is safe here where it is not on a ray: a child
 is a sphere, a capsule, a hull or a mesh, and none of those four sweeps answers a miss with one.

 @return "where the proxy first touches a child, NO_HIT when it touches none, or INVALID_DEFINITION when the proxy carries no points"
*>
fn CastOutput? CompoundData.shape_cast(&self, ShapeCastInput input) {
    check_shape_proxy(input.proxy)!;
    CastOutput result = compound_shape_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps the input's proxy at this mesh's triangles, scaled by the mesh's own scale, reporting the
 triangle struck and the fraction at which the proxy first touches it.

 A triangle is reached from either side here, where Mesh.ray_cast reaches it from the wound side
 alone: box3d measures a distance between the proxy and the triangle rather than solving a ray
 against its plane. That is also why a max_fraction above 1 needs no refusal on this one.

 @return "where the proxy first touches a triangle, NO_HIT when it touches none, or INVALID_DEFINITION when the proxy carries no points"
*>
fn CastOutput? Mesh.shape_cast(&self, ShapeCastInput input) {
    check_shape_proxy(input.proxy)!;
    CastOutput result = mesh_shape_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Sweeps the input's proxy at this height field's triangles, reporting the triangle struck and the
 fraction at which the proxy first touches it. Nothing is reported over a hole cell.

 A proxy of one point carrying no radius is cast with box3d's ray solver instead, which reads the
 field from above alone and reports a miss by returning 1 — so a max_fraction above 1 is refused
 for such a proxy, exactly as HeightFieldData.ray_cast refuses it. Any other proxy is two-sided.

 @return "where the proxy first touches the field, NO_HIT when it never does, or INVALID_DEFINITION when the proxy carries no points or a single point with no radius is given a max_fraction above 1"
*>
fn CastOutput? HeightFieldData.shape_cast(&self, ShapeCastInput input) {
    check_shape_proxy(input.proxy)!;
    if (input.max_fraction > 1.0 && input.proxy.count == 1 && input.proxy.radius == 0.0) {
        return INVALID_DEFINITION~;
    }
    CastOutput result = height_field_shape_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Whether the proxy touches this sphere placed by transform. The proxy's points are read in the
 frame that transform places the sphere into, never in the sphere's own.

 box3d answers yes while the gap is under a tenth of a linear slop, so surfaces half a millimetre
 apart at unit length scale still count as touching.

 @return "whether the two touch, or INVALID_DEFINITION when the proxy carries no points"
*>
fn bool? Sphere.overlaps(&self, Transform transform, ShapeProxy proxy) {
    check_shape_proxy(proxy)!;
    return sphere_overlap_raw(self, transform, &proxy);
}

<*
 Whether the proxy touches this capsule placed by transform. The points are read in the frame that
 transform places the capsule into, and a gap under a tenth of a linear slop still counts as
 touching.

 @return "whether the two touch, or INVALID_DEFINITION when the proxy carries no points"
*>
fn bool? Capsule.overlaps(&self, Transform transform, ShapeProxy proxy) {
    check_shape_proxy(proxy)!;
    return capsule_overlap_raw(self, transform, &proxy);
}

<*
 Whether the proxy touches this hull placed by transform. The points are read in the frame that
 transform places the hull into, and a gap under a tenth of a linear slop still counts as
 touching.

 @return "whether the two touch, or INVALID_DEFINITION when the proxy carries no points"
*>
fn bool? HullData.overlaps(&self, Transform transform, ShapeProxy proxy) {
    check_shape_proxy(proxy)!;
    return hull_overlap_raw(self, transform, &proxy);
}

<*
 Whether the proxy touches this compound placed by transform.

 The placement reaches the per-child test but not the search that picks which children to run it
 on: box3d looks the proxy's own box up in a tree built in the compound's frame, so a transform
 carrying the compound out from under that box answers no while the two plainly touch.

 @return "whether the two touch, or INVALID_DEFINITION when the proxy carries no points"
*>
fn bool? CompoundData.overlaps(&self, Transform transform, ShapeProxy proxy) {
    check_shape_proxy(proxy)!;
    return compound_overlap_raw(self, transform, &proxy);
}

<*
 Whether the proxy touches a triangle of this mesh placed by transform, scaled by the mesh's own
 scale, with a gap under a tenth of a linear slop still counting as touching. Winding does not
 enter it, so a proxy under the mesh answers the same as one above it.

 @return "whether the two touch, or INVALID_DEFINITION when the proxy carries no points"
*>
fn bool? Mesh.overlaps(&self, Transform transform, ShapeProxy proxy) {
    check_shape_proxy(proxy)!;
    return mesh_overlap_raw(self, transform, &proxy);
}

<*
 Whether the proxy touches a triangle of this height field placed by transform, with a gap under a
 tenth of a linear slop still counting as touching. A hole cell holds no triangles, so a proxy
 over one answers no.

 @return "whether the two touch, or INVALID_DEFINITION when the proxy carries no points"
*>
fn bool? HeightFieldData.overlaps(&self, Transform transform, ShapeProxy proxy) {
    check_shape_proxy(proxy)!;
    return height_field_overlap_raw(self, transform, &proxy);
}
