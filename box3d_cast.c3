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
