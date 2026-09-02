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

 Every child is cast at as a solid sphere, capsule, hull or mesh, so a ray starting inside one
 carries that kind's own answer back out through the child's placement.

 @return "where the ray meets the nearest child, or NO_HIT when it meets none"
*>
fn CastOutput? CompoundData.ray_cast(&self, RayCastInput input) {
    CastOutput result = compound_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this mesh's triangles, scaled by the mesh's own scale, reporting a fraction of the
 ray's translation and the triangle struck in triangle_index.

 A ray meeting a triangle from behind its winding reports nothing, so each triangle answers from
 one side only.

 @return "where the ray meets a triangle, or NO_HIT when it meets none"
*>
fn CastOutput? Mesh.ray_cast(&self, RayCastInput input) {
    CastOutput result = mesh_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}

<*
 Casts a ray at this height field's triangles, reporting a fraction of the ray's translation and
 the triangle struck in triangle_index. Nothing is reported from underneath, or over a hole cell.

 box3d compresses the heights into 16 bits across the global range the definition named, so the
 surface a cast meets sits under the heights it was built from by up to one of those steps, scaled
 by the field's own y scale.

 @return "where the ray meets the field, or NO_HIT when it does not"
*>
fn CastOutput? HeightFieldData.ray_cast(&self, RayCastInput input) {
    CastOutput result = height_field_ray_cast_raw(self, &input);
    if (!result.hit) return NO_HIT~;
    return result;
}
