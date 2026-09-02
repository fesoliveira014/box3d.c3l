module b3;

<*
 Builds the convex hull of a point set. The caller owns it and must destroy() it — see HullData.

 box3d silently clamps max_vertex_count to between 4 and MAX_HULL_VERTICES, and refuses both a
 point set spanning no volume and a hull needing more vertices, faces or half edges than its own
 8-bit indices reach; the null pointer it returns does not say which. Every refusal is
 allocation-free, so reaching one leaves byte_count() where it was. The null guard restores a
 check box3d does not make: it reads the first point before testing anything but the count.

 @param points : "the points to enclose; fewer than four never span a volume"
 @param max_vertex_count : "how many vertices the hull may keep"
 @return "the hull, DEGENERATE_GEOMETRY when box3d will not close these points into one, or INVALID_DEFINITION when four or more points are named through a null pointer"
*>
fn HullData*? create_hull(Vec3[] points, int max_vertex_count) {
    if (points.len < 4) return DEGENERATE_GEOMETRY~;
    if (points.ptr == null) return INVALID_DEFINITION~;
    return checked_geometry(create_hull_raw(
      points: points.ptr,
      point_count: (int)points.len,
      max_vertex_count: max_vertex_count,
    ), DEGENERATE_GEOMETRY);
}

<*
 Builds a tessellated cylinder on the y axis, owned by the caller — see HullData.

 box3d asserts a positive height and radius and at most 32 sides, compiling all three out: a
 negative height or radius builds, and the sides run past 32 until its own edge ceiling refuses at
 43. Neither end of the count is box3d's to refuse — below three it charges an allocation it never
 returns, far above it writes through one that failed — so both are bounded here, and no side
 count this refuses is one box3d would have built.

 @param y_offset : "where one ring sits on the y axis; the other sits height above it"
 @return "the hull, or DEGENERATE_GEOMETRY when the side count is outside 3 to MAX_HULL_RING_STEPS or box3d refuses the points it describes"
*>
fn HullData*? create_cylinder(float height, float radius, float y_offset, int sides) {
    if (sides < 3 || sides > MAX_HULL_RING_STEPS) return DEGENERATE_GEOMETRY~;
    return checked_geometry(create_cylinder_raw(
      height: height,
      radius: radius,
      y_offset: y_offset,
      sides: sides,
    ), DEGENERATE_GEOMETRY);
}

<*
 Builds a tessellated cone on the y axis, radius1 at y 0 and radius2 at y height. The caller owns
 it and must destroy() it — see HullData.

 box3d asserts a positive height, both radii, and between four and 32 slices, and compiles every
 one out here: three slices builds, and one radius of zero builds a true apex which reaches 64
 slices where a full cone stops at 42. The count is bounded at both ends for the reason
 create_cylinder gives, and no slice count this refuses is one box3d would have built.

 @return "the hull, or DEGENERATE_GEOMETRY when the slice count is outside 3 to MAX_HULL_RING_STEPS or box3d refuses the points it describes"
*>
fn HullData*? create_cone(float height, float radius1, float radius2, int slices) {
    if (slices < 3 || slices > MAX_HULL_RING_STEPS) return DEGENERATE_GEOMETRY~;
    return checked_geometry(create_cone_raw(
      height: height,
      radius1: radius1,
      radius2: radius2,
      slices: slices,
    ), DEGENERATE_GEOMETRY);
}

<*
 Builds a ten-point hull scattered over a sphere of the given radius. The caller owns it and must
 destroy() it — see HullData.

 box3d asserts nothing here at all: a negative radius builds the same hull its positive twin does,
 and only a radius small enough to draw the ten points together inside box3d's own build tolerance
 is refused.

 @return "the hull, or DEGENERATE_GEOMETRY when the radius collapses the ten points"
*>
fn HullData*? create_rock(float radius) {
    return checked_geometry(create_rock_raw(radius), DEGENERATE_GEOMETRY);
}

<*
 Copies the hull into a second allocation the caller owns and must destroy() separately. The
 original is untouched and byte_count() carries both.

 @return "the copy, or DEGENERATE_GEOMETRY when box3d returns a null pointer, which a safe build cannot reach: C3 traps the null receiver first"
*>
fn HullData*? HullData.clone(&self) {
    return checked_geometry(clone_hull_raw(self), DEGENERATE_GEOMETRY);
}

<*
 Copies the hull with the transform and scale baked into its points, so the copy's own aabb,
 volume, centre and inertia are the transformed ones. Ownership works as it does for clone().

 A scale component is pushed away from zero to a minimum magnitude keeping its sign rather than
 refused, so a zero scale yields a very small hull instead of nothing; a negative one mirrors the
 hull, and box3d reverses the face winding to keep it wound outwards.

 @return "the copy, or DEGENERATE_GEOMETRY when the scaled hull's volume underflows to nothing, which repeated shrinking reaches"
*>
fn HullData*? HullData.clone_and_transform(&self, Transform transform, Vec3 scale) {
    return checked_geometry(clone_and_transform_hull_raw(self, transform, scale), DEGENERATE_GEOMETRY);
}
