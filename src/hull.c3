module b3;

<*
 Builds the convex hull of a point set. The caller owns it and must destroy() it — see HullData.

 box3d silently clamps max_vertex_count to between 4 and MAX_HULL_VERTICES, and refuses both a
 point set spanning no volume and a hull needing more vertices, faces or half edges than its own
 8-bit indices reach; the null pointer it returns does not say which. Every refusal is
 allocation-free, so reaching one leaves byte_count() where it was. The null guard restores a
 check box3d does not make: it reads the first point before testing anything but the count.

 @param points : `the points to enclose; fewer than four never span a volume`
 @param max_vertex_count : `how many vertices the hull may keep`
 @return `the hull`
 @return? DEGENERATE_GEOMETRY : `box3d will not close these points into one`
 @return? INVALID_DEFINITION : `four or more points are named through a null pointer`
*>
fn HullData*? create_hull(Vec3[] points, int max_vertex_count) {
    if (points.len < 4) return DEGENERATE_GEOMETRY~;
    if (points.ptr == null) return INVALID_DEFINITION~;
    return checked_geometry(create_hull_raw(points.ptr, (int)points.len, max_vertex_count), DEGENERATE_GEOMETRY);
}

<*
 Builds a tessellated cylinder on the y axis, owned by the caller — see HullData.

 box3d asserts a positive height and radius and at most 32 sides, compiling all three out: a
 negative height or radius builds, and the sides run past 32 until its own edge ceiling refuses at
 43. Neither end of the count is box3d's to refuse — a negative one charges an allocation it never
 returns, and far above the ceiling it writes through one that failed — so both are bounded here.
 See MAX_HULL_RING_STEPS for why the upper bound costs nothing.

 @param y_offset : `where one ring sits on the y axis; the other sits height above it`
 @return `the hull`
 @return? DEGENERATE_GEOMETRY : `the side count is outside 3 to MAX_HULL_RING_STEPS, or box3d
   refuses the points it describes`
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
 slices where a full cone stops at 42. The count is bounded at both ends for the reasons
 create_cylinder and MAX_HULL_RING_STEPS give.

 @return `the hull`
 @return? DEGENERATE_GEOMETRY : `the slice count is outside 3 to MAX_HULL_RING_STEPS, or box3d
   refuses the points it describes`
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

 box3d asserts nothing here at all. A negative radius negates every lattice point, so it builds
 the point reflection of the positive one — the same count, volume and area, with mirrored bounds.
 Only a radius small enough to draw the ten points inside box3d's build tolerance is refused.

 @return `the hull`
 @return? DEGENERATE_GEOMETRY : `the radius collapses the ten points`
*>
fn HullData*? create_rock(float radius) {
    return checked_geometry(create_rock_raw(radius), DEGENERATE_GEOMETRY);
}

<*
 Copies the hull into a second allocation the caller owns and must destroy() separately. The
 original is untouched and byte_count() carries both.

 @return `the copy`
 @return? DEGENERATE_GEOMETRY : `box3d returned a null pointer, which a safe build cannot reach:
   C3 traps the null receiver first`
*>
fn HullData*? clone_hull(HullData* hull) {
    return checked_geometry(clone_hull_raw(hull), DEGENERATE_GEOMETRY);
}

<*
 Copies the hull with the transform and scale baked into its points, so the copy's own aabb,
 volume, centre and inertia are the transformed ones. Ownership works as it does for clone().

 A scale component is pushed away from zero to a minimum magnitude keeping its sign rather than
 refused, so a zero scale yields a very small hull instead of nothing; a negative one mirrors the
 hull, and box3d reverses the face winding to keep it wound outwards.

 @return `the copy`
 @return? DEGENERATE_GEOMETRY : `the scaled hull's volume underflows to nothing, which repeated
   shrinking reaches`
*>
fn HullData*? clone_and_transform_hull(HullData* hull, Transform transform, Vec3 scale) {
    return checked_geometry(clone_and_transform_hull_raw(hull, transform, scale), DEGENERATE_GEOMETRY);
}

<*
 Resolves a post scale applied after the box's placement back into half widths and a placement. A
 negative component mirrors the position and rebuilds the rotation, keeping the widths positive.

 The scale runs along the world axes, so the rotation decides which half width each component
 stretches, and box3d carries one corner through rather than refitting: under a rotation with a
 non-uniform scale the answer is approximate, and can be narrower than the box it started from.

 @param min_half_width : `a floor under every half width once the scale is in, so it widens a box
   the scale never narrowed; box3d asserts it finite and positive and compiles that out, so a
   non-positive one clamps nothing`
 @return `the half widths and the placement with the post scale resolved into them`
*>
fn ScaledBox scale_box(Vec3 half_widths, Transform transform, Vec3 post_scale, float min_half_width) {
    scale_box_raw(
      half_widths: &half_widths,
      transform: &transform,
      post_scale: post_scale,
      min_half_width: min_half_width,
    );
    return { .half_widths = half_widths, .transform = transform };
}
