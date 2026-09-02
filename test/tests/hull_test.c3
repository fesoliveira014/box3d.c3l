module b3_test;

import b3;

// The eight corners of a two by two by two box centred on the origin. Its hull is exact rather
// than approximate: eight vertices, six faces, twenty-four half edges, volume 8, and bounds of
// plus or minus one on every axis.
fn b3::Vec3[8] the_corners_of_a_two_metre_box() {
    return {
        { -1.0, -1.0, -1.0 }, { 1.0, -1.0, -1.0 }, { 1.0, 1.0, -1.0 }, { -1.0, 1.0, -1.0 },
        { -1.0, -1.0, 1.0 }, { 1.0, -1.0, 1.0 }, { 1.0, 1.0, 1.0 }, { -1.0, 1.0, 1.0 },
    };
}

fn b3::Transform a_placement_five_metres_along_x() {
    return { .p = { 5.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
}

// An expression macro rather than a function because C3 forbids an optional parameter, and the
// argument has to arrive unevaluated so a refusal reaches the body instead of propagating.
macro int @the_vertex_count_of(#hull) {
    if (try built = #hull) return built.vertex_count;
    return 0;
}

struct BuilderVertexCounts {
    int from_points;
    int cylinder;
    int cone;
    int rock;
    int clone;
    int transformed_clone;
}

// Every helper here destroys what it built before returning, and none of them asserts: C3 skips
// defer on a panicking assertion, and a stranded hull sits on byte_count() for the rest of the
// run exactly as a stranded world does.
fn BuilderVertexCounts the_hull_builders_that_produced_a_hull() {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();

    b3::HullData* from_points = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer from_points.destroy();

    b3::HullData*? cylinder = b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 0.0,
      sides: 8,
    );
    defer if (try cylinder) cylinder.destroy();

    b3::HullData*? cone = b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 8,
    );
    defer if (try cone) cone.destroy();

    b3::HullData*? rock = b3::create_rock(1.0);
    defer if (try rock) rock.destroy();

    b3::HullData*? clone = from_points.clone();
    defer if (try clone) clone.destroy();

    b3::HullData*? transformed_clone = from_points.clone_and_transform(
      a_placement_five_metres_along_x(),
      (b3::Vec3){ 2.0, 3.0, 4.0 },
    );
    defer if (try transformed_clone) transformed_clone.destroy();

    return {
        .from_points = from_points.vertex_count,
        .cylinder = @the_vertex_count_of(cylinder),
        .cone = @the_vertex_count_of(cone),
        .rock = @the_vertex_count_of(rock),
        .clone = @the_vertex_count_of(clone),
        .transformed_clone = @the_vertex_count_of(transformed_clone),
    };
}

fn void every_hull_builder_produces_a_hull() @test {
    BuilderVertexCounts built = the_hull_builders_that_produced_a_hull();

    assert(built.from_points == 8, "the eight corners of a box enclose exactly themselves");
    assert(built.cylinder == 16, "a cylinder of eight sides carries two rings of eight");
    assert(built.cone == 16, "and so does a cone of eight slices between two radii");
    assert(built.rock == 10, "a rock is ten points on a Fibonacci lattice");
    assert(built.clone == 8, "a clone carries the vertices of its original");
    assert(built.transformed_clone == 8, "and so does a clone that bakes a scale in");
}

// The byte count is read while the hull is alive and again by the caller, after the helper's
// defer has run. Taking the second reading here would need the destroy out of that defer.
fn int[2] the_byte_count_around_a_live_hull() {
    int baseline = b3::byte_count();

    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();
    b3::HullData* hull = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer hull.destroy();

    return { baseline, b3::byte_count() };
}

// The baseline is taken inside the run rather than assumed to be zero: other tests in the same
// process may still hold a world when this one runs.
fn void a_destroyed_hull_gives_back_every_byte_it_took() @test {
    int[2] around = the_byte_count_around_a_live_hull();

    assert(around[1] > around[0], "a hull box3d allocates must show up in the byte count");
    assert(b3::byte_count() == around[0], "and destroying it must give every byte back");
}

struct CloneReadings {
    int baseline;
    int with_original;
    int with_clone;
    bool distinct_pointers;
}

fn CloneReadings the_byte_count_around_a_cloned_hull() {
    int baseline = b3::byte_count();

    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();
    b3::HullData* original = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer original.destroy();
    int with_original = b3::byte_count();

    b3::HullData* clone = original.clone()!!;
    defer clone.destroy();

    return {
        .baseline = baseline,
        .with_original = with_original,
        .with_clone = b3::byte_count(),
        .distinct_pointers = original != clone,
    };
}

fn void a_clone_is_a_second_allocation_the_caller_also_owns() @test {
    CloneReadings around = the_byte_count_around_a_cloned_hull();

    assert(around.with_original > around.baseline, "the original must show up in the byte count");
    assert(around.with_clone == 2 * around.with_original - around.baseline,
        "and the clone must cost exactly what the original did");
    assert(around.distinct_pointers, "a clone is a copy, never the pointer it was made from");
    assert(b3::byte_count() == around.baseline, "destroying both must give every byte back");
}

struct TransformedCloneBounds {
    b3::Aabb original;
    b3::Aabb clone;
}

// The scale runs before the placement, so each corner is stretched about the origin and only then
// carried along x. A different scale per axis is what separates the three components.
fn TransformedCloneBounds the_bounds_around_a_stretched_and_moved_clone() {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();
    b3::HullData* original = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer original.destroy();

    b3::HullData* clone = original.clone_and_transform(
      a_placement_five_metres_along_x(),
      (b3::Vec3){ 2.0, 3.0, 4.0 },
    )!!;
    defer clone.destroy();

    return { .original = original.aabb, .clone = clone.aabb };
}

fn void a_transformed_clone_bakes_its_scale_in_and_leaves_the_original_alone() @test {
    TransformedCloneBounds bounds = the_bounds_around_a_stretched_and_moved_clone();

    assert(6.99 < bounds.clone.upper_bound.x && bounds.clone.upper_bound.x < 7.01,
        "one metre stretched to two and carried five along x");
    assert(2.99 < bounds.clone.lower_bound.x && bounds.clone.lower_bound.x < 3.01);
    assert(2.99 < bounds.clone.upper_bound.y && bounds.clone.upper_bound.y < 3.01,
        "and three and four across the axes the placement leaves alone");
    assert(-3.01 < bounds.clone.lower_bound.y && bounds.clone.lower_bound.y < -2.99);
    assert(3.99 < bounds.clone.upper_bound.z && bounds.clone.upper_bound.z < 4.01);
    assert(-4.01 < bounds.clone.lower_bound.z && bounds.clone.lower_bound.z < -3.99);

    assert(0.99 < bounds.original.upper_bound.x && bounds.original.upper_bound.x < 1.01,
        "and the hull it was cloned from keeps the reach it had");
    assert(-1.01 < bounds.original.lower_bound.z && bounds.original.lower_bound.z < -0.99);
}

struct MirroredCloneReadings {
    b3::Aabb upright;
    b3::Aabb mirrored;
    float upright_volume;
    float mirrored_volume;
}

// A cone rather than the box: a box mirrors onto itself and would report the same bounds either
// way. Mirroring also leaves every face wound inwards until box3d reverses the winding, and a
// hull wound inwards measures a negative volume, which box3d refuses outright — so the mirrored
// clone existing at all, with the volume of its upright twin, is the evidence of the rewinding.
fn MirroredCloneReadings the_readings_of_an_upright_and_a_mirrored_cone() {
    b3::HullData* cone = b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 8,
    )!!;
    defer cone.destroy();

    b3::HullData* upright = cone.clone_and_transform(
      a_placement_five_metres_along_x(),
      (b3::Vec3){ 1.0, 1.0, 1.0 },
    )!!;
    defer upright.destroy();

    b3::HullData* mirrored = cone.clone_and_transform(
      a_placement_five_metres_along_x(),
      (b3::Vec3){ 1.0, -1.0, 1.0 },
    )!!;
    defer mirrored.destroy();

    return {
        .upright = upright.aabb,
        .mirrored = mirrored.aabb,
        .upright_volume = upright.volume,
        .mirrored_volume = mirrored.volume,
    };
}

fn void a_mirrored_clone_is_turned_over_and_rewound_rather_than_inside_out() @test {
    MirroredCloneReadings cones = the_readings_of_an_upright_and_a_mirrored_cone();

    assert(-0.01 < cones.upright.lower_bound.y && cones.upright.lower_bound.y < 0.01,
        "the cone stands from zero to its height");
    assert(1.99 < cones.upright.upper_bound.y && cones.upright.upper_bound.y < 2.01);
    assert(-2.01 < cones.mirrored.lower_bound.y && cones.mirrored.lower_bound.y < -1.99,
        "and a scale of minus one on y hangs it from zero instead");
    assert(-0.01 < cones.mirrored.upper_bound.y && cones.mirrored.upper_bound.y < 0.01);
    assert(3.29 < cones.upright_volume && cones.upright_volume < 3.31,
        "both hold the same volume, which a hull wound inwards could not report");
    assert(3.29 < cones.mirrored_volume && cones.mirrored_volume < 3.31);
}

struct CylinderBounds {
    b3::Aabb lowered;
    b3::Aabb raised;
}

// Two offsets rather than one: the ring the offset places and the ring height above it move
// together, so a binding that read the two arguments in the other order would widen the cylinder
// here instead of lifting it.
fn CylinderBounds the_bounds_of_a_cylinder_at_two_offsets() {
    b3::HullData* lowered = b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: -1.0,
      sides: 8,
    )!!;
    defer lowered.destroy();

    b3::HullData* raised = b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 3.0,
      sides: 8,
    )!!;
    defer raised.destroy();

    return { .lowered = lowered.aabb, .raised = raised.aabb };
}

fn void a_cylinder_runs_from_its_offset_to_its_height_above_it() @test {
    CylinderBounds bounds = the_bounds_of_a_cylinder_at_two_offsets();

    assert(-1.01 < bounds.lowered.lower_bound.y && bounds.lowered.lower_bound.y < -0.99,
        "an offset of minus one puts one ring there");
    assert(0.99 < bounds.lowered.upper_bound.y && bounds.lowered.upper_bound.y < 1.01,
        "and the other a height of two above it");
    assert(2.99 < bounds.raised.lower_bound.y && bounds.raised.lower_bound.y < 3.01,
        "an offset of three lifts the pair rather than stretching it");
    assert(4.99 < bounds.raised.upper_bound.y && bounds.raised.upper_bound.y < 5.01);
    assert(0.49 < bounds.raised.upper_bound.x && bounds.raised.upper_bound.x < 0.51,
        "and the radius reaches across x and z, untouched by either");
    assert(-0.51 < bounds.raised.lower_bound.z && bounds.raised.lower_bound.z < -0.49);
}

struct BoxHullReadings {
    int vertex_count;
    int face_count;
    int edge_count;
    float volume;
    float surface_area;
}

fn BoxHullReadings the_readings_of_a_hull_over_a_two_metre_box() {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();
    b3::HullData* hull = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer hull.destroy();

    return {
        .vertex_count = hull.vertex_count,
        .face_count = hull.face_count,
        .edge_count = hull.edge_count,
        .volume = hull.volume,
        .surface_area = hull.surface_area,
    };
}

fn void the_hull_of_eight_box_corners_is_that_box() @test {
    BoxHullReadings box = the_readings_of_a_hull_over_a_two_metre_box();

    assert(box.vertex_count == 8, "a box keeps all eight of its corners");
    assert(box.face_count == 6);
    assert(box.edge_count == 24, "twelve edges, two half edges each");
    assert(7.99 < box.volume && box.volume < 8.01, "two cubed");
    assert(23.99 < box.surface_area && box.surface_area < 24.01, "six faces of four square metres");
}

// box3d clamps max_vertex_count into 4 to MAX_HULL_VERTICES without reporting it, so a ceiling of
// zero silently builds a tetrahedron of four of the corners rather than being refused.
fn int the_vertex_count_of_a_box_hull_capped_at(int max_vertex_count) {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();

    b3::HullData*? hull = b3::create_hull(corners[..], max_vertex_count);
    defer if (try hull) hull.destroy();

    return @the_vertex_count_of(hull);
}

fn void a_vertex_ceiling_below_four_is_corrected_rather_than_refused() @test {
    assert(the_vertex_count_of_a_box_hull_capped_at(0) == 4,
        "a ceiling of zero is raised to four and keeps four of the corners");
    assert(the_vertex_count_of_a_box_hull_capped_at(-1) == 4, "and so is a negative one");
    assert(the_vertex_count_of_a_box_hull_capped_at(8) == 8,
        "a ceiling the box fits under keeps every corner");
    assert(the_vertex_count_of_a_box_hull_capped_at(9999) == 8,
        "and one above MAX_HULL_VERTICES is lowered to it, which the box is still under");
}

// Every one of box3d's own argument checks on these four is a B3_ASSERT this release build
// compiles out, so each builds a hull where the assertion says it should not.
fn void the_arguments_box3d_asserts_and_then_accepts() @test {
    b3::HullData*? upside_down = b3::create_cylinder(
      height: -2.0,
      radius: 0.5,
      y_offset: 0.0,
      sides: 8,
    );
    defer if (try upside_down) upside_down.destroy();
    assert(@the_vertex_count_of(upside_down) == 16,
        "a negative height builds the cylinder downwards from its offset");

    b3::HullData*? apex = b3::create_cone(
      height: 2.0,
      radius1: 0.0,
      radius2: 0.5,
      slices: 8,
    );
    defer if (try apex) apex.destroy();
    assert(@the_vertex_count_of(apex) == 9, "a base radius of zero draws the base ring into one apex");

    b3::HullData*? three_slices = b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 3,
    );
    defer if (try three_slices) three_slices.destroy();
    assert(@the_vertex_count_of(three_slices) == 6, "and three slices build where box3d asserts four");

    b3::HullData*? negative_rock = b3::create_rock(-1.0);
    defer if (try negative_rock) negative_rock.destroy();
    assert(@the_vertex_count_of(negative_rock) == 10,
        "a rock of negative radius is the same ten points");
}

struct HullRefusal {
    fault refusal;
    int baseline;
    int after;
}

// The builder arrives unevaluated so the baseline is taken before it runs, which is what makes
// the byte count evidence rather than decoration.
macro HullRefusal @the_refusal_of(#builder) {
    int baseline = b3::byte_count();

    b3::HullData*? hull = #builder;
    defer if (try hull) hull.destroy();

    if (catch err = hull) return { .refusal = err, .baseline = baseline, .after = b3::byte_count() };
    return { .refusal = {}, .baseline = baseline, .after = b3::byte_count() };
}

// A hull refusal is allocation-free wherever it comes from — box3d frees its working memory on
// every path out of b3CreateHull — so the byte count is the evidence that nothing was half built.
// That does not hold of every refusal in this binding: a mesh rejected for its geometry leaks.
fn void a_point_set_too_small_to_enclose_a_volume_is_refused() @test {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();

    HullRefusal three = @the_refusal_of(b3::create_hull(corners[:3], b3::MAX_HULL_VERTICES));
    assert(three.refusal == b3::DEGENERATE_GEOMETRY, "three points span a triangle, never a volume");
    assert(three.after == three.baseline, "and box3d turns them away before allocating");

    b3::Vec3[] nothing = {};
    HullRefusal empty = @the_refusal_of(b3::create_hull(nothing, b3::MAX_HULL_VERTICES));
    assert(empty.refusal == b3::DEGENERATE_GEOMETRY,
        "and an empty slice names no points rather than naming them nowhere");
    assert(empty.after == empty.baseline);
}

fn void a_point_set_lying_in_one_plane_is_refused() @test {
    b3::Vec3[8] flat = {
        { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 1.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 },
        { 2.0, 0.0, 0.0 }, { 2.0, 0.0, 2.0 }, { 0.0, 0.0, 2.0 }, { 3.0, 0.0, 3.0 },
    };

    HullRefusal refused = @the_refusal_of(b3::create_hull(flat[..], b3::MAX_HULL_VERTICES));

    assert(refused.refusal == b3::DEGENERATE_GEOMETRY, "eight points in one plane enclose nothing");
    assert(refused.after == refused.baseline, "and box3d frees its working memory on the way out");
}

// box3d reads the first point before it tests anything but the count, so this is the difference
// between a fault and a segmentation fault.
fn b3::Vec3[] a_slice_of_eight_points_that_are_not_there() {
    b3::Vec3* nowhere = null;
    return nowhere[:8];
}

fn void a_point_count_named_through_a_null_pointer_is_refused() @test {
    HullRefusal refused = @the_refusal_of(
        b3::create_hull(a_slice_of_eight_points_that_are_not_there(), b3::MAX_HULL_VERTICES));

    assert(refused.refusal == b3::INVALID_DEFINITION, "eight points have to be somewhere");
    assert(refused.after == refused.baseline);
}

// A negative side count is the one refusal here box3d does not take cleanly: it allocates through
// a negative size and never gives the bytes back, so the count guard has to run first.
fn void a_ring_of_fewer_than_three_sides_is_refused() @test {
    HullRefusal two = @the_refusal_of(b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 0.0,
      sides: 2,
    ));
    assert(two.refusal == b3::DEGENERATE_GEOMETRY, "two sides put four points in one plane");
    assert(two.after == two.baseline);

    HullRefusal negative = @the_refusal_of(b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 0.0,
      sides: -1,
    ));
    assert(negative.refusal == b3::DEGENERATE_GEOMETRY, "and a negative count describes no ring");
    assert(negative.after == negative.baseline,
        "which box3d would have charged for and never refunded");
}

fn void a_cone_of_fewer_than_three_slices_is_refused() @test {
    HullRefusal two = @the_refusal_of(b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 2,
    ));
    assert(two.refusal == b3::DEGENERATE_GEOMETRY, "two slices put four points in one plane");
    assert(two.after == two.baseline);

    HullRefusal negative = @the_refusal_of(b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: -1,
    ));
    assert(negative.refusal == b3::DEGENERATE_GEOMETRY, "and a negative count describes no cone");
    assert(negative.after == negative.baseline,
        "which box3d would have charged for and never refunded");
}

fn void a_rock_too_small_to_separate_its_points_is_refused() @test {
    HullRefusal none = @the_refusal_of(b3::create_rock(0.0));
    assert(none.refusal == b3::DEGENERATE_GEOMETRY, "ten points at the origin are one point");
    assert(none.after == none.baseline);

    HullRefusal tiny = @the_refusal_of(b3::create_rock(0.000001));
    assert(tiny.refusal == b3::DEGENERATE_GEOMETRY,
        "and a radius inside box3d's build tolerance draws them back together");
    assert(tiny.after == tiny.baseline);
}
