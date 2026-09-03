module b3_test;

import b3;
import std::math;

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
    defer b3::destroy_hull(from_points);

    b3::HullData*? cylinder = b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 0.0,
      sides: 8,
    );
    defer if (try cylinder) b3::destroy_hull(cylinder);

    b3::HullData*? cone = b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 8,
    );
    defer if (try cone) b3::destroy_hull(cone);

    b3::HullData*? rock = b3::create_rock(1.0);
    defer if (try rock) b3::destroy_hull(rock);

    b3::HullData*? clone = b3::clone_hull(from_points);
    defer if (try clone) b3::destroy_hull(clone);

    b3::HullData*? transformed_clone = b3::clone_and_transform_hull(from_points, a_placement_five_metres_along_x(),
      (b3::Vec3){ 2.0, 3.0, 4.0 },);
    defer if (try transformed_clone) b3::destroy_hull(transformed_clone);

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
    defer b3::destroy_hull(hull);

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
    defer b3::destroy_hull(original);
    int with_original = b3::byte_count();

    b3::HullData* clone = b3::clone_hull(original)!!;
    defer b3::destroy_hull(clone);

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
    defer b3::destroy_hull(original);

    b3::HullData* clone = b3::clone_and_transform_hull(original, a_placement_five_metres_along_x(),
      (b3::Vec3){ 2.0, 3.0, 4.0 },)!!;
    defer b3::destroy_hull(clone);

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
// way. Mirroring winds every face inwards, so the volume assertion below is what shows box3d
// reversed the winding rather than refusing the hull.
fn MirroredCloneReadings the_readings_of_an_upright_and_a_mirrored_cone() {
    b3::HullData* cone = b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 8,
    )!!;
    defer b3::destroy_hull(cone);

    b3::HullData* upright = b3::clone_and_transform_hull(cone, a_placement_five_metres_along_x(),
      (b3::Vec3){ 1.0, 1.0, 1.0 },)!!;
    defer b3::destroy_hull(upright);

    b3::HullData* mirrored = b3::clone_and_transform_hull(cone, a_placement_five_metres_along_x(),
      (b3::Vec3){ 1.0, -1.0, 1.0 },)!!;
    defer b3::destroy_hull(mirrored);

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
    defer b3::destroy_hull(lowered);

    b3::HullData* raised = b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 3.0,
      sides: 8,
    )!!;
    defer b3::destroy_hull(raised);

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
    defer b3::destroy_hull(hull);

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
    defer if (try hull) b3::destroy_hull(hull);

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
    defer if (try upside_down) b3::destroy_hull(upside_down);
    assert(@the_vertex_count_of(upside_down) == 16,
        "a negative height builds the cylinder downwards from its offset");

    b3::HullData*? apex = b3::create_cone(
      height: 2.0,
      radius1: 0.0,
      radius2: 0.5,
      slices: 8,
    );
    defer if (try apex) b3::destroy_hull(apex);
    assert(@the_vertex_count_of(apex) == 9, "a base radius of zero draws the base ring into one apex");

    b3::HullData*? three_slices = b3::create_cone(
      height: 2.0,
      radius1: 1.0,
      radius2: 0.5,
      slices: 3,
    );
    defer if (try three_slices) b3::destroy_hull(three_slices);
    assert(@the_vertex_count_of(three_slices) == 6, "and three slices build where box3d asserts four");

    b3::HullData*? negative_rock = b3::create_rock(-1.0);
    defer if (try negative_rock) b3::destroy_hull(negative_rock);
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
    defer if (try hull) b3::destroy_hull(hull);

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
    assert(two.refusal == b3::DEGENERATE_GEOMETRY, "the guard refuses two sides before box3d sees them");
    assert(two.after == two.baseline);

    // Past the guard, box3d refuses on its own edge ceiling rather than on the count, which is the
    // path the guard does not cover and the only one that reaches the library.
    HullRefusal past_the_ceiling = @the_refusal_of(b3::create_cylinder(
      height: 2.0,
      radius: 0.5,
      y_offset: 0.0,
      sides: 43,
    ));
    assert(past_the_ceiling.refusal == b3::DEGENERATE_GEOMETRY, "and box3d refuses the ring above its edge ceiling");
    assert(past_the_ceiling.after == past_the_ceiling.baseline, "giving back every byte it took to find out");

    HullRefusal cone_past_the_ceiling = @the_refusal_of(b3::create_cone(
      height: 2.0,
      radius1: 0.5,
      radius2: 0.25,
      slices: 43,
    ));
    assert(cone_past_the_ceiling.refusal == b3::DEGENERATE_GEOMETRY, "a cone the same way");
    assert(cone_past_the_ceiling.after == cone_past_the_ceiling.baseline);

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

// A count below one is not a refusal box3d makes cheaply: it squares into a large positive vertex
// count, charges the allocation and never gives it back, so the guard stands between the suite and
// a permanently shifted baseline rather than merely between a caller and a null.
fn void a_canned_mesh_of_fewer_than_one_cell_is_refused_before_box3d_allocates() @test {
    int baseline = b3::byte_count();
    fault none;

    fault grid = none;
    if (catch err = b3::create_grid_mesh(
      x_count: -1000,
      z_count: -1000,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )) grid = err;

    fault wave = none;
    if (catch err = b3::create_wave_mesh(
      x_count: -1000,
      z_count: -1000,
      cell_width: 1.0,
      amplitude: 0.5,
      row_frequency: 1.0,
      column_frequency: 1.0,
    )) wave = err;

    assert(grid == b3::DEGENERATE_GEOMETRY, "a grid of fewer than one cell names no mesh");
    assert(wave == b3::DEGENERATE_GEOMETRY, "and neither does a wave over one");
    assert(b3::byte_count() == baseline, "and neither of them moved the byte count doing it");
}

struct ScaledBoxReadings {
    b3::ScaledBox doubled;
    b3::ScaledBox mirrored;
}

// Three different half widths and a placement off the origin, so a scale applied to the wrong
// component or dropped from the position is visible rather than hidden by symmetry.
fn ScaledBoxReadings the_box_a_post_scale_resolves_to() {
    b3::Vec3 half_widths = { 1.0, 2.0, 3.0 };
    b3::Transform placed = a_placement_five_metres_along_x();

    b3::ScaledBox doubled = b3::scale_box(
      half_widths: half_widths,
      transform: placed,
      post_scale: (b3::Vec3){ 2.0, 2.0, 2.0 },
      min_half_width: 0.01,
    );
    b3::ScaledBox mirrored = b3::scale_box(
      half_widths: half_widths,
      transform: placed,
      post_scale: (b3::Vec3){ -2.0, 3.0, 1.0 },
      min_half_width: 0.01,
    );

    return { .doubled = doubled, .mirrored = mirrored };
}

fn void a_post_scale_lands_in_the_half_widths_and_in_the_position() @test {
    ScaledBoxReadings box = the_box_a_post_scale_resolves_to();

    assert(1.99 < box.doubled.half_widths.x && box.doubled.half_widths.x < 2.01,
        "a scale of two doubles all three half widths");
    assert(3.99 < box.doubled.half_widths.y && box.doubled.half_widths.y < 4.01);
    assert(5.99 < box.doubled.half_widths.z && box.doubled.half_widths.z < 6.01);
    assert(9.99 < box.doubled.transform.p.x && box.doubled.transform.p.x < 10.01,
        "and carries the position out with them rather than leaving the box where it stood");

    assert(1.99 < box.mirrored.half_widths.x && box.mirrored.half_widths.x < 2.01,
        "a negative component widens by its magnitude, so no half width comes back negative");
    assert(5.99 < box.mirrored.half_widths.y && box.mirrored.half_widths.y < 6.01);
    assert(2.99 < box.mirrored.half_widths.z && box.mirrored.half_widths.z < 3.01);
    assert(-10.01 < box.mirrored.transform.p.x && box.mirrored.transform.p.x < -9.99,
        "and the reflection lands in the position instead");
}

struct FlooredBoxReadings {
    b3::ScaledBox collapsed;
    b3::ScaledBox left_alone;
    b3::ScaledBox widened;
}

// The unscaled pair is the control: without it a minimum that fires on every call would look
// exactly like one that only catches a box the scale flattened.
fn FlooredBoxReadings the_box_a_minimum_half_width_holds_open() {
    b3::Vec3 half_widths = { 1.0, 2.0, 3.0 };
    b3::Transform placed = a_placement_five_metres_along_x();
    b3::Vec3 unscaled = { 1.0, 1.0, 1.0 };

    b3::ScaledBox collapsed = b3::scale_box(
      half_widths: half_widths,
      transform: placed,
      post_scale: (b3::Vec3){ 0.0, 0.0, 0.0 },
      min_half_width: 0.01,
    );
    b3::ScaledBox left_alone = b3::scale_box(
      half_widths: half_widths,
      transform: placed,
      post_scale: unscaled,
      min_half_width: 0.01,
    );
    b3::ScaledBox widened = b3::scale_box(
      half_widths: half_widths,
      transform: placed,
      post_scale: unscaled,
      min_half_width: 10.0,
    );

    return { .collapsed = collapsed, .left_alone = left_alone, .widened = widened };
}

fn void a_minimum_half_width_is_a_floor_under_every_axis_rather_than_a_refusal() @test {
    FlooredBoxReadings box = the_box_a_minimum_half_width_holds_open();

    assert(0.009 < box.collapsed.half_widths.x && box.collapsed.half_widths.x < 0.011,
        "a scale of zero flattens the box onto the minimum and is accepted");
    assert(0.009 < box.collapsed.half_widths.y && box.collapsed.half_widths.y < 0.011);
    assert(0.009 < box.collapsed.half_widths.z && box.collapsed.half_widths.z < 0.011);

    assert(0.99 < box.left_alone.half_widths.x && box.left_alone.half_widths.x < 1.01,
        "a minimum under all three half widths leaves every one of them where it was");
    assert(1.99 < box.left_alone.half_widths.y && box.left_alone.half_widths.y < 2.01);
    assert(2.99 < box.left_alone.half_widths.z && box.left_alone.half_widths.z < 3.01);

    assert(9.99 < box.widened.half_widths.x && box.widened.half_widths.x < 10.01,
        "and one above them widens a box the scale never narrowed");
    assert(9.99 < box.widened.half_widths.y && box.widened.half_widths.y < 10.01);
    assert(9.99 < box.widened.half_widths.z && box.widened.half_widths.z < 10.01);
}

struct TurnedBoxReadings {
    b3::ScaledBox upright;
    b3::ScaledBox quarter_turn;
    b3::ScaledBox eighth_turn;
    b3::ScaledBox eighth_turn_scaled_evenly;
}

// The upright reading is what says which half width a turned box hands to the stretching
// component. An eighth turn is what separates a box3d that refits the box from one that carries
// a single corner through: at a quarter turn the box lands square to the axes and the two agree.
fn TurnedBoxReadings the_box_a_post_scale_resolves_to_under_a_rotation() {
    b3::Vec3 half_widths = { 1.0, 2.0, 3.0 };
    b3::Vec3 forward = { 0.0, 0.0, 1.0 };
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Vec3 along_x = { 2.0, 1.0, 1.0 };

    b3::Transform quarter = a_turn_about(forward, math::PI / 2.0, origin);
    b3::Transform eighth = a_turn_about(forward, math::PI / 4.0, origin);

    b3::ScaledBox upright = b3::scale_box(
      half_widths: half_widths,
      transform: an_untouched_placement(),
      post_scale: along_x,
      min_half_width: 0.01,
    );
    b3::ScaledBox quarter_turn = b3::scale_box(
      half_widths: half_widths,
      transform: quarter,
      post_scale: along_x,
      min_half_width: 0.01,
    );
    b3::ScaledBox eighth_turn = b3::scale_box(
      half_widths: half_widths,
      transform: eighth,
      post_scale: along_x,
      min_half_width: 0.01,
    );
    b3::ScaledBox eighth_turn_scaled_evenly = b3::scale_box(
      half_widths: half_widths,
      transform: eighth,
      post_scale: (b3::Vec3){ 2.0, 2.0, 2.0 },
      min_half_width: 0.01,
    );

    return {
      .upright = upright,
      .quarter_turn = quarter_turn,
      .eighth_turn = eighth_turn,
      .eighth_turn_scaled_evenly = eighth_turn_scaled_evenly,
    };
}

fn void a_rotation_decides_which_half_width_a_post_scale_stretches() @test {
    TurnedBoxReadings box = the_box_a_post_scale_resolves_to_under_a_rotation();

    assert(1.99 < box.upright.half_widths.x && box.upright.half_widths.x < 2.01,
        "upright, a scale of two along x doubles the box's own x");
    assert(1.99 < box.upright.half_widths.y && box.upright.half_widths.y < 2.01);
    assert(2.99 < box.upright.half_widths.z && box.upright.half_widths.z < 3.01);

    assert(3.99 < box.quarter_turn.half_widths.y && box.quarter_turn.half_widths.y < 4.01,
        "a quarter turn lays the box's y along world x, so that is the half width the same scale doubles");
    assert(0.99 < box.quarter_turn.half_widths.x && box.quarter_turn.half_widths.x < 1.01);
    assert(2.99 < box.quarter_turn.half_widths.z && box.quarter_turn.half_widths.z < 3.01,
        "leaving the axis the turn does not move alone");
}

fn void a_rotation_with_an_uneven_scale_answers_narrower_than_the_box_it_was_given() @test {
    TurnedBoxReadings box = the_box_a_post_scale_resolves_to_under_a_rotation();

    assert(0.49 < box.eighth_turn.half_widths.x && box.eighth_turn.half_widths.x < 0.51,
        "an eighth turn shears the box, and box3d carries one corner through rather than refitting");
    assert(2.49 < box.eighth_turn.half_widths.y && box.eighth_turn.half_widths.y < 2.51);
    assert(box.eighth_turn.half_widths.x < 1.0,
        "so the answer is approximate, and here narrower on x than the box it started from");

    assert(1.99 < box.eighth_turn_scaled_evenly.half_widths.x && box.eighth_turn_scaled_evenly.half_widths.x < 2.01,
        "a scale that is even across the three axes is exact under the same turn");
    assert(3.99 < box.eighth_turn_scaled_evenly.half_widths.y && box.eighth_turn_scaled_evenly.half_widths.y < 4.01);
    assert(5.99 < box.eighth_turn_scaled_evenly.half_widths.z && box.eighth_turn_scaled_evenly.half_widths.z < 6.01);
}

struct MirroredTurnReadings {
    b3::ScaledBox turned;
    b3::ScaledBox turned_and_mirrored;
}

// A rotation is what shows that a negative component rebuilds the quaternion: with the box
// upright the reflection has nothing to turn and the rotation comes back untouched.
fn MirroredTurnReadings the_box_a_mirroring_scale_resolves_to_under_a_rotation() {
    b3::Vec3 half_widths = { 1.0, 2.0, 3.0 };
    b3::Vec3 forward = { 0.0, 0.0, 1.0 };
    b3::Transform quarter = a_turn_about(forward, math::PI / 2.0, (b3::Vec3){ 5.0, 0.0, 0.0 });

    b3::ScaledBox turned = b3::scale_box(
      half_widths: half_widths,
      transform: quarter,
      post_scale: (b3::Vec3){ 2.0, 3.0, 1.0 },
      min_half_width: 0.01,
    );
    b3::ScaledBox turned_and_mirrored = b3::scale_box(
      half_widths: half_widths,
      transform: quarter,
      post_scale: (b3::Vec3){ -2.0, 3.0, 1.0 },
      min_half_width: 0.01,
    );

    return { .turned = turned, .turned_and_mirrored = turned_and_mirrored };
}

fn void a_mirroring_scale_turns_the_box_the_other_way_instead_of_flipping_a_half_width() @test {
    MirroredTurnReadings box = the_box_a_mirroring_scale_resolves_to_under_a_rotation();

    assert(0.706 < box.turned.transform.q.k && box.turned.transform.q.k < 0.708,
        "a positive scale leaves the quarter turn exactly as it found it");
    assert(9.99 < box.turned.transform.p.x && box.turned.transform.p.x < 10.01);

    assert(-0.708 < box.turned_and_mirrored.transform.q.k && box.turned_and_mirrored.transform.q.k < -0.706,
        "and mirroring one axis turns the box the other way rather than reflecting it");
    assert(0.706 < box.turned_and_mirrored.transform.q.l && box.turned_and_mirrored.transform.q.l < 0.708);
    assert(-10.01 < box.turned_and_mirrored.transform.p.x && box.turned_and_mirrored.transform.p.x < -9.99);

    assert(2.99 < box.turned_and_mirrored.half_widths.x && box.turned_and_mirrored.half_widths.x < 3.01,
        "leaving the same three half widths the unmirrored scale gives");
    assert(3.99 < box.turned_and_mirrored.half_widths.y && box.turned_and_mirrored.half_widths.y < 4.01);
    assert(2.99 < box.turned_and_mirrored.half_widths.z && box.turned_and_mirrored.half_widths.z < 3.01);
}
