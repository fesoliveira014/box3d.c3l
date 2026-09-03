module b3_test;

import b3;
import std::math;

// What one predicate answered. An expression macro for the reason @the_reading_of is one: C3 forbids
// an optional parameter, so the call has to arrive unevaluated. A refusal reads as no here, which is
// safe only because every fixture below pairs a no with a yes.
macro bool @the_answer_to(#overlap) {
    if (try answer = #overlap) return answer;
    return false;
}

struct OverlapPair {
    bool at_the_shape;
    bool fifty_metres_out;
}

struct OverlapReadings {
    OverlapPair sphere;
    OverlapPair capsule;
    OverlapPair hull;
    OverlapPair compound;
    OverlapPair mesh;
    OverlapPair height_field;
}

fn b3::Transform an_unmoved_placement() {
    return { .p = { 0.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
}

fn b3::Transform a_placement_fifty_metres_along_x() {
    return { .p = { 50.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
}

// The points are borrowed for the length of the call, so each fixture keeps its own array alive
// across every predicate it puts them to.
fn b3::ShapeProxy a_probe_over(b3::Vec3[] points, float radius) {
    return { .points = points.ptr, .count = (int)points.len, .radius = radius };
}

fn OverlapReadings the_geometry_a_probe_is_held_against() {
    b3::Sphere unit = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer b3::destroy_mesh(grid);
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer b3::destroy_height_field(flat);

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer b3::destroy_compound(one_sphere);

    b3::Vec3[1] origin = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] over_the_grid = { { 1.5, 0.0, 1.5 } };
    b3::Vec3[1] over_the_field = { { 3.0, 0.0, 3.0 } };
    b3::Vec3[1] far_out = { { 50.0, 0.0, 0.0 } };

    b3::Transform here = an_unmoved_placement();
    b3::ShapeProxy at_the_origin = a_probe_over(origin[..], 0.25);
    b3::ShapeProxy on_the_grid = a_probe_over(over_the_grid[..], 0.25);
    b3::ShapeProxy on_the_field = a_probe_over(over_the_field[..], 0.25);
    b3::ShapeProxy away = a_probe_over(far_out[..], 0.25);

    return {
      .sphere = {
        .at_the_shape = @the_answer_to(b3::overlap_sphere(&unit, here, at_the_origin)),
        .fifty_metres_out = @the_answer_to(b3::overlap_sphere(&unit, here, away)),
      },
      .capsule = {
        .at_the_shape = @the_answer_to(b3::overlap_capsule(&rod, here, at_the_origin)),
        .fifty_metres_out = @the_answer_to(b3::overlap_capsule(&rod, here, away)),
      },
      .hull = {
        .at_the_shape = @the_answer_to(b3::overlap_hull(&cube.base, here, at_the_origin)),
        .fifty_metres_out = @the_answer_to(b3::overlap_hull(&cube.base, here, away)),
      },
      .compound = {
        .at_the_shape = @the_answer_to(b3::overlap_compound(one_sphere, here, at_the_origin)),
        .fifty_metres_out = @the_answer_to(b3::overlap_compound(one_sphere, here, away)),
      },
      .mesh = {
        .at_the_shape = @the_answer_to(b3::overlap_mesh(&mesh, here, on_the_grid)),
        .fifty_metres_out = @the_answer_to(b3::overlap_mesh(&mesh, here, away)),
      },
      .height_field = {
        .at_the_shape = @the_answer_to(b3::overlap_height_field(flat, here, on_the_field)),
        .fifty_metres_out = @the_answer_to(b3::overlap_height_field(flat, here, away)),
      },
    };
}

fn void a_probe_held_against_geometry_answers_yes_and_one_held_away_answers_no() @test {
    OverlapReadings held = the_geometry_a_probe_is_held_against();

    assert(held.sphere.at_the_shape && !held.sphere.fifty_metres_out, "a sphere answers for a probe at it alone");
    assert(held.capsule.at_the_shape && !held.capsule.fifty_metres_out, "so does a capsule");
    assert(held.hull.at_the_shape && !held.hull.fifty_metres_out, "so does a hull");
    assert(held.compound.at_the_shape && !held.compound.fifty_metres_out, "so does a compound");
    assert(held.mesh.at_the_shape && !held.mesh.fifty_metres_out, "so does a mesh");
    assert(held.height_field.at_the_shape && !held.height_field.fifty_metres_out, "and so does a height field");
}

struct PlacedOverlaps {
    bool sphere;
    bool capsule;
    bool hull;
    bool mesh;
    bool height_field;
}

// Every reading here moves the shape fifty metres and moves the probe the same fifty, so a predicate
// that reads its placement answers exactly as it did at the origin.
fn PlacedOverlaps the_geometry_a_placement_carries_out_from_under_a_probe() {
    b3::Sphere unit = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer b3::destroy_mesh(grid);
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer b3::destroy_height_field(flat);

    b3::Vec3[1] moved = { { 50.0, 0.0, 0.0 } };
    b3::Vec3[1] moved_over_the_grid = { { 51.5, 0.0, 1.5 } };
    b3::Vec3[1] moved_over_the_field = { { 53.0, 0.0, 3.0 } };

    b3::Transform there = a_placement_fifty_metres_along_x();
    b3::ShapeProxy following = a_probe_over(moved[..], 0.25);

    return {
      .sphere = @the_answer_to(b3::overlap_sphere(&unit, there, following)),
      .capsule = @the_answer_to(b3::overlap_capsule(&rod, there, following)),
      .hull = @the_answer_to(b3::overlap_hull(&cube.base, there, following)),
      .mesh = @the_answer_to(b3::overlap_mesh(&mesh, there, a_probe_over(moved_over_the_grid[..], 0.25))),
      .height_field = @the_answer_to(b3::overlap_height_field(flat, there, a_probe_over(moved_over_the_field[..], 0.25))),
    };
}

fn void a_placement_carries_five_of_the_six_kinds_under_the_probe_with_it() @test {
    PlacedOverlaps placed = the_geometry_a_placement_carries_out_from_under_a_probe();

    assert(placed.sphere && placed.capsule && placed.hull, "a placement carries the three convex kinds with it");
    assert(placed.mesh && placed.height_field, "and the two surfaces as well");
}

struct CompoundPlacements {
    bool square_to_the_probe;
    bool turned_away_from_the_probe;
    bool carried_away_from_the_probe;
    bool left_where_it_was_built;
}

// The child sits a metre out along x so that a quarter turn takes it off the probe while its box,
// which is the box the search reads, still covers it. That separates what a placement does here:
// the turn reaches the child, and the fifty metres answers no without ever having reached it.
fn CompoundPlacements the_placements_a_compound_is_given() {
    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 1.0, 0.0, 0.0 }, .radius = 0.2 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer b3::destroy_compound(one_sphere);

    b3::Vec3[1] at_the_child = { { 1.0, 0.0, 0.0 } };
    b3::Vec3[1] moved = { { 51.0, 0.0, 0.0 } };
    float half = (float)math::sqrt(0.5);

    b3::Transform here = an_unmoved_placement();
    b3::Transform quarter_turn = { .p = { 0.0, 0.0, 0.0 }, .q = { 0.0, 0.0, half, half } };
    b3::Transform there = a_placement_fifty_metres_along_x();
    b3::ShapeProxy probe = a_probe_over(at_the_child[..], 0.05);

    return {
      .square_to_the_probe = @the_answer_to(b3::overlap_compound(one_sphere, here, probe)),
      .turned_away_from_the_probe = @the_answer_to(b3::overlap_compound(one_sphere, quarter_turn, probe)),
      .carried_away_from_the_probe = @the_answer_to(b3::overlap_compound(one_sphere, there, a_probe_over(moved[..], 0.05))),
      .left_where_it_was_built = @the_answer_to(b3::overlap_compound(one_sphere, here, probe)),
    };
}

fn void a_compound_loses_a_probe_its_placement_carries_it_away_from() @test {
    CompoundPlacements placed = the_placements_a_compound_is_given();

    assert(placed.square_to_the_probe, "a compound answers yes for a probe held at its child");
    assert(!placed.turned_away_from_the_probe,
        "and no once a quarter turn takes that child off the probe, so the placement does reach the child");

    assert(placed.left_where_it_was_built, "the same compound still answers where it was built");
    assert(!placed.carried_away_from_the_probe,
        "yet not for a probe fifty metres out that follows it, the search reading the probe in the wrong frame");
}

struct HoleReadings {
    bool over_a_hole;
    bool beside_a_hole;
}

fn HoleReadings the_height_field_a_probe_is_held_over() {
    b3::HeightFieldData* holed = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: true,
    )!!;
    defer b3::destroy_height_field(holed);

    b3::Vec3[1] hole = { { 2.5, 0.0, 2.5 } };
    b3::Vec3[1] kept = { { 0.5, 0.0, 0.5 } };
    b3::Transform here = an_unmoved_placement();

    return {
      .over_a_hole = @the_answer_to(b3::overlap_height_field(holed, here, a_probe_over(hole[..], 0.1))),
      .beside_a_hole = @the_answer_to(b3::overlap_height_field(holed, here, a_probe_over(kept[..], 0.1))),
    };
}

fn void a_probe_over_a_punched_out_cell_touches_nothing() @test {
    HoleReadings held = the_height_field_a_probe_is_held_over();

    assert(held.beside_a_hole, "a probe over a cell the field kept touches it");
    assert(!held.over_a_hole, "and the same probe over a cell punched out does not");
}

struct SlopReadings {
    bool within_the_slop;
    bool past_the_slop;
}

// box3d calls it an overlap while the gap is under a tenth of a linear slop, and the linear slop is
// defined in terms of the ambient length unit, so both points are placed by that expression rather
// than at a recorded distance.
fn SlopReadings the_gaps_a_sphere_still_calls_an_overlap() {
    b3::Sphere unit = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    float overlap_slop = 0.0005f * b3::length_units_per_meter();

    b3::Vec3[1] near = { { 0.0, 0.5f + 0.5f * overlap_slop, 0.0 } };
    b3::Vec3[1] clear = { { 0.0, 0.5f + 2.0f * overlap_slop, 0.0 } };
    b3::Transform here = an_unmoved_placement();

    return {
      .within_the_slop = @the_answer_to(b3::overlap_sphere(&unit, here, a_probe_over(near[..], 0.0))),
      .past_the_slop = @the_answer_to(b3::overlap_sphere(&unit, here, a_probe_over(clear[..], 0.0))),
    };
}

fn void two_surfaces_a_slop_apart_still_count_as_touching() @test {
    SlopReadings gaps = the_gaps_a_sphere_still_calls_an_overlap();

    assert(gaps.within_the_slop, "a point half a slop off the surface overlaps the sphere");
    assert(!gaps.past_the_slop, "and one twice a slop off it does not");
}

struct OverlapRefusals {
    fault sphere;
    fault capsule;
    fault hull;
    fault compound;
    fault mesh;
    fault height_field;
    bool an_ordinary_probe_still_answers;
}

fn OverlapRefusals the_proxies_the_predicates_refuse() {
    b3::Sphere unit = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer b3::destroy_mesh(grid);
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer b3::destroy_height_field(flat);

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer b3::destroy_compound(one_sphere);

    b3::Vec3[1] origin = { { 0.0, 0.0, 0.0 } };
    b3::Transform here = an_unmoved_placement();
    b3::ShapeProxy empty = a_probe_over(origin[:0], 0.25);

    OverlapRefusals refusals;
    if (catch err = b3::overlap_sphere(&unit, here, empty)) refusals.sphere = err;
    if (catch err = b3::overlap_capsule(&rod, here, empty)) refusals.capsule = err;
    if (catch err = b3::overlap_hull(&cube.base, here, empty)) refusals.hull = err;
    if (catch err = b3::overlap_compound(one_sphere, here, empty)) refusals.compound = err;
    if (catch err = b3::overlap_mesh(&mesh, here, empty)) refusals.mesh = err;
    if (catch err = b3::overlap_height_field(flat, here, empty)) refusals.height_field = err;

    refusals.an_ordinary_probe_still_answers = @the_answer_to(b3::overlap_sphere(&unit, here, a_probe_over(origin[..], 0.25)));
    return refusals;
}

// A proxy of no points is read as its first point by four kinds, and answered by the mesh and the
// height field from a stack buffer nothing wrote, so leaving it to box3d makes the answer depend
// on the caller's own stray memory.
fn void an_overlap_refuses_a_proxy_that_carries_no_points() @test {
    OverlapRefusals refused = the_proxies_the_predicates_refuse();

    assert(refused.sphere == b3::INVALID_DEFINITION, "a sphere will not take a proxy of no points");
    assert(refused.capsule == b3::INVALID_DEFINITION, "nor a capsule");
    assert(refused.hull == b3::INVALID_DEFINITION, "nor a hull");
    assert(refused.compound == b3::INVALID_DEFINITION, "nor a compound");
    assert(refused.mesh == b3::INVALID_DEFINITION, "nor a mesh");
    assert(refused.height_field == b3::INVALID_DEFINITION, "nor a height field");

    assert(refused.an_ordinary_probe_still_answers, "while a proxy of one point is answered");
}

struct OverlapAllocation {
    int baseline;
    int before;
    int after;
    bool every_predicate_answered;
}

fn OverlapAllocation the_byte_count_around_every_overlap() {
    int baseline = b3::byte_count();

    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer b3::destroy_mesh(grid);
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer b3::destroy_height_field(flat);

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer b3::destroy_compound(one_sphere);

    b3::Sphere unit = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::Vec3[1] origin = { { 0.0, 0.0, 0.0 } };
    b3::Vec3[1] over_the_grid = { { 1.5, 0.0, 1.5 } };
    b3::Vec3[1] over_the_field = { { 3.0, 0.0, 3.0 } };

    b3::Transform here = an_unmoved_placement();
    b3::ShapeProxy at_the_origin = a_probe_over(origin[..], 0.25);

    int before = b3::byte_count();

    bool[6] every = {
      @the_answer_to(b3::overlap_sphere(&unit, here, at_the_origin)),
      @the_answer_to(b3::overlap_capsule(&rod, here, at_the_origin)),
      @the_answer_to(b3::overlap_hull(&cube.base, here, at_the_origin)),
      @the_answer_to(b3::overlap_compound(one_sphere, here, at_the_origin)),
      @the_answer_to(b3::overlap_mesh(&mesh, here, a_probe_over(over_the_grid[..], 0.25))),
      @the_answer_to(b3::overlap_height_field(flat, here, a_probe_over(over_the_field[..], 0.25))),
    };

    int after = b3::byte_count();

    bool all_answered = true;
    foreach (answer : every) {
        all_answered = all_answered && answer;
    }

    return {
      .baseline = baseline,
      .before = before,
      .after = after,
      .every_predicate_answered = all_answered,
    };
}

fn void asking_geometry_whether_a_probe_touches_it_allocates_nothing() @test {
    OverlapAllocation around = the_byte_count_around_every_overlap();

    assert(around.every_predicate_answered, "all six must have found the probe for the count to mean anything");
    assert(around.before > around.baseline, "the geometry three of them are put to is built and live");
    assert(around.after == around.before, "and answering every one of them allocates nothing");
    assert(b3::byte_count() == around.baseline, "and that geometry gives back every byte it took");
}

fn void a_proxy_naming_no_array_is_refused() @test {
    b3::Sphere ball = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Transform here = { .p = { 0.0, 0.0, 0.0 }, .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 } };
    b3::ShapeProxy nothing = { .points = null, .count = 1, .radius = 0.1 };
    fault refusal;
    if (catch err = b3::overlap_sphere(&ball, here, nothing)) refusal = err;
    assert(refusal == b3::INVALID_DEFINITION, "a proxy naming no array is refused before box3d reads it");
}
