module b3_test;

import b3;
import std::math;

// What one cast answered, flattened so a fixture can hand back a hit and a miss together. hit is
// whether the wrapper returned a value rather than NO_HIT, which is the whole of what a caller
// sees of box3d's own hit flag, and so is the only place a wrong reading of it can surface.
struct CastReading {
    bool hit;
    float fraction;
    b3::Vec3 point;
    b3::Vec3 normal;
    int triangle_index;
    int child_index;
    int iterations;
}

// An expression macro rather than a function because C3 forbids an optional parameter, and the
// cast has to arrive unevaluated so a miss reaches the body instead of propagating.
macro CastReading @the_reading_of(#cast) {
    if (try out = #cast) {
        return {
          .hit = true,
          .fraction = out.fraction,
          .point = out.point,
          .normal = out.normal,
          .triangle_index = out.triangle_index,
          .child_index = out.child_index,
          .iterations = out.iterations,
        };
    }
    return {};
}

fn b3::RayCastInput a_ray(b3::Vec3 origin, b3::Vec3 translation, float max_fraction) {
    return { .origin = origin, .translation = translation, .max_fraction = max_fraction };
}

fn b3::Sphere a_half_metre_sphere_at_the_origin() {
    return { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
}

// Every fixture below casts from three metres up and translates six metres down, so the closed
// form of a hit on a shape reaching h above the origin is (3 - h) / 6.
fn b3::RayCastInput a_ray_dropped_from_three_metres() {
    return a_ray({ 0.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0);
}

struct SphereReadings {
    CastReading through;
    CastReading beside;
}

fn SphereReadings the_solid_sphere_a_ray_drops_onto() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();

    return {
      .through = @the_reading_of(unit.ray_cast(a_ray_dropped_from_three_metres())),
      .beside = @the_reading_of(unit.ray_cast(a_ray({ 5.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
    };
}

fn void a_ray_reports_where_it_enters_a_solid_sphere() @test {
    SphereReadings dropped = the_solid_sphere_a_ray_drops_onto();

    float expected = (3.0f - 0.5f) / 6.0f;
    assert(dropped.through.hit, "a ray dropped onto a sphere from above must find it");
    assert(math::abs(dropped.through.fraction - expected) < 0.00001,
        "and enter it where the gap to its top divides the translation");
    assert(math::abs(dropped.through.point.y - 0.5f) < 0.00001, "the hit point is the top of the sphere");
    assert(math::abs(dropped.through.point.x) < 0.00001 && math::abs(dropped.through.point.z) < 0.00001);
    assert(math::abs(dropped.through.normal.y - 1.0f) < 0.00001, "and the surface there faces straight up");
    assert(math::abs(dropped.through.normal.x) < 0.00001 && math::abs(dropped.through.normal.z) < 0.00001);

    assert(!dropped.beside.hit, "a ray five metres to the side finds nothing");
}

struct SphereFromWithin {
    CastReading from_inside;
    CastReading off_the_surface;
}

fn SphereFromWithin the_solid_sphere_a_ray_starts_within() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();

    return {
      .from_inside = @the_reading_of(unit.ray_cast(a_ray({ 0.1, 0.0, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
      .off_the_surface = @the_reading_of(unit.ray_cast(a_ray({ 0.0, 0.5, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
    };
}

fn void a_ray_starting_inside_a_solid_sphere_reports_its_own_origin_and_no_normal() @test {
    SphereFromWithin within = the_solid_sphere_a_ray_starts_within();

    assert(within.from_inside.hit, "a ray that starts inside a solid sphere is a hit");
    assert(math::abs(within.from_inside.fraction) < 0.00001, "at no distance along itself");
    assert(math::abs(within.from_inside.point.x - 0.1f) < 0.00001, "reported at the ray's own origin");
    assert(math::abs(within.from_inside.point.y) < 0.00001 && math::abs(within.from_inside.point.z) < 0.00001);
    assert(math::abs(within.from_inside.normal.x) < 0.00001 && math::abs(within.from_inside.normal.y) < 0.00001 &&
        math::abs(within.from_inside.normal.z) < 0.00001,
        "and with a zero normal, which normalising turns into a NaN");

    assert(!within.off_the_surface.hit, "a ray leaving the surface it starts on finds nothing");
}

struct SphereContrast {
    CastReading solid_from_outside;
    CastReading hollow_from_outside;
    CastReading solid_from_inside;
    CastReading hollow_from_inside;
}

// The two casts are put to the same two rays rather than to rays chosen for each, because what is
// being measured is that one answers where the other does not.
fn SphereContrast the_two_sphere_casts_put_to_the_same_rays() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::RayCastInput from_outside = a_ray_dropped_from_three_metres();
    b3::RayCastInput from_inside = a_ray({ 0.1, 0.0, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0);

    return {
      .solid_from_outside = @the_reading_of(unit.ray_cast(from_outside)),
      .hollow_from_outside = @the_reading_of(unit.ray_cast_hollow(from_outside)),
      .solid_from_inside = @the_reading_of(unit.ray_cast(from_inside)),
      .hollow_from_inside = @the_reading_of(unit.ray_cast_hollow(from_inside)),
    };
}

fn void the_two_sphere_casts_answer_opposite_situations() @test {
    SphereContrast both = the_two_sphere_casts_put_to_the_same_rays();

    assert(both.solid_from_outside.hit, "the solid cast finds a sphere a ray drops onto from above");
    assert(!both.hollow_from_outside.hit,
        "where the hollow cast, measuring a distance against the same bound of one, does not");

    float shell = (float)math::sqrt(0.5 * 0.5 - 0.1 * 0.1);
    assert(both.solid_from_inside.hit && both.hollow_from_inside.hit, "both answer a ray that starts inside");
    assert(math::abs(both.solid_from_inside.fraction) < 0.00001, "the solid one at no distance at all");
    assert(math::abs(both.hollow_from_inside.fraction - shell) < 0.00001, "the hollow one out at the shell");
    assert(math::abs(both.hollow_from_inside.normal.x - 0.2f) < 0.00001, "with a normal off the shell it met");
    assert(math::abs(both.hollow_from_inside.normal.y - shell / 0.5f) < 0.00001);
    assert(math::abs(both.hollow_from_inside.normal.z) < 0.00001);
}

struct FractionUnits {
    CastReading solid_over_six_metres;
    CastReading solid_over_one_metre;
    CastReading hollow_over_six_metres;
    CastReading hollow_over_one_metre;
}

fn FractionUnits the_same_path_walked_by_two_translations() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::RayCastInput over_six = a_ray({ 0.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 10.0);
    b3::RayCastInput over_one = a_ray({ 0.0, 3.0, 0.0 }, { 0.0, -1.0, 0.0 }, 10.0);

    return {
      .solid_over_six_metres = @the_reading_of(unit.ray_cast(over_six)),
      .solid_over_one_metre = @the_reading_of(unit.ray_cast(over_one)),
      .hollow_over_six_metres = @the_reading_of(unit.ray_cast_hollow(over_six)),
      .hollow_over_one_metre = @the_reading_of(unit.ray_cast_hollow(over_one)),
    };
}

fn void the_hollow_sphere_cast_reports_a_distance_where_the_solid_one_reports_a_fraction() @test {
    FractionUnits walked = the_same_path_walked_by_two_translations();

    assert(walked.solid_over_six_metres.hit && walked.solid_over_one_metre.hit, "both translations reach the sphere");
    assert(math::abs(walked.solid_over_six_metres.fraction - 2.5f / 6.0f) < 0.00001,
        "the solid cast divides the gap by the translation it was given");
    assert(math::abs(walked.solid_over_one_metre.fraction - 2.5f) < 0.00001,
        "so a sixth of the translation is six times the fraction");

    assert(walked.hollow_over_six_metres.hit && walked.hollow_over_one_metre.hit, "so do both for the shell");
    assert(math::abs(walked.hollow_over_six_metres.fraction - 2.5f) < 0.00001,
        "the hollow cast reports the gap itself, in the sphere's own units");
    assert(math::abs(walked.hollow_over_one_metre.fraction - 2.5f) < 0.00001,
        "and the length of the translation does not move it");
}

struct CapsuleReadings {
    CastReading through;
    CastReading beside;
    CastReading from_inside;
}

fn CapsuleReadings the_capsule_a_ray_drops_onto() {
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };

    return {
      .through = @the_reading_of(rod.ray_cast(a_ray_dropped_from_three_metres())),
      .beside = @the_reading_of(rod.ray_cast(a_ray({ 5.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
      .from_inside = @the_reading_of(rod.ray_cast(a_ray({ 0.5, 0.0, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
    };
}

fn void a_ray_reports_where_it_enters_a_capsule() @test {
    CapsuleReadings dropped = the_capsule_a_ray_drops_onto();

    float expected = (3.0f - 0.25f) / 6.0f;
    assert(dropped.through.hit, "a ray dropped across a capsule must find its side");
    assert(math::abs(dropped.through.fraction - expected) < 0.00001,
        "and enter it where the gap to the radius above its axis divides the translation");
    assert(math::abs(dropped.through.point.y - 0.25f) < 0.00001, "the hit point is a radius above the axis");
    assert(math::abs(dropped.through.normal.y - 1.0f) < 0.00001, "and the surface there faces straight up");
    assert(math::abs(dropped.through.normal.x) < 0.00001 && math::abs(dropped.through.normal.z) < 0.00001);

    assert(!dropped.beside.hit, "a ray past the end of the capsule finds nothing");

    assert(dropped.from_inside.hit, "a ray that starts inside a capsule is a hit");
    assert(math::abs(dropped.from_inside.fraction) < 0.00001, "at no distance along itself");
    assert(math::abs(dropped.from_inside.point.x - 0.5f) < 0.00001, "reported at the ray's own origin");
    assert(math::abs(dropped.from_inside.normal.x) < 0.00001 && math::abs(dropped.from_inside.normal.y) < 0.00001 &&
        math::abs(dropped.from_inside.normal.z) < 0.00001, "and with a zero normal");
}

struct HullReadings {
    CastReading through;
    CastReading beside;
    CastReading from_inside;
}

fn HullReadings the_hull_a_ray_drops_onto() {
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    return {
      .through = @the_reading_of(cube.base.ray_cast(a_ray_dropped_from_three_metres())),
      .beside = @the_reading_of(cube.base.ray_cast(a_ray({ 5.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
      .from_inside = @the_reading_of(cube.base.ray_cast(a_ray({ 0.25, 0.0, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
    };
}

fn void a_ray_reports_where_it_enters_a_hull() @test {
    HullReadings dropped = the_hull_a_ray_drops_onto();

    float expected = (3.0f - 0.5f) / 6.0f;
    assert(dropped.through.hit, "a ray dropped onto a cube from above must find its top face");
    assert(math::abs(dropped.through.fraction - expected) < 0.00001,
        "and enter it where the gap to that face divides the translation");
    assert(math::abs(dropped.through.point.y - 0.5f) < 0.00001, "the hit point is on the top face");
    assert(math::abs(dropped.through.normal.y - 1.0f) < 0.00001, "whose plane faces straight up");
    assert(math::abs(dropped.through.normal.x) < 0.00001 && math::abs(dropped.through.normal.z) < 0.00001);

    assert(!dropped.beside.hit, "a ray five metres to the side finds nothing");

    assert(dropped.from_inside.hit, "a ray that starts within every face plane is a hit");
    assert(math::abs(dropped.from_inside.fraction) < 0.00001, "at no distance along itself");
    assert(math::abs(dropped.from_inside.point.x - 0.25f) < 0.00001, "reported at the ray's own origin");
    assert(math::abs(dropped.from_inside.normal.x) < 0.00001 && math::abs(dropped.from_inside.normal.y) < 0.00001 &&
        math::abs(dropped.from_inside.normal.z) < 0.00001, "and with a zero normal");
}

struct MeshReadings {
    CastReading onto_a_triangle;
    CastReading from_underneath;
    CastReading past_the_edge;
}

// The grid is four cells square and flat on y = 0, so a ray dropped anywhere within two metres of
// the origin meets it halfway down. The destroy sits on the line after the build: a panic between
// the two would strand the mesh on the shared byte count for the rest of the run.
fn MeshReadings the_grid_mesh_a_ray_drops_onto() {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer grid.destroy();

    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    return {
      .onto_a_triangle = @the_reading_of(mesh.ray_cast(a_ray({ 1.5, 3.0, 1.5 }, { 0.0, -6.0, 0.0 }, 1.0))),
      .from_underneath = @the_reading_of(mesh.ray_cast(a_ray({ 1.5, -3.0, 1.5 }, { 0.0, 6.0, 0.0 }, 1.0))),
      .past_the_edge = @the_reading_of(mesh.ray_cast(a_ray({ 50.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
    };
}

fn void a_ray_reports_which_triangle_of_a_mesh_it_struck() @test {
    MeshReadings grid = the_grid_mesh_a_ray_drops_onto();

    assert(grid.onto_a_triangle.hit, "a ray dropped onto a flat grid must meet it");
    assert(math::abs(grid.onto_a_triangle.fraction - 0.5f) < 0.00001, "halfway down a six metre drop from three");
    assert(math::abs(grid.onto_a_triangle.point.y) < 0.00001, "on the plane the grid was built on");
    assert(math::abs(grid.onto_a_triangle.point.x - 1.5f) < 0.00001 &&
        math::abs(grid.onto_a_triangle.point.z - 1.5f) < 0.00001, "under the ray it was cast along");
    assert(math::abs(grid.onto_a_triangle.normal.y - 1.0f) < 0.00001, "off a triangle wound upwards");
    assert(grid.onto_a_triangle.triangle_index >= 0 && grid.onto_a_triangle.triangle_index < 32,
        "and names one of the two triangles each of the sixteen cells is cut into");

    assert(!grid.past_the_edge.hit, "a ray dropped fifty metres out finds no triangle");
}

fn void a_mesh_is_reached_from_one_side_only() @test {
    MeshReadings grid = the_grid_mesh_a_ray_drops_onto();

    assert(grid.onto_a_triangle.hit, "a ray meeting the wound side of a triangle reports it");
    assert(!grid.from_underneath.hit, "and the same ray reversed underneath it reports nothing");
}

struct HeightFieldReadings {
    CastReading onto_the_surface;
    CastReading from_underneath;
    CastReading over_a_hole;
    CastReading beside_a_hole;
}

// Two fields, because the holes a field is built with are the only way to reach the skipped-cell
// path, and a field without them is what the surface reading needs.
fn HeightFieldReadings the_height_fields_a_ray_drops_onto() {
    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer flat.destroy();

    b3::HeightFieldData* holed = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: true,
    )!!;
    defer holed.destroy();

    return {
      .onto_the_surface = @the_reading_of(flat.ray_cast(a_ray({ 3.0, 3.0, 3.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
      .from_underneath = @the_reading_of(flat.ray_cast(a_ray({ 3.0, -3.0, 3.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
      .over_a_hole = @the_reading_of(holed.ray_cast(a_ray({ 2.5, 3.0, 2.5 }, { 0.0, -6.0, 0.0 }, 1.0))),
      .beside_a_hole = @the_reading_of(holed.ray_cast(a_ray({ 0.5, 3.0, 0.5 }, { 0.0, -6.0, 0.0 }, 1.0))),
    };
}

// The grid builder declares a global height range of plus or minus 256 and box3d stores heights as
// sixteen-bit steps across it, truncating, so the surface sits under the plane by less than one
// step. The band is closed at both ends: a surface stored above the heights given would fail it.
fn void a_ray_meets_a_height_field_just_under_the_heights_it_was_built_from() @test {
    HeightFieldReadings field = the_height_fields_a_ray_drops_onto();

    float one_step_of_the_compressed_range = 512.0f / 65535.0f;
    assert(field.onto_the_surface.hit, "a ray dropped onto a flat field must meet it");
    assert(field.onto_the_surface.point.y <= 0.0f && field.onto_the_surface.point.y > -one_step_of_the_compressed_range,
        "under the plane it was built on by less than one step of its compressed range");
    assert(field.onto_the_surface.fraction >= 0.5f && field.onto_the_surface.fraction < 0.5f + one_step_of_the_compressed_range / 6.0f,
        "and the fraction is the same distance past the halfway point of the drop");
    assert(math::abs(field.onto_the_surface.point.x - 3.0f) < 0.00001 &&
        math::abs(field.onto_the_surface.point.z - 3.0f) < 0.00001, "under the ray it was cast along");
    assert(math::abs(field.onto_the_surface.normal.y - 1.0f) < 0.00001, "off a surface facing up");
}

fn void a_height_field_reports_nothing_from_underneath_or_over_a_hole() @test {
    HeightFieldReadings field = the_height_fields_a_ray_drops_onto();

    assert(!field.from_underneath.hit, "a ray arriving from below the surface finds nothing");
    assert(field.beside_a_hole.hit, "a ray over a cell the field kept meets it");
    assert(!field.over_a_hole.hit, "and the same ray over a cell punched out does not");
}

struct CompoundReadings {
    CastReading from_above;
    CastReading from_below;
    CastReading beside;
}

// Two spheres stacked a metre and a half apart, so the child a ray reaches first depends on which
// end it comes from and a binding reporting either the first or the last child would fail one.
fn CompoundReadings the_stacked_compound_a_ray_passes_through() {
    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
      {
        .sphere = { .center = { 0.0, 1.5, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
    };

    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;

    b3::CompoundData* tower = b3::create_compound(&def)!!;
    defer tower.destroy();

    return {
      .from_above = @the_reading_of(tower.ray_cast(a_ray_dropped_from_three_metres())),
      .from_below = @the_reading_of(tower.ray_cast(a_ray({ 0.0, -3.0, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
      .beside = @the_reading_of(tower.ray_cast(a_ray({ 5.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
    };
}

fn void a_ray_reports_the_nearest_child_of_a_compound() @test {
    CompoundReadings tower = the_stacked_compound_a_ray_passes_through();

    assert(tower.from_above.hit, "a ray dropped down the stack meets it");
    assert(tower.from_above.child_index == 1, "at the upper sphere, which is the second child");
    assert(math::abs(tower.from_above.fraction - (3.0f - 2.0f) / 6.0f) < 0.00001,
        "where the gap to the top of that sphere divides the translation");
    assert(math::abs(tower.from_above.point.y - 2.0f) < 0.00001, "and the hit point is the top of it");

    assert(tower.from_below.hit, "the same ray reversed meets the stack too");
    assert(tower.from_below.child_index == 0, "at the lower sphere, which is the first child");
    assert(math::abs(tower.from_below.fraction - (3.0f - 0.5f) / 6.0f) < 0.00001,
        "where the gap to the bottom of that sphere divides the translation");
    assert(math::abs(tower.from_below.point.y + 0.5f) < 0.00001, "and the hit point is the bottom of it");

    assert(!tower.beside.hit, "a ray five metres to the side meets no child at all");
}

struct RayValidity {
    bool ordinary;
    bool zero_translation;
    bool negative_fraction;
    bool zero_fraction;
    bool huge_fraction;
    bool not_a_number;
}

fn RayValidity the_rays_box3d_is_asked_to_judge() {
    return {
      .ordinary = a_ray_dropped_from_three_metres().is_valid(),
      .zero_translation = a_ray({ 0.0, 3.0, 0.0 }, { 0.0, 0.0, 0.0 }, 1.0).is_valid(),
      .negative_fraction = a_ray({ 0.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, -1.0).is_valid(),
      .zero_fraction = a_ray({ 0.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 0.0).is_valid(),
      .huge_fraction = a_ray({ 0.0, 3.0, 0.0 }, { 0.0, -6.0, 0.0 }, 1.0e30).is_valid(),
      .not_a_number = a_ray({ 0.0, (float)(0.0 / 0.0), 0.0 }, { 0.0, -6.0, 0.0 }, 1.0).is_valid(),
    };
}

fn void a_ray_is_valid_when_it_is_finite_and_its_fraction_is_in_range() @test {
    RayValidity judged = the_rays_box3d_is_asked_to_judge();

    assert(judged.ordinary, "an ordinary ray is valid");
    assert(!judged.negative_fraction, "a negative fraction is refused");
    assert(!judged.huge_fraction, "so is one past box3d's own huge bound");
    assert(!judged.not_a_number, "and so is a ray with a component that is not a number");
}

// The check is narrower than its name, which is the reason both halves of this sit in one place: a
// caller reaching for it to keep a cast off its degenerate answers gets nothing from it.
fn void a_valid_ray_can_still_be_one_no_cast_can_use() @test {
    RayValidity judged = the_rays_box3d_is_asked_to_judge();
    SphereFromWithin within = the_solid_sphere_a_ray_starts_within();

    assert(judged.zero_translation, "a ray that travels nowhere is valid");
    assert(judged.zero_fraction, "and so is one bounded to none of its own translation");
    assert(within.from_inside.hit && math::abs(within.from_inside.normal.y) < 0.00001,
        "and the ray whose answer carries a zero normal is valid too");
}

struct CastAllocation {
    int baseline;
    int before;
    int after;
    bool cast_ran;
}

// One baseline for the three kinds that allocate, because a mesh, a height field and a compound
// all sit on the same count and a leak in any of them is indistinguishable from a leak in the
// others. Every destroy sits on the line after its build for the same reason.
fn CastAllocation the_byte_count_around_every_ray_cast() {
    int baseline = b3::byte_count();

    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer grid.destroy();

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer flat.destroy();

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;

    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer one_sphere.destroy();

    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };
    b3::RayCastInput drop = a_ray_dropped_from_three_metres();

    int before = b3::byte_count();

    CastReading[7] every = {
      @the_reading_of(unit.ray_cast(drop)),
      @the_reading_of(unit.ray_cast_hollow(a_ray({ 0.0, 0.0, 0.0 }, { 0.0, 6.0, 0.0 }, 1.0))),
      @the_reading_of(rod.ray_cast(drop)),
      @the_reading_of(cube.base.ray_cast(drop)),
      @the_reading_of(mesh.ray_cast(drop)),
      @the_reading_of(flat.ray_cast(a_ray({ 3.0, 3.0, 3.0 }, { 0.0, -6.0, 0.0 }, 1.0))),
      @the_reading_of(one_sphere.ray_cast(drop)),
    };

    int after = b3::byte_count();

    bool all_hit = true;
    foreach (reading : every) {
        all_hit = all_hit && reading.hit;
    }

    return { .baseline = baseline, .before = before, .after = after, .cast_ran = all_hit };
}

fn void casting_rays_at_geometry_allocates_nothing() @test {
    CastAllocation around = the_byte_count_around_every_ray_cast();

    assert(around.cast_ran, "all seven casts must have reached their shape for the count to mean anything");
    assert(around.before > around.baseline, "the geometry three of them are cast at is built and live");
    assert(around.after == around.before, "and answering every one of them allocates nothing");
    assert(b3::byte_count() == around.baseline, "and that geometry gives back every byte it took");
}

struct PhantomGuards {
    fault mesh;
    fault height_field;
    fault compound;
    bool mesh_still_hits;
}

fn PhantomGuards the_refusals_of_a_fraction_past_one() {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer grid.destroy();
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer field.destroy();

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 1.5, 0.0, 1.5 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* compound = b3::create_compound(&def)!!;
    defer compound.destroy();

    b3::RayCastInput past_one = {
      .origin = { 1.5, -3.0, 1.5 },
      .translation = { 0.0, 6.0, 0.0 },
      .max_fraction = 10.0,
    };

    PhantomGuards guards;
    if (catch err = mesh.ray_cast(past_one)) guards.mesh = err;
    if (catch err = field.ray_cast(past_one)) guards.height_field = err;
    if (catch err = compound.ray_cast(past_one)) guards.compound = err;

    b3::RayCastInput from_above = {
      .origin = { 1.5, 3.0, 1.5 },
      .translation = { 0.0, -6.0, 0.0 },
      .max_fraction = 1.0,
    };
    if (try hit = mesh.ray_cast(from_above)) guards.mesh_still_hits = hit.hit;

    return guards;
}

// box3d's triangle solver reports a miss by returning one and the search keeps whatever beats the
// fraction it was given, so a fraction above one lets that miss win: a grid cast at from underneath
// answers a hit at exactly one, three metres above itself. Refused rather than answered.
fn void a_cast_fraction_past_one_is_refused_where_the_solver_would_invent_a_hit() @test {
    PhantomGuards guards = the_refusals_of_a_fraction_past_one();
    fault none;

    assert(guards.mesh == b3::INVALID_DEFINITION, "a mesh will not take a fraction past one");
    assert(guards.height_field == b3::INVALID_DEFINITION, "and neither will a height field");
    assert(guards.compound == b3::INVALID_DEFINITION, "nor a compound, whose children may be meshes");
    assert(guards.mesh_still_hits, "while a fraction of one still answers a real hit");
}

// box3d closes a shape cast on the two radii less one linear slop, so a pair carrying radius
// between them lands that slop past the geometric touch and a pair carrying none lands short of it.
// The slop follows the ambient length unit, so it is written out rather than quoted as a number.
fn float a_linear_slop() {
    return 0.005f * b3::length_units_per_meter();
}

// The proxy's points are borrowed for the length of the call, so every fixture below holds its own
// array across the cast rather than taking one from a helper.
fn b3::ShapeCastInput a_probe_swept(
  b3::Vec3[] points,
  float radius,
  b3::Vec3 translation,
  float max_fraction,
) {
    return {
      .proxy = { .points = points.ptr, .count = (int)points.len, .radius = radius },
      .translation = translation,
      .max_fraction = max_fraction,
    };
}

fn b3::ShapeCastInput a_probe_dropped_from_three_metres(b3::Vec3[] points, float radius) {
    return a_probe_swept(
      points: points,
      radius: radius,
      translation: (b3::Vec3){ 0.0, -6.0, 0.0 },
      max_fraction: 1.0,
    );
}

struct SweepContrast {
    CastReading ray;
    CastReading swept;
}

fn SweepContrast the_ray_and_the_probe_put_to_the_same_sphere() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::Vec3[1] tip = { { 0.0, 3.0, 0.0 } };

    return {
      .ray = @the_reading_of(unit.ray_cast(a_ray_dropped_from_three_metres())),
      .swept = @the_reading_of(unit.shape_cast(a_probe_dropped_from_three_metres(tip[..], 0.25))),
    };
}

fn void a_swept_probe_touches_a_sphere_before_a_ray_reaches_it() @test {
    SweepContrast both = the_ray_and_the_probe_put_to_the_same_sphere();
    float touch = (3.0f - 0.75f) / 6.0f;

    assert(both.ray.hit && both.swept.hit, "the ray and the probe both find the sphere");
    assert(math::abs(both.ray.fraction - (3.0f - 0.5f) / 6.0f) < 0.00001, "the ray runs to the surface itself");
    assert(both.swept.fraction > touch, "the probe stops where its own surface meets the shape, not where its centre would");
    assert(both.swept.fraction < touch + 2.0f * a_linear_slop() / 6.0f, "and past it by no more than a linear slop");
    assert(math::abs(both.swept.point.y - 0.5f) < 0.00001, "touching the top of the sphere");
    assert(math::abs(both.swept.normal.y - 1.0f) < 0.00001, "along the normal there");

    assert(both.ray.iterations == 0, "a ray cast is closed form and counts no iterations");
    assert(both.swept.iterations > 0, "where a sweep searches and reports how far it looked");
}

struct SweptReadings {
    CastReading sphere;
    CastReading capsule;
    CastReading hull;
    CastReading compound;
    CastReading sphere_beside;
    CastReading capsule_beside;
    CastReading hull_beside;
    CastReading compound_beside;
    CastReading receding;
}

fn SweptReadings the_shapes_a_probe_is_swept_at() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
      {
        .sphere = { .center = { 0.0, 1.5, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
    };
    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;

    b3::CompoundData* tower = b3::create_compound(&def)!!;
    defer tower.destroy();

    b3::Vec3[1] tip = { { 0.0, 3.0, 0.0 } };
    b3::Vec3[1] aside = { { 5.0, 3.0, 0.0 } };
    b3::ShapeCastInput dropped = a_probe_dropped_from_three_metres(tip[..], 0.25);
    b3::ShapeCastInput lifted = a_probe_swept(
      points: tip[..],
      radius: 0.25,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 1.0,
    );

    return {
      .sphere = @the_reading_of(unit.shape_cast(dropped)),
      .capsule = @the_reading_of(rod.shape_cast(dropped)),
      .hull = @the_reading_of(cube.base.shape_cast(dropped)),
      .compound = @the_reading_of(tower.shape_cast(dropped)),
      .sphere_beside = @the_reading_of(unit.shape_cast(a_probe_dropped_from_three_metres(aside[..], 0.25))),
      .capsule_beside = @the_reading_of(rod.shape_cast(a_probe_dropped_from_three_metres(aside[..], 0.25))),
      .hull_beside = @the_reading_of(cube.base.shape_cast(a_probe_dropped_from_three_metres(aside[..], 0.25))),
      .compound_beside = @the_reading_of(tower.shape_cast(a_probe_dropped_from_three_metres(aside[..], 0.25))),
      .receding = @the_reading_of(unit.shape_cast(lifted)),
    };
}

// Each band opens at the touch the geometry implies and closes two linear slops later, because the
// one slop box3d gives up is the whole of the difference and a band closed tighter than that would
// be asserting a rounding.
fn void a_swept_probe_reports_where_it_first_touches_each_kind() @test {
    SweptReadings swept = the_shapes_a_probe_is_swept_at();
    float window = 2.0f * a_linear_slop() / 6.0f;

    assert(swept.sphere.hit && swept.capsule.hit && swept.hull.hit && swept.compound.hit,
        "a probe dropped from three metres reaches all four kinds");

    float onto_the_sphere = (3.0f - 0.75f) / 6.0f;
    assert(swept.sphere.fraction > onto_the_sphere && swept.sphere.fraction < onto_the_sphere + window,
        "the sphere is touched where its radius and the probe's leave the gap");

    float onto_the_capsule = (3.0f - 0.5f) / 6.0f;
    assert(swept.capsule.fraction > onto_the_capsule && swept.capsule.fraction < onto_the_capsule + window,
        "the capsule a quarter of a metre lower, its own radius being smaller");

    float onto_the_hull = (3.0f - 0.75f) / 6.0f;
    assert(swept.hull.fraction > onto_the_hull && swept.hull.fraction < onto_the_hull + window,
        "the cube at its top face, which lends no radius of its own");

    float onto_the_upper_sphere = (3.0f - 2.25f) / 6.0f;
    assert(swept.compound.fraction > onto_the_upper_sphere &&
        swept.compound.fraction < onto_the_upper_sphere + window,
        "and the compound at the upper of its two spheres");
    assert(swept.compound.child_index == 1, "which is its second child");
}

fn void a_swept_probe_that_reaches_nothing_reports_no_hit() @test {
    SweptReadings swept = the_shapes_a_probe_is_swept_at();

    assert(swept.sphere.hit && swept.capsule.hit && swept.hull.hit && swept.compound.hit,
        "all four kinds are reached by a probe swept down onto them");
    assert(!swept.sphere_beside.hit && !swept.capsule_beside.hit, "and none of them by one five metres to the side");
    assert(!swept.hull_beside.hit && !swept.compound_beside.hit);
    assert(!swept.receding.hit, "nor by one swept away from the sphere along the same line");
}

struct EncroachReadings {
    CastReading sphere_plain;
    CastReading sphere_encroaching;
    CastReading hull_plain;
    CastReading hull_encroaching;
}

// One probe, one starting position, one shape of each kind: the flag is the only thing that varies
// between the two sphere readings and the two hull readings, and the radius each shape lends is the
// only thing that varies between the pairs.
fn EncroachReadings the_probe_that_starts_inside_two_shapes() {
    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::BoxHull cube = b3::make_cube_hull(0.5);
    b3::Vec3[1] within = { { 0.0, 0.4, 0.0 } };

    b3::ShapeCastInput plain = a_probe_dropped_from_three_metres(within[..], 0.0);
    b3::ShapeCastInput encroaching = plain;
    encroaching.can_encroach = true;

    return {
      .sphere_plain = @the_reading_of(unit.shape_cast(plain)),
      .sphere_encroaching = @the_reading_of(unit.shape_cast(encroaching)),
      .hull_plain = @the_reading_of(cube.base.shape_cast(plain)),
      .hull_encroaching = @the_reading_of(cube.base.shape_cast(encroaching)),
    };
}

fn void a_probe_may_encroach_only_where_the_shape_lends_it_a_radius() @test {
    EncroachReadings inside = the_probe_that_starts_inside_two_shapes();
    float window = 2.0f * a_linear_slop() / 6.0f;

    assert(inside.sphere_plain.hit, "a probe starting inside the sphere is a hit at once");
    assert(math::abs(inside.sphere_plain.fraction) < 0.00001, "at no distance along itself");
    assert(math::abs(inside.sphere_plain.normal.y) < 0.00001, "and with a zero normal");

    assert(inside.sphere_encroaching.hit, "the same probe allowed to encroach is a hit too");
    assert(inside.sphere_encroaching.fraction > 0.0f && inside.sphere_encroaching.fraction < window,
        "a slop further on, having advanced instead of stopping where it began");
    assert(math::abs(inside.sphere_encroaching.normal.y - 1.0f) < 0.00001, "with a real normal off the surface");

    assert(inside.hull_plain.hit && inside.hull_encroaching.hit, "both readings on the cube are hits");
    assert(math::abs(inside.hull_encroaching.fraction) < 0.00001, "and the flag moves neither of them");
    assert(math::abs(inside.hull_encroaching.normal.y) < 0.00001,
        "because a hull lends no radius and the window is never opened");
}

struct MeshSweepReadings {
    CastReading swept_from_above;
    CastReading swept_from_below;
    CastReading ray_from_below;
    CastReading swept_past_the_edge;
}

fn MeshSweepReadings the_grid_mesh_a_probe_is_swept_at() {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer grid.destroy();

    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };
    b3::Vec3[1] above = { { 1.5, 3.0, 1.5 } };
    b3::Vec3[1] below = { { 1.5, -3.0, 1.5 } };
    b3::Vec3[1] aside = { { 50.0, 3.0, 0.0 } };

    b3::ShapeCastInput upward = a_probe_swept(
      points: below[..],
      radius: 0.25,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 1.0,
    );

    return {
      .swept_from_above = @the_reading_of(mesh.shape_cast(a_probe_dropped_from_three_metres(above[..], 0.25))),
      .swept_from_below = @the_reading_of(mesh.shape_cast(upward)),
      .ray_from_below = @the_reading_of(mesh.ray_cast(a_ray({ 1.5, -3.0, 1.5 }, { 0.0, 6.0, 0.0 }, 1.0))),
      .swept_past_the_edge = @the_reading_of(mesh.shape_cast(a_probe_dropped_from_three_metres(aside[..], 0.25))),
    };
}

fn void a_swept_probe_reaches_a_mesh_triangle_from_either_side() @test {
    MeshSweepReadings grid = the_grid_mesh_a_probe_is_swept_at();
    float onto_a_triangle = (3.0f - 0.25f) / 6.0f;
    float window = 2.0f * a_linear_slop() / 6.0f;

    assert(grid.swept_from_above.hit, "a probe dropped onto the grid touches it");
    assert(grid.swept_from_above.fraction > onto_a_triangle &&
        grid.swept_from_above.fraction < onto_a_triangle + window, "a probe radius above the plane");
    assert(math::abs(grid.swept_from_above.normal.y - 1.0f) < 0.00001, "off the wound side of a triangle");

    assert(grid.swept_from_below.hit, "and one raised from underneath touches it as well");
    assert(math::abs(grid.swept_from_below.normal.y + 1.0f) < 0.00001, "off the other side of the same triangle");
    assert(!grid.ray_from_below.hit, "where a ray sent along that second path reports nothing at all");

    assert(!grid.swept_past_the_edge.hit, "a probe dropped fifty metres out touches no triangle");
}

struct FieldSweepReadings {
    CastReading probe_from_above;
    CastReading probe_from_below;
    CastReading probe_past_one;
    CastReading pair_of_points_past_one;
    fault point_at_one;
    fault point_past_one;
}

// The field is read two ways by one function: a single point carrying no radius takes box3d's ray
// solver and everything else takes its sweep, so the readings are paired by which of the two they
// reach rather than by where they start.
fn FieldSweepReadings the_height_field_a_probe_is_swept_at() {
    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer flat.destroy();

    b3::Vec3[1] above = { { 3.0, 3.0, 3.0 } };
    b3::Vec3[1] below = { { 3.0, -3.0, 3.0 } };
    b3::Vec3[2] a_pair_below = { { 3.0, -3.0, 3.0 }, { 3.1, -3.0, 3.0 } };

    b3::ShapeCastInput upward = a_probe_swept(
      points: below[..],
      radius: 0.25,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 1.0,
    );
    b3::ShapeCastInput upward_past_one = a_probe_swept(
      points: below[..],
      radius: 0.25,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 10.0,
    );
    b3::ShapeCastInput a_point_upward = a_probe_swept(
      points: below[..],
      radius: 0.0,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 1.0,
    );
    b3::ShapeCastInput a_point_past_one = a_probe_swept(
      points: below[..],
      radius: 0.0,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 10.0,
    );
    b3::ShapeCastInput a_pair_past_one = a_probe_swept(
      points: a_pair_below[..],
      radius: 0.0,
      translation: (b3::Vec3){ 0.0, 6.0, 0.0 },
      max_fraction: 10.0,
    );

    FieldSweepReadings readings = {
      .probe_from_above = @the_reading_of(flat.shape_cast(a_probe_dropped_from_three_metres(above[..], 0.25))),
      .probe_from_below = @the_reading_of(flat.shape_cast(upward)),
      .probe_past_one = @the_reading_of(flat.shape_cast(upward_past_one)),
      .pair_of_points_past_one = @the_reading_of(flat.shape_cast(a_pair_past_one)),
    };
    if (catch err = flat.shape_cast(a_point_upward)) readings.point_at_one = err;
    if (catch err = flat.shape_cast(a_point_past_one)) readings.point_past_one = err;
    return readings;
}

// The grid builder declares a global height range of plus or minus 256 and box3d stores heights as
// sixteen-bit steps across it, truncating, so the surface sits under the plane by less than one
// step and the band has to hold that as well as the slop.
fn void a_swept_probe_reaches_a_height_field_from_either_side() @test {
    FieldSweepReadings field = the_height_field_a_probe_is_swept_at();
    float one_step_of_the_compressed_range = 512.0f / 65535.0f;
    float onto_the_surface = (3.0f - 0.25f) / 6.0f;
    float window = (one_step_of_the_compressed_range + 2.0f * a_linear_slop()) / 6.0f;

    assert(field.probe_from_above.hit, "a probe dropped onto the field touches it");
    assert(field.probe_from_above.fraction > onto_the_surface &&
        field.probe_from_above.fraction < onto_the_surface + window, "a probe radius above the stored surface");
    assert(field.probe_from_above.point.y <= 0.0f &&
        field.probe_from_above.point.y > -one_step_of_the_compressed_range,
        "which sits under the plane the field was built on by less than one step of its range");
    assert(math::abs(field.probe_from_above.normal.y - 1.0f) < 0.00001, "off a surface facing up");

    assert(field.probe_from_below.hit, "a probe raised from underneath touches it too");
    assert(math::abs(field.probe_from_below.normal.y + 1.0f) < 0.00001, "off the underside of the same triangle");
}

// The refusal and the plain miss are told apart here rather than folded into one reading, because a
// guard that fired on every point probe would be indistinguishable from one that never fired.
fn void a_height_field_reads_a_point_probe_with_its_ray_solver() @test {
    FieldSweepReadings field = the_height_field_a_probe_is_swept_at();

    assert(field.point_at_one == b3::NO_HIT,
        "a single point carrying no radius takes the one-sided solver and finds nothing from below");
    assert(field.point_past_one == b3::INVALID_DEFINITION,
        "and past a fraction of one it is refused, where that solver would answer a miss with a hit");

    assert(field.probe_past_one.hit,
        "a probe with a radius takes the sweep instead, which the same fraction leaves untroubled");
    assert(field.pair_of_points_past_one.hit,
        "and so does a second point, radius or no radius, which is the other half of what picks the solver");
}

struct ProxyRefusals {
    fault sphere;
    fault capsule;
    fault hull;
    fault compound;
    fault mesh;
    fault height_field;
    fault one_point_too_many;
    bool a_full_proxy_still_answers;
    bool an_ordinary_proxy_still_answers;
}

// The crowd's first points sit fifty metres out and its last sits over the grid, so truncating it
// at the ceiling changes the answer rather than only the count.
fn ProxyRefusals the_proxies_the_casts_refuse() {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer grid.destroy();
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer flat.destroy();

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer one_sphere.destroy();

    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::Vec3[1] tip = { { 0.0, 3.0, 0.0 } };
    b3::ShapeCastInput empty = a_probe_dropped_from_three_metres(tip[:0], 0.25);

    b3::Vec3[b3::MAX_SHAPE_CAST_POINTS + 1] crowd;
    foreach (index, &point : crowd) {
        *point = index + 1 < crowd.len ? (b3::Vec3){ 50.0, 3.0, 0.0 } : (b3::Vec3){ 1.5, 3.0, 1.5 };
    }

    ProxyRefusals refusals;
    if (catch err = unit.shape_cast(empty)) refusals.sphere = err;
    if (catch err = rod.shape_cast(empty)) refusals.capsule = err;
    if (catch err = cube.base.shape_cast(empty)) refusals.hull = err;
    if (catch err = one_sphere.shape_cast(empty)) refusals.compound = err;
    if (catch err = mesh.shape_cast(empty)) refusals.mesh = err;
    if (catch err = flat.shape_cast(empty)) refusals.height_field = err;

    b3::ShapeCastInput crowded = a_probe_dropped_from_three_metres(crowd[..], 0.0);
    if (catch err = mesh.shape_cast(crowded)) refusals.one_point_too_many = err;

    b3::ShapeCastInput full = a_probe_dropped_from_three_metres(crowd[1..], 0.0);
    if (try out = mesh.shape_cast(full)) refusals.a_full_proxy_still_answers = out.hit;
    if (try out = unit.shape_cast(a_probe_dropped_from_three_metres(tip[..], 0.25))) {
        refusals.an_ordinary_proxy_still_answers = out.hit;
    }

    return refusals;
}

// box3d reads a proxy of no points as though it held its first point, and truncates one past its
// ceiling in silence, so both answer plausibly about geometry the caller never named — the same
// inversion the mesh ray cast's fraction guard exists for.
fn void a_shape_cast_refuses_a_proxy_box3d_would_answer_wrongly() @test {
    ProxyRefusals refused = the_proxies_the_casts_refuse();

    assert(refused.sphere == b3::INVALID_DEFINITION, "a sphere will not take a proxy of no points");
    assert(refused.capsule == b3::INVALID_DEFINITION, "nor a capsule");
    assert(refused.hull == b3::INVALID_DEFINITION, "nor a hull");
    assert(refused.compound == b3::INVALID_DEFINITION, "nor a compound");
    assert(refused.mesh == b3::INVALID_DEFINITION, "nor a mesh");
    assert(refused.height_field == b3::INVALID_DEFINITION, "nor a height field");

    assert(refused.one_point_too_many == b3::INVALID_DEFINITION,
        "and one point past the ceiling is refused rather than dropped");
    assert(refused.a_full_proxy_still_answers, "where a proxy filled exactly to the ceiling is answered");
    assert(refused.an_ordinary_proxy_still_answers, "as is one of a single point");
}

struct SweepAllocation {
    int baseline;
    int before;
    int after;
    bool cast_ran;
}

// One baseline for the three kinds that allocate, for the reason the ray casts share theirs. Every
// destroy sits on the line after its build.
fn SweepAllocation the_byte_count_around_every_shape_cast() {
    int baseline = b3::byte_count();

    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer grid.destroy();

    b3::HeightFieldData* flat = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer flat.destroy();

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one_sphere = b3::create_compound(&def)!!;
    defer one_sphere.destroy();

    b3::Sphere unit = a_half_metre_sphere_at_the_origin();
    b3::Capsule rod = { .center1 = { -1.0, 0.0, 0.0 }, .center2 = { 1.0, 0.0, 0.0 }, .radius = 0.25 };
    b3::BoxHull cube = b3::make_cube_hull(0.5);
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::Vec3[1] tip = { { 0.0, 3.0, 0.0 } };
    b3::Vec3[1] over_the_grid = { { 1.5, 3.0, 1.5 } };
    b3::Vec3[1] over_the_field = { { 3.0, 3.0, 3.0 } };
    b3::ShapeCastInput dropped = a_probe_dropped_from_three_metres(tip[..], 0.25);

    int before = b3::byte_count();

    CastReading[6] every = {
      @the_reading_of(unit.shape_cast(dropped)),
      @the_reading_of(rod.shape_cast(dropped)),
      @the_reading_of(cube.base.shape_cast(dropped)),
      @the_reading_of(one_sphere.shape_cast(dropped)),
      @the_reading_of(mesh.shape_cast(a_probe_dropped_from_three_metres(over_the_grid[..], 0.25))),
      @the_reading_of(flat.shape_cast(a_probe_dropped_from_three_metres(over_the_field[..], 0.25))),
    };

    int after = b3::byte_count();

    bool all_hit = true;
    foreach (reading : every) {
        all_hit = all_hit && reading.hit;
    }

    return { .baseline = baseline, .before = before, .after = after, .cast_ran = all_hit };
}

fn void sweeping_a_probe_at_geometry_allocates_nothing() @test {
    SweepAllocation around = the_byte_count_around_every_shape_cast();

    assert(around.cast_ran, "all six sweeps must have touched their shape for the count to mean anything");
    assert(around.before > around.baseline, "the geometry three of them are swept at is built and live");
    assert(around.after == around.before, "and answering every one of them allocates nothing");
    assert(b3::byte_count() == around.baseline, "and that geometry gives back every byte it took");
}
