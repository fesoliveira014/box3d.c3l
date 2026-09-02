module b3_test;

import b3;
import std::math;

// What one query reported. The indices are recorded rather than counted, because triangle 0 and
// child 0 are real answers a zeroed struct cannot be told apart from, so every assertion below
// names an index the geometry puts somewhere other than the front.
struct QueryLog {
    int count;
    int first_index;
    int last_index;
    int stop_after;
    b3::Vec3[3] first_triangle;
    int[8] arrival;
    bool[128] seen;
    b3::CompoundData* subject;
}

fn bool log_triangle(b3::Vec3 a, b3::Vec3 b, b3::Vec3 c, int triangle_index, void* context) {
    QueryLog* log = (QueryLog*)context;
    if (log.count == 0) {
        log.first_index = triangle_index;
        log.first_triangle = { a, b, c };
    }
    log.last_index = triangle_index;
    if (triangle_index >= 0 && triangle_index < 128) log.seen[triangle_index] = true;
    if (log.count < 8) log.arrival[log.count] = triangle_index;
    log.count++;
    return log.stop_after == 0 || log.count < log.stop_after;
}

fn bool log_child(b3::CompoundData* compound, int child_index, void* context) {
    QueryLog* log = (QueryLog*)context;
    log.subject = compound;
    if (log.count == 0) log.first_index = child_index;
    log.last_index = child_index;
    if (child_index >= 0 && child_index < 128) log.seen[child_index] = true;
    if (log.count < 8) log.arrival[log.count] = child_index;
    log.count++;
    return log.stop_after == 0 || log.count < log.stop_after;
}

fn b3::Aabb a_box_from(b3::Vec3 lower, b3::Vec3 upper) {
    return { .lower_bound = lower, .upper_bound = upper };
}

fn b3::Aabb a_box_around_everything() {
    return a_box_from({ -100.0, -100.0, -100.0 }, { 100.0, 100.0, 100.0 });
}

fn b3::Vec3 the_normal_of(b3::Vec3 a, b3::Vec3 b, b3::Vec3 c) {
    b3::Vec3 u = b - a;
    b3::Vec3 v = c - a;
    return { u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x };
}

// A four by four grid of unit cells spans minus two to two and holds 32 triangles, two per cell,
// numbered so the cell nearest the far corner is the last pair.
fn b3::MeshData*? a_four_by_four_grid() {
    return b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    );
}

// Eight by eight grid points make seven by seven cells and 98 triangles. Holes are punched at
// every sixteenth cell, which lands on the cells numbered 16, 32 and 48.
fn b3::HeightFieldData*? an_eight_by_eight_field(bool make_holes) {
    return b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: make_holes,
    );
}

struct MeshQueryReadings {
    QueryLog everything;
    QueryLog far_corner_cell;
    QueryLog above_the_plane;
    QueryLog fifty_metres_out;
    QueryLog stopped_early;
}

fn MeshQueryReadings the_triangles_a_grid_mesh_reports() {
    b3::MeshData* grid = a_four_by_four_grid()!!;
    defer grid.destroy();
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    QueryLog everything;
    mesh.query(a_box_around_everything(), &log_triangle, &everything);

    QueryLog far_corner_cell;
    mesh.query(a_box_from({ 1.1, -1.0, 1.1 }, { 1.9, 1.0, 1.9 }), &log_triangle, &far_corner_cell);

    QueryLog above_the_plane;
    mesh.query(a_box_from({ -1.9, 0.5, -1.9 }, { -1.1, 1.0, -1.1 }), &log_triangle, &above_the_plane);

    QueryLog fifty_metres_out;
    mesh.query(a_box_from({ 50.0, -1.0, 50.0 }, { 51.0, 1.0, 51.0 }), &log_triangle, &fifty_metres_out);

    QueryLog stopped_early = { .stop_after = 3 };
    mesh.query(a_box_around_everything(), &log_triangle, &stopped_early);

    return {
      .everything = everything,
      .far_corner_cell = far_corner_cell,
      .above_the_plane = above_the_plane,
      .fifty_metres_out = fifty_metres_out,
      .stopped_early = stopped_early,
    };
}

fn void a_mesh_reports_the_triangles_a_box_meets() @test {
    MeshQueryReadings reported = the_triangles_a_grid_mesh_reports();

    assert(reported.everything.count == 32, "a box around the whole grid reaches both triangles of all sixteen cells");
    assert(reported.everything.last_index == 31, "and the last of them is the last triangle the mesh holds");

    assert(reported.far_corner_cell.count == 2, "a box inside one cell reaches that cell's two triangles");
    assert(reported.far_corner_cell.first_index == 30, "which for the cell at the far corner are the final pair");
    assert(reported.far_corner_cell.last_index == 31);
    assert(math::abs(reported.far_corner_cell.first_triangle[0].x - 1.0f) < 0.00001 &&
        math::abs(reported.far_corner_cell.first_triangle[0].z - 1.0f) < 0.00001,
        "and the vertices arrive in the mesh's own frame, at the cell's near corner");

    assert(reported.fifty_metres_out.count == 0, "a box fifty metres away reaches nothing");
}

fn void a_mesh_tests_each_triangle_rather_than_the_node_holding_it() @test {
    MeshQueryReadings reported = the_triangles_a_grid_mesh_reports();

    assert(reported.everything.count == 32, "the grid answers at all, so the empty answer below means something");
    assert(reported.above_the_plane.count == 0,
        "a box half a metre over the grid reaches no triangle, though the node holding them spans it");
}

fn void a_mesh_query_ends_when_its_callback_says_so() @test {
    MeshQueryReadings reported = the_triangles_a_grid_mesh_reports();

    assert(reported.everything.count == 32, "the same box unstopped reaches every triangle");
    assert(reported.stopped_early.count == 3, "and a callback answering false on its third call ends it there");
}

struct CastAgreement {
    bool hit;
    int cast_triangle;
    float cast_fraction;
    QueryLog around_the_hit;
}

fn CastAgreement the_triangle_a_cast_and_a_query_both_name() {
    b3::MeshData* grid = a_four_by_four_grid()!!;
    defer grid.destroy();
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::RayCastInput dropped = {
      .origin = { 1.35, 3.0, 1.35 },
      .translation = { 0.0, -6.0, 0.0 },
      .max_fraction = 1.0,
    };

    QueryLog around_the_hit;
    if (try struck = mesh.ray_cast(dropped)) {
        b3::Vec3 point = struck.point;
        mesh.query(
          a_box_from({ point.x - 0.01f, point.y - 0.01f, point.z - 0.01f },
            { point.x + 0.01f, point.y + 0.01f, point.z + 0.01f }),
          &log_triangle,
          &around_the_hit,
        );
        return {
          .hit = true,
          .cast_triangle = struck.triangle_index,
          .cast_fraction = struck.fraction,
          .around_the_hit = around_the_hit,
        };
    }
    return {};
}

fn void a_query_reports_the_triangle_a_cast_at_the_same_mesh_struck() @test {
    CastAgreement agreed = the_triangle_a_cast_and_a_query_both_name();

    assert(agreed.hit, "the ray must reach the grid for the two indices to be comparable");
    assert(math::abs(agreed.cast_fraction - 0.5f) < 0.00001, "half of a six metre drop onto the plane y = 0");
    assert(agreed.cast_triangle == 30, "the cast names a triangle other than the mesh's first");
    assert(agreed.around_the_hit.count == 2, "a box around the hit point reaches the cell it landed in");
    assert(agreed.around_the_hit.seen[agreed.cast_triangle],
        "and the index a query reports is the index a cast reports");
}

struct ScaledReadings {
    QueryLog unscaled;
    QueryLog doubled;
    b3::Vec3 doubled_first_vertex;
}

// The box sits between three and four metres out, which is past a grid of unit cells and inside
// the same grid read at twice the size — so the scale alone decides whether anything is reported.
fn ScaledReadings the_same_box_put_to_a_mesh_at_two_scales() {
    b3::MeshData* grid = a_four_by_four_grid()!!;
    defer grid.destroy();
    b3::Mesh unit = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };
    b3::Mesh doubled = { .data = grid, .scale = { 2.0, 2.0, 2.0 } };

    b3::Aabb out_at_three = a_box_from({ 3.1, -1.0, 3.1 }, { 3.9, 1.0, 3.9 });

    QueryLog at_one;
    unit.query(out_at_three, &log_triangle, &at_one);

    QueryLog at_two;
    doubled.query(out_at_three, &log_triangle, &at_two);

    return {
      .unscaled = at_one,
      .doubled = at_two,
      .doubled_first_vertex = at_two.first_triangle[0],
    };
}

fn void a_mesh_is_queried_in_the_frame_its_scale_makes() @test {
    ScaledReadings put = the_same_box_put_to_a_mesh_at_two_scales();

    assert(put.unscaled.count == 0, "at its own size the grid stops two metres out and the box is past it");
    assert(put.doubled.count == 2, "read at twice the size the same box lands inside its far corner cell");
    assert(put.doubled.first_index == 30, "on the same pair of triangles the unscaled cell would hold");
    assert(math::abs(put.doubled_first_vertex.x - 2.0f) < 0.00001 &&
        math::abs(put.doubled_first_vertex.z - 2.0f) < 0.00001,
        "and the vertex comes back doubled, not at the one the mesh stores");
}

struct WindingReadings {
    b3::Vec3 plain_normal;
    b3::Vec3 mirrored_normal;
    float plain_first_x;
    float mirrored_second_x;
    int plain_count;
    int mirrored_count;
}

fn WindingReadings the_winding_a_mirrored_scale_reports() {
    b3::MeshData* grid = a_four_by_four_grid()!!;
    defer grid.destroy();
    b3::Mesh plain = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };
    b3::Mesh mirrored = { .data = grid, .scale = { -1.0, 1.0, 1.0 } };

    QueryLog upright;
    plain.query(a_box_from({ 0.1, -1.0, 0.1 }, { 0.9, 1.0, 0.9 }), &log_triangle, &upright);

    QueryLog reflected;
    mirrored.query(a_box_from({ -0.9, -1.0, 0.1 }, { -0.1, 1.0, 0.9 }), &log_triangle, &reflected);

    return {
      .plain_normal = the_normal_of(upright.first_triangle[0], upright.first_triangle[1], upright.first_triangle[2]),
      .mirrored_normal = the_normal_of(reflected.first_triangle[0], reflected.first_triangle[1],
        reflected.first_triangle[2]),
      .plain_first_x = upright.first_triangle[0].x,
      .mirrored_second_x = reflected.first_triangle[1].x,
      .plain_count = upright.count,
      .mirrored_count = reflected.count,
    };
}

fn void a_mirroring_scale_keeps_the_winding_of_the_triangles_it_reports() @test {
    WindingReadings read = the_winding_a_mirrored_scale_reports();

    assert(read.plain_count == 2 && read.mirrored_count == 2, "both meshes answer the cell put to them");
    assert(math::abs(read.plain_first_x - 1.0f) < 0.00001, "the plain mesh reports the cell's far corner at plus one");
    assert(math::abs(read.mirrored_second_x) < 0.00001,
        "where the mirrored one reports a second vertex the plain one gives third");

    assert(read.plain_normal.y > 0.5, "the plain triangle faces up");
    assert(read.mirrored_normal.y > 0.5,
        "and so does the mirrored one, box3d having swapped two vertices to keep it that way");
}

struct InvertedReadings {
    int mesh_upright;
    int mesh_inverted;
    int field_upright;
    int field_inverted;
}

fn InvertedReadings the_answers_an_inverted_box_gets() {
    b3::MeshData* grid = a_four_by_four_grid()!!;
    defer grid.destroy();
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = an_eight_by_eight_field(false)!!;
    defer flat.destroy();

    b3::Aabb upright = a_box_around_everything();
    b3::Aabb inverted = a_box_from({ 100.0, 100.0, 100.0 }, { -100.0, -100.0, -100.0 });

    QueryLog mesh_upright;
    mesh.query(upright, &log_triangle, &mesh_upright);
    QueryLog mesh_inverted;
    mesh.query(inverted, &log_triangle, &mesh_inverted);

    QueryLog field_upright;
    flat.query(upright, &log_triangle, &field_upright);
    QueryLog field_inverted;
    flat.query(inverted, &log_triangle, &field_inverted);

    return {
      .mesh_upright = mesh_upright.count,
      .mesh_inverted = mesh_inverted.count,
      .field_upright = field_upright.count,
      .field_inverted = field_inverted.count,
    };
}

fn void an_inverted_box_is_repaired_by_a_mesh_and_read_as_empty_by_a_field() @test {
    InvertedReadings got = the_answers_an_inverted_box_gets();

    assert(got.mesh_upright == 32 && got.field_upright == 98, "both answer the same box written the right way round");
    assert(got.mesh_inverted == 32,
        "a mesh takes the box's own minimum and maximum, so turning it round changes nothing");
    assert(got.field_inverted == 0, "where a field walks from the lower cell to the upper and finds no cells to walk");
}

struct FieldQueryReadings {
    QueryLog everything;
    QueryLog half_a_cell;
    QueryLog above_the_cell;
    QueryLog fifty_metres_out;
    QueryLog stopped_early;
    QueryLog wider_than_its_indices;
}

fn FieldQueryReadings the_triangles_a_flat_field_reports() {
    b3::HeightFieldData* flat = an_eight_by_eight_field(false)!!;
    defer flat.destroy();

    QueryLog everything;
    flat.query(a_box_around_everything(), &log_triangle, &everything);

    QueryLog half_a_cell;
    flat.query(a_box_from({ 4.02, -0.1, 3.7 }, { 4.10, 0.1, 3.9 }), &log_triangle, &half_a_cell);

    QueryLog above_the_cell;
    flat.query(a_box_from({ 4.02, 5.0, 3.7 }, { 4.10, 6.0, 3.9 }), &log_triangle, &above_the_cell);

    QueryLog fifty_metres_out;
    flat.query(a_box_from({ 50.0, -1.0, 50.0 }, { 51.0, 1.0, 51.0 }), &log_triangle, &fifty_metres_out);

    QueryLog stopped_early = { .stop_after = 3 };
    flat.query(a_box_around_everything(), &log_triangle, &stopped_early);

    QueryLog wider_than_its_indices;
    flat.query(a_box_from({ -1.0e30, -1.0e30, -1.0e30 }, { 1.0e30, 1.0e30, 1.0e30 }), &log_triangle,
      &wider_than_its_indices);

    return {
      .everything = everything,
      .half_a_cell = half_a_cell,
      .above_the_cell = above_the_cell,
      .fifty_metres_out = fifty_metres_out,
      .stopped_early = stopped_early,
      .wider_than_its_indices = wider_than_its_indices,
    };
}

fn void a_height_field_reports_both_triangles_of_every_cell_a_box_reaches() @test {
    FieldQueryReadings reported = the_triangles_a_flat_field_reports();

    assert(reported.everything.count == 98, "seven by seven cells carry two triangles each");
    assert(reported.everything.last_index == 97, "and the last of them is the field's last triangle");

    assert(reported.half_a_cell.count == 2, "a box lying in one half of one cell still reports the whole cell");
    assert(reported.half_a_cell.first_index == 50, "the pair belonging to the cell four columns and three rows out");
    assert(reported.half_a_cell.last_index == 51,
        "the second of which the box does not touch, the test being the cell's box rather than the triangle");

    assert(reported.above_the_cell.count == 0,
        "a box five metres over that cell reaches neither, so the cell's box does bound it");
    assert(reported.fifty_metres_out.count == 0, "and a box fifty metres away reaches no cell at all");
}

fn void a_height_field_query_ignores_what_its_callback_returns() @test {
    FieldQueryReadings reported = the_triangles_a_flat_field_reports();

    assert(reported.everything.count == 98, "the same box read to the end");
    assert(reported.stopped_early.count == 98,
        "and a callback answering false on its third call is called for every one of them regardless");
}

fn void a_box_spanning_more_cells_than_a_field_can_index_reports_nothing() @test {
    FieldQueryReadings reported = the_triangles_a_flat_field_reports();

    assert(reported.everything.count == 98, "a box two hundred metres across reaches the whole field");
    assert(reported.wider_than_its_indices.count == 0,
        "where widening it past what a signed int of cells spans reports nothing at all");
}

struct HoleQueryReadings {
    int flat_total;
    int holed_total;
    int over_the_hole;
    int over_its_neighbour;
}

fn HoleQueryReadings the_triangles_a_holed_field_withholds() {
    b3::HeightFieldData* flat = an_eight_by_eight_field(false)!!;
    defer flat.destroy();
    b3::HeightFieldData* holed = an_eight_by_eight_field(true)!!;
    defer holed.destroy();

    QueryLog whole_flat;
    flat.query(a_box_around_everything(), &log_triangle, &whole_flat);

    QueryLog whole_holed;
    holed.query(a_box_around_everything(), &log_triangle, &whole_holed);

    // Cell 16 is the second row's third column, punched out; cell 17 is the one beside it.
    QueryLog on_the_hole;
    holed.query(a_box_from({ 2.1, -0.1, 2.1 }, { 2.9, 0.1, 2.9 }), &log_triangle, &on_the_hole);

    QueryLog beside_it;
    holed.query(a_box_from({ 3.1, -0.1, 2.1 }, { 3.9, 0.1, 2.9 }), &log_triangle, &beside_it);

    return {
      .flat_total = whole_flat.count,
      .holed_total = whole_holed.count,
      .over_the_hole = on_the_hole.count,
      .over_its_neighbour = beside_it.count,
    };
}

fn void a_hole_cell_reports_neither_of_its_triangles() @test {
    HoleQueryReadings withheld = the_triangles_a_holed_field_withholds();

    assert(withheld.flat_total == 98, "an unpunched field of this size holds 98 triangles");
    assert(withheld.holed_total == 92, "and punching every sixteenth cell out withholds three cells' worth");
    assert(withheld.over_the_hole == 0, "a box over a punched cell reports neither triangle");
    assert(withheld.over_its_neighbour == 2, "where the cell beside it reports both");
}

struct CompoundQueryReadings {
    QueryLog both;
    QueryLog right_only;
    QueryLog at_the_right_box_corner;
    QueryLog fifty_metres_out;
    bool bare_sphere_reaches_the_corner;
    bool the_callback_was_given_the_compound;
}

// Two half metre spheres a metre either side of the origin. The child at index 1 is the one on
// the right, so every assertion below names it rather than the child a zeroed log would name.
fn CompoundQueryReadings the_children_a_two_sphere_compound_reports() {
    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { -1.0, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
      {
        .sphere = { .center = { 1.0, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
    };
    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;
    b3::CompoundData* pair = b3::create_compound(&def)!!;
    defer pair.destroy();

    QueryLog both;
    pair.query(a_box_around_everything(), &log_child, &both);

    QueryLog right_only;
    pair.query(a_box_from({ 0.4, -1.0, -1.0 }, { 1.6, 1.0, 1.0 }), &log_child, &right_only);

    QueryLog at_the_right_box_corner;
    pair.query(a_box_from({ 1.45, 0.45, 0.45 }, { 1.5, 0.5, 0.5 }), &log_child, &at_the_right_box_corner);

    QueryLog fifty_metres_out;
    pair.query(a_box_from({ 50.0, -1.0, -1.0 }, { 51.0, 1.0, 1.0 }), &log_child, &fifty_metres_out);

    b3::Sphere bare = { .center = { 1.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Transform here = { .p = { 0.0, 0.0, 0.0 }, .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 } };
    b3::Vec3[1] corner = { { 1.475, 0.475, 0.475 } };
    b3::ShapeProxy at_the_corner = { .points = &corner[0], .count = 1, .radius = 0.0 };
    bool reaches = false;
    if (try answer = bare.overlaps(here, at_the_corner)) reaches = answer;

    return {
      .both = both,
      .right_only = right_only,
      .at_the_right_box_corner = at_the_right_box_corner,
      .fifty_metres_out = fifty_metres_out,
      .bare_sphere_reaches_the_corner = reaches,
      .the_callback_was_given_the_compound = both.subject == pair,
    };
}

fn void a_compound_reports_the_children_a_box_reaches() @test {
    CompoundQueryReadings reported = the_children_a_two_sphere_compound_reports();

    assert(reported.both.count == 2, "a box around the whole compound reaches both children");
    assert(reported.both.seen[0] && reported.both.seen[1], "and names each of them once");

    assert(reported.right_only.count == 1, "a box around one sphere reaches that child alone");
    assert(reported.right_only.first_index == 1, "the sphere on the right being the second child");

    assert(reported.fifty_metres_out.count == 0, "and a box fifty metres away reaches neither");
    assert(reported.the_callback_was_given_the_compound, "the callback is handed the compound it was called on");
}

fn void a_compound_answers_from_a_child_s_bounds_rather_than_its_geometry() @test {
    CompoundQueryReadings reported = the_children_a_two_sphere_compound_reports();

    assert(reported.at_the_right_box_corner.count == 1,
        "a box at the corner of a sphere child's bounds reports that child");
    assert(reported.at_the_right_box_corner.first_index == 1, "naming the sphere whose bounds it clipped");
    assert(!reported.bare_sphere_reaches_the_corner,
        "though the same sphere put to the same corner directly does not reach it");
}

struct ChildIndexReadings {
    bool queried;
    int reported_index;
    b3::ShapeType kind_at_that_index;
    b3::ShapeType kind_at_the_other;
}

// One capsule and one sphere, which box3d bakes in that order whatever order the definition names
// them in, so the two children answer to different kinds and the index can be checked by kind.
fn ChildIndexReadings the_kind_behind_the_index_a_query_reports() {
    b3::CompoundCapsuleDef rod = {
      .capsule = { .center1 = { -3.0, 0.0, 0.0 }, .center2 = { -2.0, 0.0, 0.0 }, .radius = 0.25 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundSphereDef ball = {
      .sphere = { .center = { 3.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.capsules = &rod;
    def.capsule_count = 1;
    def.spheres = &ball;
    def.sphere_count = 1;
    b3::CompoundData* mixed = b3::create_compound(&def)!!;
    defer mixed.destroy();

    QueryLog over_the_sphere;
    mixed.query(a_box_from({ 2.4, -1.0, -1.0 }, { 3.6, 1.0, 1.0 }), &log_child, &over_the_sphere);
    if (over_the_sphere.count != 1) return {};

    int reported = over_the_sphere.first_index;
    b3::ChildShape named = mixed.child(reported)!!;
    b3::ChildShape other = mixed.child(1 - reported)!!;

    return {
      .queried = true,
      .reported_index = reported,
      .kind_at_that_index = named.type,
      .kind_at_the_other = other.type,
    };
}

fn void the_index_a_compound_query_reports_is_the_one_its_child_accessor_takes() @test {
    ChildIndexReadings named = the_kind_behind_the_index_a_query_reports();

    assert(named.queried, "a box around the sphere alone must reach exactly one child");
    assert(named.reported_index == 1, "the sphere sits behind the capsule in box3d's baking order");
    assert(named.kind_at_that_index == b3::ShapeType.SPHERE,
        "and the index the query reported names the sphere the box was drawn around");
    assert(named.kind_at_the_other == b3::ShapeType.CAPSULE, "where the other index names the capsule");
}

struct CompoundOrderReadings {
    int count;
    int[8] arrival;
    bool each_child_once;
    int stopped_early;
}

fn CompoundOrderReadings the_order_four_children_arrive_in() {
    b3::CompoundSphereDef[4] spheres;
    for (int i = 0; i < 4; i++) {
        spheres[i] = {
          .sphere = { .center = { 2.0f * i, 0.0, 0.0 }, .radius = 0.5 },
          .material = b3::default_surface_material(),
        };
    }
    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 4;
    b3::CompoundData* row = b3::create_compound(&def)!!;
    defer row.destroy();

    QueryLog all_four;
    row.query(a_box_around_everything(), &log_child, &all_four);

    QueryLog stopped = { .stop_after = 1 };
    row.query(a_box_around_everything(), &log_child, &stopped);

    bool once = true;
    for (int i = 0; i < 4; i++) {
        once = once && all_four.seen[i];
    }

    return {
      .count = all_four.count,
      .arrival = all_four.arrival,
      .each_child_once = once,
      .stopped_early = stopped.count,
    };
}

fn void a_compound_reports_its_children_in_tree_order_rather_than_index_order() @test {
    CompoundOrderReadings arrived = the_order_four_children_arrive_in();

    assert(arrived.count == 4, "every child of a four child compound is reported once");
    assert(arrived.each_child_once, "and the four indices between them cover the compound");

    int total = 0;
    bool ascending = true;
    for (int i = 0; i < 4; i++) {
        total += arrived.arrival[i];
        if (i > 0) ascending = ascending && arrived.arrival[i - 1] < arrived.arrival[i];
    }
    assert(total == 6, "the four that arrived are the four the definition named");
    assert(!ascending, "but not in the order it named them in, which is box3d's tree talking");
}

fn void a_compound_query_ends_when_its_callback_says_so() @test {
    CompoundOrderReadings arrived = the_order_four_children_arrive_in();

    assert(arrived.count == 4, "the same box unstopped reaches every child");
    assert(arrived.stopped_early == 1, "and a callback answering false on its first call ends it there");
}

// The empty answer is put beside a one-child compound taking the same box through the same
// callback, because a count of nothing is also what a query that never ran would report.
fn void a_compound_with_no_children_reports_none() @test {
    b3::CompoundDef nothing_in_it;
    b3::CompoundData* empty = b3::create_compound(&nothing_in_it)!!;
    defer empty.destroy();

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef one_in_it;
    one_in_it.spheres = &sphere;
    one_in_it.sphere_count = 1;
    b3::CompoundData* single = b3::create_compound(&one_in_it)!!;
    defer single.destroy();

    QueryLog from_the_empty;
    empty.query(a_box_around_everything(), &log_child, &from_the_empty);

    QueryLog from_the_single;
    single.query(a_box_around_everything(), &log_child, &from_the_single);

    assert(from_the_single.count == 1, "the box reaches the one child a compound of one holds");
    assert(from_the_empty.count == 0,
        "and a childless compound answers it without reaching for the tree root it has not got");
}

struct QueryAllocation {
    int baseline;
    int before;
    int after;
    bool every_query_answered;
}

fn QueryAllocation the_byte_count_around_every_query() {
    int baseline = b3::byte_count();

    b3::MeshData* grid = a_four_by_four_grid()!!;
    defer grid.destroy();
    b3::Mesh mesh = { .data = grid, .scale = { 1.0, 1.0, 1.0 } };

    b3::HeightFieldData* flat = an_eight_by_eight_field(false)!!;
    defer flat.destroy();

    b3::CompoundSphereDef sphere = {
      .sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 },
      .material = b3::default_surface_material(),
    };
    b3::CompoundDef def;
    def.spheres = &sphere;
    def.sphere_count = 1;
    b3::CompoundData* one = b3::create_compound(&def)!!;
    defer one.destroy();

    int before = b3::byte_count();

    QueryLog triangles;
    mesh.query(a_box_around_everything(), &log_triangle, &triangles);
    QueryLog cells;
    flat.query(a_box_around_everything(), &log_triangle, &cells);
    QueryLog children;
    one.query(a_box_around_everything(), &log_child, &children);

    int after = b3::byte_count();

    return {
      .baseline = baseline,
      .before = before,
      .after = after,
      .every_query_answered = triangles.count == 32 && cells.count == 98 && children.count == 1,
    };
}

fn void asking_geometry_which_of_it_meets_a_box_allocates_nothing() @test {
    QueryAllocation around = the_byte_count_around_every_query();

    assert(around.every_query_answered, "all three must have found something for the count to mean anything");
    assert(around.before > around.baseline, "the geometry they are put to is built and live");
    assert(around.after == around.before, "and answering every one of them allocates nothing");
    assert(b3::byte_count() == around.baseline, "and that geometry gives back every byte it took");
}
