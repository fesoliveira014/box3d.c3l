module b3_test;

import b3;
import std::math;

const int WINDOW_CELLS = 16;
const int MAX_WINDOWS = 4;
const int STEP_COUNT = 150;

// Measured on these rides, the largest deviation from a whole field is 0.00003 in position,
// 0.00015 in height and 0.0004 in speed with aprons. Without them the smallest speed deviation is
// 0.027, more than five times the bound.
const float POSITION_EPSILON = 0.001;
const float HEIGHT_EPSILON = 0.0005;
const float SPEED_EPSILON = 0.005;

enum Stitch {
    WHOLE,
    APRONS,
    NONE,
}

enum Rider {
    SPHERE,
    CAPSULE_ACROSS,
    CAPSULE_ALONG,
}

// The surface is the plane height = per_column * x + per_row * z, so every window cuts the same
// continuous surface and the quantisation range they share is the only thing they must agree on.
struct Slope {
    float per_column;
    float per_row;
}

struct Tiling {
    int windows_x;
    int windows_z;
    Stitch stitch;
}

struct Ride {
    Rider rider;
    Slope slope;
    float start_x;
    float start_z;
    float clearance;
    float velocity_x;
    float velocity_z;
}

struct Track {
    float[STEP_COUNT] x;
    float[STEP_COUNT] y;
    float[STEP_COUNT] speed;
}

struct Deviation {
    float position;
    float height;
    float speed;
}

fn float plane_height(Slope slope, int row, int column) {
    return slope.per_column * (float)column + slope.per_row * (float)row;
}

fn Slope flat() => { .per_column = 0.0, .per_row = 0.0 };

fn Slope tilted() => { .per_column = 0.04, .per_row = 0.02 };

fn Tiling a_seam(Stitch stitch) => { .windows_x = 2, .windows_z = 1, .stitch = stitch };

fn Tiling a_corner(Stitch stitch) => { .windows_x = 2, .windows_z = 2, .stitch = stitch };

fn Ride a_ride(Rider rider, Slope slope) {
    bool slides = rider == Rider.CAPSULE_ALONG;
    return {
        .rider = rider,
        .slope = slope,
        .start_x = slides ? 12.0f : 10.0f,
        .start_z = 8.0f,
        .clearance = 0.0f,
        .velocity_x = slides ? 8.0f : 4.0f,
        .velocity_z = 0.0f,
    };
}

// The field of one window: its samples are cut from the plane at the window's place in the
// tiling, and with an apron the ring around them is cut from the same plane, which is the
// neighbouring window's own samples.
fn b3::HeightFieldData*? a_window(
  Slope slope,
  int first_row,
  int first_column,
  int row_cells,
  int column_cells,
  char apron_sides,
) {
    int padding = apron_sides != 0 ? 1 : 0;
    int padded_rows = row_cells + 1 + 2 * padding;
    int padded_columns = column_cells + 1 + 2 * padding;
    float[] heights = mem::new_array(float, (usz)(padded_rows * padded_columns));
    defer free(heights);

    for (int row = 0; row < padded_rows; row++) {
        for (int column = 0; column < padded_columns; column++) {
            heights[row * padded_columns + column] = plane_height(
              slope,
              first_row + row - padding,
              first_column + column - padding,
            );
        }
    }

    b3::HeightFieldDef def;
    def.heights = heights.ptr;
    def.scale = (b3::Vec3){ 1.0, 1.0, 1.0 };
    def.count_x = column_cells + 1;
    def.count_z = row_cells + 1;
    def.global_minimum_height = -10.0;
    def.global_maximum_height = 10.0;
    def.apron_sides = apron_sides;
    return b3::create_height_field(&def);
}

fn char the_apron_sides_of_a_window(Tiling tiling, int window_x, int window_z) {
    if (tiling.stitch != Stitch.APRONS) return 0;

    char sides = 0;
    if (window_x > 0) sides |= b3::HEIGHT_FIELD_APRON_NEGATIVE_X;
    if (window_x < tiling.windows_x - 1) sides |= b3::HEIGHT_FIELD_APRON_POSITIVE_X;
    if (window_z > 0) sides |= b3::HEIGHT_FIELD_APRON_NEGATIVE_Z;
    if (window_z < tiling.windows_z - 1) sides |= b3::HEIGHT_FIELD_APRON_POSITIVE_Z;
    return sides;
}

fn void destroy_windows(b3::HeightFieldData*[] windows) {
    foreach (window : windows) {
        if (window != null) b3::destroy_height_field(window);
    }
}

fn b3::BodyId a_rider_body(b3::WorldId world, Ride ride) {
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;

    float radius = ride.rider == Rider.SPHERE ? 0.5f : 0.4f;
    float surface = plane_height(ride.slope, (int)ride.start_z, (int)ride.start_x);
    body_def.position = (b3::Pos){ ride.start_x, surface + radius + ride.clearance, ride.start_z };
    b3::BodyId body = b3::create_body(world, &body_def)!!;

    switch (ride.rider) {
        case SPHERE:
            b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = radius };
            b3::create_sphere_shape(body, &shape_def, &sphere)!!;
        case CAPSULE_ACROSS:
            b3::Capsule capsule = {
                .center1 = { 0.0, 0.0, -0.5 },
                .center2 = { 0.0, 0.0, 0.5 },
                .radius = radius,
            };
            b3::create_capsule_shape(body, &shape_def, &capsule)!!;
        case CAPSULE_ALONG:
            b3::Capsule capsule = {
                .center1 = { -0.5, 0.0, 0.0 },
                .center2 = { 0.5, 0.0, 0.0 },
                .radius = radius,
            };
            b3::create_capsule_shape(body, &shape_def, &capsule)!!;
    }

    body.set_linear_velocity((b3::Vec3){ ride.velocity_x, 0.0, ride.velocity_z });
    return body;
}

// Every resource is destroyed here and nothing is asserted: C3 skips defer on a panicking
// assertion, and a stranded height field sits on byte_count() for the rest of the process.
fn Track the_track_of_a_ride(Ride ride, Tiling tiling) {
    b3::HeightFieldData*[MAX_WINDOWS] windows;
    defer destroy_windows(windows[..]);

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::ShapeDef shape_def = b3::default_shape_def();
    bool whole = tiling.stitch == Stitch.WHOLE;
    int window_count = whole ? 1 : tiling.windows_x * tiling.windows_z;

    for (int index = 0; index < window_count; index++) {
        int window_x = whole ? 0 : index % tiling.windows_x;
        int window_z = whole ? 0 : index / tiling.windows_x;
        int row_cells = (whole ? tiling.windows_z : 1) * WINDOW_CELLS;
        int column_cells = (whole ? tiling.windows_x : 1) * WINDOW_CELLS;

        windows[index] = a_window(
          slope: ride.slope,
          first_row: window_z * WINDOW_CELLS,
          first_column: window_x * WINDOW_CELLS,
          row_cells: row_cells,
          column_cells: column_cells,
          apron_sides: the_apron_sides_of_a_window(tiling, window_x, window_z),
        )!!;

        b3::BodyDef ground_def = b3::default_body_def();
        ground_def.position = (b3::Pos){
            (float)(window_x * WINDOW_CELLS),
            0.0,
            (float)(window_z * WINDOW_CELLS),
        };
        b3::BodyId ground = b3::create_body(world, &ground_def)!!;
        b3::create_height_field_shape(ground, &shape_def, windows[index])!!;
    }

    b3::BodyId body = a_rider_body(world, ride);

    Track track;
    for (int step = 0; step < STEP_COUNT; step++) {
        b3::world_step(world, 1.0f / 60.0f, 4);
        b3::Pos position = body.position();
        track.x[step] = position.x;
        track.y[step] = position.y;
        track.speed[step] = body.linear_velocity().x;
    }
    return track;
}

fn Deviation the_deviation_from_the_whole_field(Ride ride, Tiling tiling) {
    Tiling whole_field = {
        .windows_x = tiling.windows_x,
        .windows_z = tiling.windows_z,
        .stitch = Stitch.WHOLE,
    };
    Track reference = the_track_of_a_ride(ride, whole_field);
    Track track = the_track_of_a_ride(ride, tiling);

    Deviation deviation;
    for (int step = 0; step < STEP_COUNT; step++) {
        deviation.position = math::max(deviation.position, math::abs(track.x[step] - reference.x[step]));
        deviation.height = math::max(deviation.height, math::abs(track.y[step] - reference.y[step]));
        deviation.speed = math::max(deviation.speed, math::abs(track.speed[step] - reference.speed[step]));
    }
    return deviation;
}

fn void assert_follows_the_whole_field(Deviation deviation) {
    assert(deviation.position < POSITION_EPSILON, "the rider travels as far as it does on a whole field");
    assert(deviation.height < HEIGHT_EPSILON, "and rises no higher over the seam");
    assert(deviation.speed < SPEED_EPSILON, "and loses no more speed to it");
}

fn void assert_is_slowed_by_the_seam(Deviation deviation) {
    assert(deviation.speed > 2.0f * SPEED_EPSILON, "a seam without aprons slows the rider measurably");
}

fn void a_sphere_rolls_across_an_apron_seam_as_across_a_whole_field() @test {
    Tiling seam = a_seam(Stitch.APRONS);

    assert_follows_the_whole_field(the_deviation_from_the_whole_field(a_ride(Rider.SPHERE, flat()), seam));
    assert_follows_the_whole_field(the_deviation_from_the_whole_field(a_ride(Rider.SPHERE, tilted()), seam));
}

fn void a_capsule_rolls_across_an_apron_seam_as_across_a_whole_field() @test {
    Tiling seam = a_seam(Stitch.APRONS);

    assert_follows_the_whole_field(the_deviation_from_the_whole_field(a_ride(Rider.CAPSULE_ACROSS, flat()), seam));
    assert_follows_the_whole_field(the_deviation_from_the_whole_field(a_ride(Rider.CAPSULE_ACROSS, tilted()), seam));
}

fn void a_capsule_slides_across_an_apron_seam_as_across_a_whole_field() @test {
    Tiling seam = a_seam(Stitch.APRONS);

    assert_follows_the_whole_field(the_deviation_from_the_whole_field(a_ride(Rider.CAPSULE_ALONG, flat()), seam));
    assert_follows_the_whole_field(the_deviation_from_the_whole_field(a_ride(Rider.CAPSULE_ALONG, tilted()), seam));
}

fn void a_sphere_rolls_through_the_corner_of_four_windows_as_across_a_whole_field() @test {
    Ride diagonal = a_ride(Rider.SPHERE, flat());
    diagonal.start_x = 11.0;
    diagonal.start_z = 11.0;
    diagonal.velocity_x = 3.0;
    diagonal.velocity_z = 3.0;

    assert_follows_the_whole_field(the_deviation_from_the_whole_field(diagonal, a_corner(Stitch.APRONS)));
    assert_is_slowed_by_the_seam(the_deviation_from_the_whole_field(diagonal, a_corner(Stitch.NONE)));
}

// The control that makes the tests above mean something: the same rides over the same seam with
// no apron are out by far more than the bounds, so the bounds are not simply loose.
fn void without_aprons_a_seam_slows_a_sphere_and_a_capsule() @test {
    Tiling seam = a_seam(Stitch.NONE);

    assert_is_slowed_by_the_seam(the_deviation_from_the_whole_field(a_ride(Rider.SPHERE, flat()), seam));
    assert_is_slowed_by_the_seam(the_deviation_from_the_whole_field(a_ride(Rider.SPHERE, tilted()), seam));
    assert_is_slowed_by_the_seam(the_deviation_from_the_whole_field(a_ride(Rider.CAPSULE_ACROSS, flat()), seam));
    assert_is_slowed_by_the_seam(the_deviation_from_the_whole_field(a_ride(Rider.CAPSULE_ALONG, flat()), seam));
}

// Dropped from rest, so it comes down on the seam rather than rolling across it.
fn float the_resting_height_of_a_sphere_dropped_at(float x, float z) {
    Ride drop = a_ride(Rider.SPHERE, flat());
    drop.start_x = x;
    drop.start_z = z;
    drop.clearance = 1.0;
    drop.velocity_x = 0.0;

    Track track = the_track_of_a_ride(drop, a_seam(Stitch.APRONS));
    return track.y[STEP_COUNT - 1];
}

fn float[3] the_resting_heights_of_spheres_dropped_on_the_interior_and_the_seam() {
    return {
        the_resting_height_of_a_sphere_dropped_at(8.0, 8.0),
        the_resting_height_of_a_sphere_dropped_at(16.0, 8.0),
        the_resting_height_of_a_sphere_dropped_at(16.0, 8.5),
    };
}

// A sphere centred over the shared edge meets nothing but edges of the two windows, so a contact
// that was dropped there rather than turned onto the face would let it through.
fn void a_sphere_dropped_on_the_shared_edge_rests_as_high_as_one_on_the_interior() @test {
    float[3] resting = the_resting_heights_of_spheres_dropped_on_the_interior_and_the_seam();

    assert(resting[0] > 0.49 && resting[0] < 0.51, "a sphere rests its own radius above a flat field");
    assert(math::abs(resting[1] - resting[0]) < HEIGHT_EPSILON, "and no lower on a vertex of the shared edge");
    assert(math::abs(resting[2] - resting[0]) < HEIGHT_EPSILON, "or half way along one of its segments");
}

fn int[2] the_bytes_of_a_window_with_and_without_an_apron() {
    int baseline = b3::byte_count();
    b3::HeightFieldData* bare = a_window(
      slope: flat(),
      first_row: 0,
      first_column: 0,
      row_cells: WINDOW_CELLS,
      column_cells: WINDOW_CELLS,
      apron_sides: 0,
    )!!;
    defer b3::destroy_height_field(bare);
    int bare_bytes = b3::byte_count() - baseline;

    int before_apron = b3::byte_count();
    char all_sides = b3::HEIGHT_FIELD_APRON_NEGATIVE_X | b3::HEIGHT_FIELD_APRON_POSITIVE_X
      | b3::HEIGHT_FIELD_APRON_NEGATIVE_Z | b3::HEIGHT_FIELD_APRON_POSITIVE_Z;
    b3::HeightFieldData* aproned = a_window(
      slope: flat(),
      first_row: 0,
      first_column: 0,
      row_cells: WINDOW_CELLS,
      column_cells: WINDOW_CELLS,
      apron_sides: all_sides,
    )!!;
    defer b3::destroy_height_field(aproned);

    return { bare_bytes, b3::byte_count() - before_apron };
}

fn void an_apron_is_not_stored_with_the_field() @test {
    int[2] bytes = the_bytes_of_a_window_with_and_without_an_apron();

    assert(bytes[0] > 0, "a field box3d allocates shows up in the byte count");
    assert(bytes[1] == bytes[0], "and the apron around its samples adds nothing to it");
}

fn fault the_refusal_of_a_bit_that_is_no_side() {
    b3::HeightFieldData*? field = a_window(
      slope: flat(),
      first_row: 0,
      first_column: 0,
      row_cells: WINDOW_CELLS,
      column_cells: WINDOW_CELLS,
      apron_sides: 0x10,
    );
    defer if (try field) b3::destroy_height_field(field);

    if (catch err = field) return err;
    return {};
}

fn void an_apron_bit_beyond_the_four_sides_is_refused() @test {
    assert(the_refusal_of_a_bit_that_is_no_side() == b3::INVALID_DEFINITION);
}
