module b3_test;

import b3;
import std::math;

fn b3::Transform an_untouched_placement() {
    return { .p = { 0.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
}

// A half-angle quaternion over a unit axis, written out directly: box3d's own axis-angle
// constructor approximates its sine and lands about a thousandth off, wider than these bands.
fn b3::Transform a_turn_about(b3::Vec3 axis, double radians, b3::Vec3 position) {
    float half = (float)math::sin(0.5 * radians);
    return {
      .p = position,
      .q = { axis.x * half, axis.y * half, axis.z * half, (float)math::cos(0.5 * radians) },
    };
}

fn b3::MassData the_mass_of_a_two_metre_box_hull() {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();
    b3::HullData* hull = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer b3::destroy_hull(hull);

    return hull.compute_mass_data(1.0);
}

fn void a_hull_reports_the_mass_and_inertia_its_own_volume_implies() @test {
    b3::MassData box = the_mass_of_a_two_metre_box_hull();

    double side = 2.0;
    float volume = (float)(side * side * side);
    float inertia = (float)(volume * (side * side + side * side) / 12.0);

    assert(math::abs(box.mass - volume) < 0.001, "a hull weighs its own volume at a density of one");
    assert(math::abs(box.center.x) < 0.001 && math::abs(box.center.y) < 0.001 && math::abs(box.center.z) < 0.001,
        "and a box built about the origin has its centre of mass there");
    assert(math::abs(box.inertia.m00 - inertia) < 0.01, "the inertia of a box is m times h squared plus d squared over twelve");
    assert(math::abs(box.inertia.m11 - inertia) < 0.01);
    assert(math::abs(box.inertia.m22 - inertia) < 0.01);
    assert(math::abs(box.inertia.m10) < 0.001 && math::abs(box.inertia.m20) < 0.001,
        "and a box square to the axes carries no products of inertia");
}

struct BrickMass {
    b3::MassData at_density_one;
    b3::MassData at_density_three;
}

// A box hull carries its own storage, so this weighs one without allocating anything and the
// byte count never moves. Three different half widths are what separate the three inertia axes;
// a cube reports the same number in all three and could not tell a transposition apart.
fn BrickMass the_mass_of_a_brick_hull() {
    b3::BoxHull brick = b3::make_box_hull(1.0, 2.0, 3.0);
    return {
      .at_density_one = brick.base.compute_mass_data(1.0),
      .at_density_three = brick.base.compute_mass_data(3.0),
    };
}

fn void a_hull_measures_a_different_inertia_about_each_of_its_three_axes() @test {
    BrickMass brick = the_mass_of_a_brick_hull();

    double width = 2.0;
    double height = 4.0;
    double depth = 6.0;
    double mass = width * height * depth;
    float about_x = (float)(mass * (height * height + depth * depth) / 12.0);
    float about_y = (float)(mass * (width * width + depth * depth) / 12.0);
    float about_z = (float)(mass * (width * width + height * height) / 12.0);

    assert(math::abs(brick.at_density_one.mass - (float)mass) < 0.01, "a two by four by six brick weighs forty-eight");
    assert(math::abs(brick.at_density_one.inertia.m00 - about_x) < 0.05, "and is hardest to turn about its shortest axis");
    assert(math::abs(brick.at_density_one.inertia.m11 - about_y) < 0.05);
    assert(math::abs(brick.at_density_one.inertia.m22 - about_z) < 0.05);

    assert(math::abs(brick.at_density_three.mass - (float)(3.0 * mass)) < 0.05,
        "mass is linear in density, because box3d scales the volume it baked");
    assert(math::abs(brick.at_density_three.inertia.m00 - 3.0f * about_x) < 0.2, "and so is every entry of the tensor");
    assert(math::abs(brick.at_density_three.inertia.m22 - 3.0f * about_z) < 0.2);
}

fn b3::MassData the_mass_of_a_box_hull_five_metres_along_x() {
    b3::BoxHull placed = b3::make_offset_box_hull(1.0, 1.0, 1.0, (b3::Vec3){ 5.0, 0.0, 0.0 });
    return placed.base.compute_mass_data(1.0);
}

fn b3::MassData the_mass_of_that_same_hull_at_three_times_the_density() {
    b3::BoxHull placed = b3::make_offset_box_hull(1.0, 1.0, 1.0, (b3::Vec3){ 5.0, 0.0, 0.0 });
    return placed.base.compute_mass_data(3.0);
}

fn void a_hull_measures_its_inertia_about_its_own_centre_of_mass() @test {
    b3::MassData placed = the_mass_of_a_box_hull_five_metres_along_x();
    b3::MassData at_the_origin = the_mass_of_a_two_metre_box_hull();

    assert(math::abs(placed.center.x - 5.0) < 0.001, "a box built five metres along x has its centre of mass there");
    assert(math::abs(placed.center.y) < 0.001 && math::abs(placed.center.z) < 0.001);
    assert(math::abs(placed.inertia.m00 - at_the_origin.inertia.m00) < 0.01,
        "and carries the tensor of the same box at the origin, not the one a parallel axis shift would give");
    assert(math::abs(placed.inertia.m22 - at_the_origin.inertia.m22) < 0.01);

    // The centroid is the one term density does not touch, and only an off-origin hull can show
    // it: at the origin a scaled centroid and an unscaled one are the same zero.
    b3::MassData denser = the_mass_of_that_same_hull_at_three_times_the_density();
    assert(math::abs(denser.mass - placed.mass * 3.0) < 0.01, "three times the density weighs three times as much");
    assert(math::abs(denser.center.x - 5.0) < 0.001, "and leaves the centre of mass where the hull is");
}

struct BallMass {
    b3::MassData at_the_origin;
    b3::MassData placed;
}

fn BallMass the_mass_of_a_half_metre_ball() {
    b3::Sphere at_the_origin = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::Sphere placed = { .center = { 1.0, 2.0, 3.0 }, .radius = 0.5 };
    return {
      .at_the_origin = at_the_origin.compute_mass_data(1.0),
      .placed = placed.compute_mass_data(1.0),
    };
}

fn void a_sphere_weighs_its_volume_and_measures_its_inertia_about_its_own_centre() @test {
    BallMass ball = the_mass_of_a_half_metre_ball();

    double radius = 0.5;
    float mass = (float)(4.0 / 3.0 * math::PI * radius * radius * radius);
    float inertia = (float)(0.4 * mass * radius * radius);

    assert(math::abs(ball.at_the_origin.mass - mass) < 0.0001, "four thirds pi r cubed at a density of one");
    assert(math::abs(ball.at_the_origin.inertia.m00 - inertia) < 0.0001, "and two fifths m r squared about every axis alike");
    assert(math::abs(ball.at_the_origin.inertia.m11 - inertia) < 0.0001);
    assert(math::abs(ball.at_the_origin.inertia.m22 - inertia) < 0.0001);

    assert(math::abs(ball.placed.center.x - 1.0) < 0.001, "a sphere away from the origin reports its own centre");
    assert(math::abs(ball.placed.center.y - 2.0) < 0.001);
    assert(math::abs(ball.placed.center.z - 3.0) < 0.001);
    assert(math::abs(ball.placed.inertia.m00 - inertia) < 0.0001,
        "and the same tensor, because it is measured about that centre and not the shape frame's origin");
}

struct RodMass {
    b3::MassData along_y;
    b3::MassData along_x;
}

fn RodMass the_mass_of_a_capsule_laid_along_two_axes() {
    // Offset off the origin so the centre of mass is a value the zero struct cannot produce.
    b3::Capsule along_y = { .center1 = { 3.0, 1.5, -2.0 }, .center2 = { 3.0, 2.5, -2.0 }, .radius = 0.25 };
    b3::Capsule along_x = { .center1 = { -0.5, 0.0, 0.0 }, .center2 = { 0.5, 0.0, 0.0 }, .radius = 0.25 };
    return {
      .along_y = along_y.compute_mass_data(1.0),
      .along_x = along_x.compute_mass_data(1.0),
    };
}

fn void a_capsule_weighs_a_cylinder_and_two_end_caps_about_the_axis_it_lies_on() @test {
    RodMass rod = the_mass_of_a_capsule_laid_along_two_axes();

    double radius = 0.25;
    double height = 1.0;
    double cylinder = math::PI * radius * radius * height;
    double caps = 4.0 / 3.0 * math::PI * radius * radius * radius;
    float axial = (float)(0.5 * cylinder * radius * radius + 0.4 * caps * radius * radius);
    float across = (float)(cylinder * (3.0 * radius * radius + height * height) / 12.0
        + caps * (0.4 * radius * radius + 0.25 * height * height + 0.375 * height * radius));

    assert(math::abs(rod.along_y.mass - (float)(cylinder + caps)) < 0.0001,
        "pi r squared h of cylinder and four thirds pi r cubed of end cap");
    assert(math::abs(rod.along_y.center.x - 3.0) < 0.001, "with the centre of mass midway between the two centres");
    assert(math::abs(rod.along_y.center.y - 2.0) < 0.001);
    assert(math::abs(rod.along_y.center.z + 2.0) < 0.001);
    assert(math::abs(rod.along_y.inertia.m11 - axial) < 0.0001, "about its own axis a capsule is a cylinder and a ball");
    assert(math::abs(rod.along_y.inertia.m00 - across) < 0.0001, "and end over end it carries its caps out along the segment");

    assert(math::abs(rod.along_x.inertia.m00 - axial) < 0.0001,
        "a capsule laid along x reports the axial entry on x, so the tensor follows the segment");
    assert(math::abs(rod.along_x.inertia.m11 - across) < 0.0001);
    assert(math::abs(rod.along_x.inertia.m22 - across) < 0.0001);
}

fn b3::MassData the_mass_of_a_capsule_whose_two_centres_meet() {
    b3::Capsule pinched = { .center1 = { 0.0, 0.0, 0.0 }, .center2 = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    return pinched.compute_mass_data(1.0);
}

// A capsule of no length is a ball, and box3d weighs it as one rather than refusing it — the
// cylinder term falls to zero and the axis it would have been rotated onto is never computed.
fn void a_capsule_of_no_length_weighs_the_ball_its_two_caps_make() @test {
    b3::MassData pinched = the_mass_of_a_capsule_whose_two_centres_meet();

    double radius = 0.5;
    float mass = (float)(4.0 / 3.0 * math::PI * radius * radius * radius);
    float inertia = (float)(0.4 * mass * radius * radius);

    assert(math::abs(pinched.mass - mass) < 0.0001, "two hemispherical caps and no cylinder between them");
    assert(math::abs(pinched.inertia.m00 - inertia) < 0.0001, "and the tensor of a solid ball, equal about all three axes");
    assert(math::abs(pinched.inertia.m11 - inertia) < 0.0001);
    assert(math::abs(pinched.inertia.m22 - inertia) < 0.0001);
}

struct HullBounds {
    b3::Aabb at_rest;
    b3::Aabb placed;
}

fn HullBounds the_bounds_of_a_two_metre_box_hull() {
    b3::Vec3[8] corners = the_corners_of_a_two_metre_box();
    b3::HullData* hull = b3::create_hull(corners[..], b3::MAX_HULL_VERTICES)!!;
    defer b3::destroy_hull(hull);

    return {
      .at_rest = hull.compute_aabb(an_untouched_placement()),
      .placed = hull.compute_aabb(a_placement_five_metres_along_x()),
    };
}

fn void a_hull_reports_the_bounds_its_own_points_imply() @test {
    HullBounds box = the_bounds_of_a_two_metre_box_hull();

    assert(math::abs(box.at_rest.lower_bound.x + 1.0) < 0.001, "the eight corners of a two metre box bound plus or minus one");
    assert(math::abs(box.at_rest.lower_bound.y + 1.0) < 0.001);
    assert(math::abs(box.at_rest.lower_bound.z + 1.0) < 0.001);
    assert(math::abs(box.at_rest.upper_bound.x - 1.0) < 0.001);
    assert(math::abs(box.at_rest.upper_bound.y - 1.0) < 0.001);
    assert(math::abs(box.at_rest.upper_bound.z - 1.0) < 0.001);

    assert(math::abs(box.placed.lower_bound.x - 4.0) < 0.001, "and carry five metres along x when the transform does");
    assert(math::abs(box.placed.upper_bound.x - 6.0) < 0.001);
    assert(math::abs(box.placed.lower_bound.y + 1.0) < 0.001, "leaving the axes the placement does not touch alone");
    assert(math::abs(box.placed.upper_bound.z - 1.0) < 0.001);
}

struct TurnedHullBounds {
    b3::Aabb quarter_turn_from_the_box;
    b3::Aabb quarter_turn_from_the_points;
    b3::Aabb eighth_turn_from_the_box;
    b3::Aabb eighth_turn_from_the_points;
}

// A rock is ten points scattered over a sphere, so its own box is loose around it and the two
// readings can disagree. A cube could not show this: its points are the corners of its box, and
// a rotated box bounds them exactly.
fn TurnedHullBounds the_bounds_of_a_rock_hull_turned_two_ways() {
    b3::Vec3 up = { 0.0, 1.0, 0.0 };
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Vec3 unscaled = { 1.0, 1.0, 1.0 };

    b3::Transform quarter = a_turn_about(up, math::PI / 2.0, origin);
    b3::Transform eighth = a_turn_about(up, math::PI / 4.0, origin);

    b3::HullData* rock = b3::create_rock(1.0)!!;
    defer b3::destroy_hull(rock);

    b3::HullData* turned_a_quarter = b3::clone_and_transform_hull(rock, quarter, unscaled)!!;
    defer b3::destroy_hull(turned_a_quarter);

    b3::HullData* turned_an_eighth = b3::clone_and_transform_hull(rock, eighth, unscaled)!!;
    defer b3::destroy_hull(turned_an_eighth);

    return {
      .quarter_turn_from_the_box = rock.compute_aabb(quarter),
      .quarter_turn_from_the_points = turned_a_quarter.aabb,
      .eighth_turn_from_the_box = rock.compute_aabb(eighth),
      .eighth_turn_from_the_points = turned_an_eighth.aabb,
    };
}

fn void a_hull_turned_off_the_axes_bounds_its_own_box_rather_than_its_points() @test {
    TurnedHullBounds rock = the_bounds_of_a_rock_hull_turned_two_ways();

    float quarter_span = rock.quarter_turn_from_the_box.upper_bound.x - rock.quarter_turn_from_the_box.lower_bound.x;
    assert(1.5 < quarter_span && quarter_span < 2.1, "a rock of radius one spans between a metre and a half and two");

    assert(math::abs(rock.quarter_turn_from_the_box.lower_bound.x - rock.quarter_turn_from_the_points.lower_bound.x) < 0.001,
        "a quarter turn carries a box onto another box, so the two readings agree exactly");
    assert(math::abs(rock.quarter_turn_from_the_box.upper_bound.z - rock.quarter_turn_from_the_points.upper_bound.z) < 0.001);

    float lower_slack = rock.eighth_turn_from_the_points.lower_bound.x - rock.eighth_turn_from_the_box.lower_bound.x;
    float upper_slack = rock.eighth_turn_from_the_box.upper_bound.x - rock.eighth_turn_from_the_points.upper_bound.x;
    assert(0.2 < lower_slack && lower_slack < 0.6,
        "an eighth turn does not, and the box around the rotated box stands well clear of the points");
    assert(0.2 < upper_slack && upper_slack < 0.6);
}

fn b3::Aabb the_bounds_of_a_ball_turned_off_the_axes() {
    b3::Vec3 up = { 0.0, 1.0, 0.0 };
    b3::Sphere ball = { .center = { 1.0, 2.0, 3.0 }, .radius = 0.5 };
    return ball.compute_aabb(a_turn_about(up, math::PI / 4.0, (b3::Vec3){ 4.0, 0.0, 0.0 }));
}

// An eighth turn, because a quarter turn about a coordinate axis carries a box onto a box and the
// widened reading would agree with the exact one. The axis matters more than the angle: a quarter
// turn about a diagonal is not exempt.
fn void a_sphere_keeps_its_radius_through_a_rotation() @test {
    b3::Aabb ball = the_bounds_of_a_ball_turned_off_the_axes();

    double diagonal = math::sqrt(0.5);
    float diameter = (float)1.0;
    float center_x = (float)(4.0 + 4.0 * diagonal);
    float center_y = (float)2.0;
    float center_z = (float)(2.0 * diagonal);

    float across_x = ball.upper_bound.x - ball.lower_bound.x;
    float across_y = ball.upper_bound.y - ball.lower_bound.y;
    float across_z = ball.upper_bound.z - ball.lower_bound.z;
    assert(math::abs(across_x - diameter) < 0.001,
        "a turned sphere is one diameter across each axis, not the wider box a turned cube would give");
    assert(math::abs(across_y - diameter) < 0.001);
    assert(math::abs(across_z - diameter) < 0.001);

    float middle_x = 0.5f * (ball.lower_bound.x + ball.upper_bound.x);
    float middle_y = 0.5f * (ball.lower_bound.y + ball.upper_bound.y);
    float middle_z = 0.5f * (ball.lower_bound.z + ball.upper_bound.z);
    assert(math::abs(middle_x - center_x) < 0.001, "and it sits where the transform carries the sphere's own centre");
    assert(math::abs(middle_y - center_y) < 0.001);
    assert(math::abs(middle_z - center_z) < 0.001);
}

fn b3::Aabb the_bounds_of_a_capsule_turned_off_the_axes() {
    b3::Vec3 forward = { 0.0, 0.0, 1.0 };
    b3::Capsule rod = { .center1 = { 0.0, -0.5, 0.0 }, .center2 = { 0.0, 0.5, 0.0 }, .radius = 0.25 };
    return rod.compute_aabb(a_turn_about(forward, math::PI / 4.0, (b3::Vec3){ 2.0, 0.0, 0.0 }));
}

// An eighth turn for the reason the sphere's is one: the box follows the two centres onto the
// diagonal, where widening the box they started in would answer a whole radius more.
fn void a_capsule_carries_both_of_its_centres_through_the_transform() @test {
    b3::Aabb rod = the_bounds_of_a_capsule_turned_off_the_axes();

    double diagonal = math::sqrt(0.5);
    float across_the_turn = (float)(diagonal + 0.5);
    float diameter = (float)0.5;

    float across_x = rod.upper_bound.x - rod.lower_bound.x;
    float across_y = rod.upper_bound.y - rod.lower_bound.y;
    float across_z = rod.upper_bound.z - rod.lower_bound.z;
    assert(math::abs(across_x - across_the_turn) < 0.001,
        "half a metre of segment laid on the diagonal, and a quarter of radius on each side");
    assert(math::abs(across_y - across_the_turn) < 0.001);
    assert(math::abs(across_z - diameter) < 0.001, "and the radius alone across the axis it turned about");

    assert(math::abs(rod.lower_bound.x - (2.0f - 0.5f * across_the_turn)) < 0.001,
        "centred where the transform carries the midpoint of the two centres");
    assert(math::abs(rod.upper_bound.x - (2.0f + 0.5f * across_the_turn)) < 0.001);
}

struct MeshBounds {
    b3::Aabb grid_unscaled;
    b3::Aabb grid_stretched;
    b3::Aabb grid_stretched_and_carried;
    b3::Aabb offset_box_unscaled;
    b3::Aabb offset_box_mirrored;
}

// The offset box mesh is what a negative scale can be seen on: a grid centred on the origin
// mirrors onto itself and reads the same either way. The carried reading is what separates a
// transform that was applied from one that was dropped, since a zeroed one is the identity.
fn MeshBounds the_bounds_of_two_meshes_read_at_three_scales() {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer b3::destroy_mesh(grid);

    b3::MeshData* offset_box = b3::create_box_mesh(
      (b3::Vec3){ 1.0, 0.0, 0.0 },
      (b3::Vec3){ 1.0, 1.0, 1.0 },
      false,
    )!!;
    defer b3::destroy_mesh(offset_box);

    b3::Transform rest = an_untouched_placement();
    b3::Transform carried = { .p = { 10.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };

    return {
      .grid_unscaled = grid.compute_aabb(rest, (b3::Vec3){ 1.0, 1.0, 1.0 }),
      .grid_stretched = grid.compute_aabb(rest, (b3::Vec3){ 2.0, 1.0, 2.0 }),
      .grid_stretched_and_carried = grid.compute_aabb(carried, (b3::Vec3){ 2.0, 1.0, 2.0 }),
      .offset_box_unscaled = offset_box.compute_aabb(rest, (b3::Vec3){ 1.0, 1.0, 1.0 }),
      .offset_box_mirrored = offset_box.compute_aabb(rest, (b3::Vec3){ -1.0, 1.0, 1.0 }),
    };
}

fn void a_mesh_scales_its_bounds_before_it_places_them() @test {
    MeshBounds mesh = the_bounds_of_two_meshes_read_at_three_scales();

    assert(math::abs(mesh.grid_unscaled.lower_bound.x + 2.0) < 0.001, "four cells a metre wide reach two metres each way");
    assert(math::abs(mesh.grid_unscaled.upper_bound.z - 2.0) < 0.001);
    assert(math::abs(mesh.grid_unscaled.upper_bound.y) < 0.001, "and a flat grid has no thickness at all");

    assert(math::abs(mesh.grid_stretched.lower_bound.x + 4.0) < 0.001, "a scale of two in x and z doubles both");
    assert(math::abs(mesh.grid_stretched.upper_bound.x - 4.0) < 0.001);
    assert(math::abs(mesh.grid_stretched.upper_bound.z - 4.0) < 0.001);
    assert(math::abs(mesh.grid_stretched.upper_bound.y) < 0.001, "the grid being flat, its y extent is zero at any scale");

    assert(math::abs(mesh.grid_stretched_and_carried.lower_bound.x - 6.0) < 0.001,
        "the scale runs in mesh space, so ten metres along x carries the stretched box ten and not twenty");
    assert(math::abs(mesh.grid_stretched_and_carried.upper_bound.x - 14.0) < 0.001);
    assert(math::abs(mesh.grid_stretched_and_carried.upper_bound.z - 4.0) < 0.001, "and leaves the axis it does not move alone");

    assert(math::abs(mesh.offset_box_unscaled.lower_bound.x) < 0.001, "a box mesh a metre along x reaches from nothing to two");
    assert(math::abs(mesh.offset_box_unscaled.upper_bound.x - 2.0) < 0.001);
    assert(math::abs(mesh.offset_box_mirrored.lower_bound.x + 2.0) < 0.001,
        "and a negative scale mirrors it rather than leaving the bounds crossed over");
    assert(math::abs(mesh.offset_box_mirrored.upper_bound.x) < 0.001);
}

struct HeightFieldBounds {
    b3::Aabb unscaled;
    b3::Aabb stretched;
    b3::Aabb stretched_and_carried;
}

fn HeightFieldBounds the_bounds_of_two_height_field_grids() {
    b3::HeightFieldData* unscaled = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: { 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer b3::destroy_height_field(unscaled);

    b3::HeightFieldData* stretched = b3::create_height_field_grid(
      row_count: 8,
      column_count: 8,
      scale: { 2.0, 1.0, 3.0 },
      make_holes: false,
    )!!;
    defer b3::destroy_height_field(stretched);

    b3::Transform rest = an_untouched_placement();
    b3::Transform carried = { .p = { 10.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };

    return {
      .unscaled = unscaled.compute_aabb(rest),
      .stretched = stretched.compute_aabb(rest),
      .stretched_and_carried = stretched.compute_aabb(carried),
    };
}

fn void a_height_field_carries_the_scale_it_was_built_with() @test {
    HeightFieldBounds field = the_bounds_of_two_height_field_grids();

    assert(math::abs(field.unscaled.lower_bound.x) < 0.001, "a grid of eight points a metre apart starts at the origin");
    assert(math::abs(field.unscaled.upper_bound.x - 7.0) < 0.001, "and reaches seven, one span fewer than its count");
    assert(math::abs(field.unscaled.upper_bound.z - 7.0) < 0.001);

    assert(math::abs(field.stretched.upper_bound.x - 14.0) < 0.001,
        "the field's own scale is already in the box this reads, so there is no scale to pass here");
    assert(math::abs(field.stretched.upper_bound.z - 21.0) < 0.001);

    assert(math::abs(field.stretched_and_carried.lower_bound.x - 10.0) < 0.001, "and the placement carries the whole box");
    assert(math::abs(field.stretched_and_carried.upper_bound.x - 24.0) < 0.001);
    assert(math::abs(field.stretched_and_carried.upper_bound.z - 21.0) < 0.001, "leaving the axis it does not move alone");
}

struct CompoundBounds {
    b3::Aabb at_rest;
    b3::Aabb carried;
}

fn CompoundBounds the_bounds_of_a_two_sphere_compound() {
    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { -2.0, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
      {
        .sphere = { .center = { 2.0, 1.0, 0.0 }, .radius = 0.25 },
        .material = b3::default_surface_material(),
      },
    };

    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;

    b3::CompoundData* compound = b3::create_compound(&def)!!;
    defer b3::destroy_compound(compound);

    return {
      .at_rest = compound.compute_aabb(an_untouched_placement())!!,
      .carried = compound.compute_aabb({ .p = { 1.0, 2.0, 3.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } })!!,
    };
}

fn void a_compound_bounds_every_child_it_holds() @test {
    CompoundBounds compound = the_bounds_of_a_two_sphere_compound();

    assert(math::abs(compound.at_rest.lower_bound.x + 2.5) < 0.001, "the far side of the larger sphere");
    assert(math::abs(compound.at_rest.upper_bound.x - 2.25) < 0.001, "and the far side of the smaller one");
    assert(math::abs(compound.at_rest.lower_bound.y + 0.5) < 0.001);
    assert(math::abs(compound.at_rest.upper_bound.y - 1.25) < 0.001, "raised by the height the smaller one sits at");

    assert(math::abs(compound.carried.lower_bound.x + 1.5) < 0.001, "and the whole box moves with the transform");
    assert(math::abs(compound.carried.upper_bound.y - 3.25) < 0.001);
    assert(math::abs(compound.carried.lower_bound.z - 2.5) < 0.001);
}

struct EmptyCompoundBounds {
    fault refusal;
    int baseline;
    int after;
}

fn EmptyCompoundBounds the_bounds_a_childless_compound_reports() {
    b3::CompoundDef childless;

    int baseline = b3::byte_count();

    b3::CompoundData* compound = b3::create_compound(&childless)!!;
    defer b3::destroy_compound(compound);

    fault refusal;
    if (catch err = compound.compute_aabb(an_untouched_placement())) refusal = err;

    return { .refusal = refusal, .baseline = baseline, .after = b3::byte_count() };
}

// box3d indexes the compound's node array at the tree root, and a compound with no children
// carries the root minus one. The read lands back inside the compound's own header and answers a
// zero-extent box at the transform's position, which is what an empty box would look like too.
fn void a_compound_with_no_children_bounds_nothing() @test {
    EmptyCompoundBounds childless = the_bounds_a_childless_compound_reports();

    assert(childless.refusal == b3::DEGENERATE_GEOMETRY, "a compound holding nothing has no bounds to report");
    assert(childless.after > childless.baseline, "and the compound it was asked of was a real one, alive at the time");
    assert(b3::byte_count() == childless.baseline, "which gave every byte back when it was destroyed");
}

fn float[2] the_x_reach_of_a_grid_mesh_flat_and_turned() {
    b3::MeshData* grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: false,
    )!!;
    defer b3::destroy_mesh(grid);

    b3::Vec3 up = { 0.0, 1.0, 0.0 };
    b3::Vec3 origin = { 0.0, 0.0, 0.0 };
    b3::Aabb flat = grid.compute_aabb(an_untouched_placement(), (b3::Vec3){ 1.0, 1.0, 1.0 });
    b3::Aabb turned = grid.compute_aabb(a_turn_about(up, math::PI / 4.0, origin), (b3::Vec3){ 1.0, 1.0, 1.0 });

    return { flat.upper_bound.x, turned.upper_bound.x };
}

// The mesh, height field and compound bounds all carry a cached box through the rotation rather
// than refitting, so an eighth turn reaches the corner of that box rather than the edge of the
// geometry. A square of half width two reaches two root two, which is where the widening shows.
fn void a_mesh_turned_off_the_axes_reports_wider_than_it_reaches() @test {
    float[2] reach = the_x_reach_of_a_grid_mesh_flat_and_turned();

    assert(reach[0] > 1.99 && reach[0] < 2.01, "flat, a four by four grid of unit cells reaches two");
    assert(reach[1] > 2.827 && reach[1] < 2.83, "turned an eighth, the carried box reaches its own corner");
    assert(reach[1] > reach[0] * 1.41, "which is wider than the mesh itself ever goes");
}
