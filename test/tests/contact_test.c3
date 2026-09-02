module b3_test;

import b3;
import std::math;

fn b3::WorldId a_world_with_a_ball_resting_on_ground(b3::BodyId* out_ball, b3::ShapeId* out_ball_shape) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull ground_box = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    ground.create_hull_shape(&ground_shape_def, &ground_box.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 0.5, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.density = 1.0;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    b3::ShapeId ball_shape = ball.create_sphere_shape(&shape_def, &sphere)!!;

    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    *out_ball = ball;
    *out_ball_shape = ball_shape;
    return world;
}

fn void a_resting_body_reports_its_touching_contact() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);
    assert(contacts.len == 1, "the resting ball touches the ground and nothing else");

    b3::ContactData contact = contacts[0];
    assert(contact.contact_id.is_valid(), "the reported contact is live");

    bool names_the_ball = contact.shape_id_a.index1 == ball_shape.index1
                       || contact.shape_id_b.index1 == ball_shape.index1;
    assert(names_the_ball, "and names the ball's shape as one of its two");
}

fn void contact_capacity_is_an_upper_bound_on_what_is_written() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    int capacity = ball.contact_capacity();
    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);

    assert(capacity >= 1, "the resting ball has at least one contact to report");
    assert((usz)capacity >= contacts.len, "capacity never under-reports what contact_data writes");

    b3::ContactData[] nothing = ball.contact_data(storage[:0]);
    assert(nothing.len == 0, "a zero-length buffer is filled with nothing");
}

fn void a_contact_carries_its_manifold_geometry() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);
    assert(contacts.len == 1, "the resting ball reports its one contact");

    b3::ContactData contact = contacts[0];
    b3::Manifold[] manifolds = contact.contact_manifolds();
    assert(manifolds.len == (usz)contact.manifold_count, "the slice spans exactly the reported count");
    assert(manifolds.len >= 1, "a touching contact has geometry");

    b3::Manifold manifold = manifolds[0];
    float normal_length = math::sqrt(
      manifold.normal.x * manifold.normal.x
      + manifold.normal.y * manifold.normal.y
      + manifold.normal.z * manifold.normal.z);
    assert(normal_length > 0.999f && normal_length < 1.001f, "the manifold normal is a unit vector");
    assert(manifold.normal.y > 0.9f || manifold.normal.y < -0.9f,
        "and points along the axis separating a ball from flat ground");

    b3::ManifoldPoint[] points = manifold.contact_points();
    assert(points.len == (usz)manifold.point_count, "the point slice spans exactly point_count");
    assert(points.len >= 1 && points.len <= 4, "a manifold carries between one and four points");
    assert(points[0].separation > -0.01f && points[0].separation < 0.01f,
        "a resting sphere sits within a hair of the ground, neither overlapping nor floating");
}

fn void a_shape_reports_the_same_contact_as_its_body() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] body_storage;
    b3::ContactData[] from_body = ball.contact_data(body_storage[..]);

    b3::ContactData[8] shape_storage;
    b3::ContactData[] from_shape = ball_shape.contact_data(shape_storage[..]);

    assert(from_body.len == 1, "the ball has one contact");
    assert(from_shape.len == from_body.len, "the ball's only shape carries the ball's only contact");
    assert(from_shape[0].contact_id.index1 == from_body[0].contact_id.index1, "the same contact");
    assert(ball_shape.contact_capacity() >= 1, "and the shape reports capacity for it");
}

fn void a_contact_id_survives_a_step_and_dies_with_its_body() @test {
    b3::BodyId ball;
    b3::ShapeId ball_shape;
    b3::WorldId world = a_world_with_a_ball_resting_on_ground(&ball, &ball_shape);
    defer world.destroy();

    b3::ContactData[8] storage;
    b3::ContactData[] contacts = ball.contact_data(storage[..]);
    assert(contacts.len == 1, "the resting ball reports its one contact");

    b3::ContactId held = contacts[0].contact_id;

    world.step(1.0f / 60.0f, 4);
    assert(held.is_valid(), "a resting contact outlives a step");

    b3::ContactData refetched = held.data();
    assert(refetched.contact_manifolds().len >= 1, "and its geometry can be fetched again");

    ball.destroy();
    assert(!held.is_valid(), "destroying a body ends its contacts");
}

const float SLAB_FRICTION = 0.3;
const float BALL_FRICTION = 0.7;
const float SLAB_RESTITUTION = 0.1;
const float BALL_RESTITUTION = 0.2;

struct MixingRecord {
    int calls;
    float first_a;
    float first_b;
}

MixingRecord friction_record;
MixingRecord restitution_record;
float friction_answer;
float restitution_answer;

fn float answer_friction(float friction_a, ulong material_a, float friction_b, ulong material_b) {
    if (friction_record.calls == 0) {
        friction_record.first_a = friction_a;
        friction_record.first_b = friction_b;
    }
    friction_record.calls++;
    return friction_answer;
}

fn float answer_restitution(float restitution_a, ulong material_a, float restitution_b, ulong material_b) {
    if (restitution_record.calls == 0) {
        restitution_record.first_a = restitution_a;
        restitution_record.first_b = restitution_b;
    }
    restitution_record.calls++;
    return restitution_answer;
}

fn b3::WorldId a_world_of_two_distinct_surfaces(b3::BodyId* out_ball) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull slab = b3::make_box_hull(40.0, 0.5, 40.0);
    b3::ShapeDef slab_def = b3::default_shape_def();
    slab_def.base_material.friction = SLAB_FRICTION;
    slab_def.base_material.restitution = SLAB_RESTITUTION;
    ground.create_hull_shape(&slab_def, &slab.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 4.0, 0.0};
    b3::BodyId ball = world.create_body(&body_def)!!;
    b3::ShapeDef ball_def = b3::default_shape_def();
    ball_def.density = 1.0;
    ball_def.base_material.friction = BALL_FRICTION;
    ball_def.base_material.restitution = BALL_RESTITUTION;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    ball.create_sphere_shape(&ball_def, &sphere)!!;

    *out_ball = ball;
    return world;
}

struct Drop {
    float final_y;
    float final_vy;
    float late_peak;
}

fn Drop the_drop_of_a_ball(bool install_friction, bool install_restitution) {
    b3::BodyId ball;
    b3::WorldId world = a_world_of_two_distinct_surfaces(&ball);
    defer world.destroy();

    if (install_friction) world.set_friction_callback(&answer_friction);
    if (install_restitution) world.set_restitution_callback(&answer_restitution);

    float late_peak = -1000.0f;
    for (int i = 0; i < 240; i++) {
        world.step(1.0f / 60.0f, 4);
        if (i >= 180 && ball.position().y > late_peak) late_peak = ball.position().y;
    }

    return {
        .final_y = ball.position().y,
        .final_vy = ball.linear_velocity().y,
        .late_peak = late_peak,
    };
}

// The closing condition: the same ball, the same slab and the same 240 steps, with nothing
// changed but the world's restitution mixing. The two surfaces carry 0.1 and 0.2 of their own,
// so the callback's 1.0 is a value neither shape holds and the default max(a, b) cannot reach.
fn void a_restitution_callback_keeps_a_ball_bouncing_that_otherwise_rests() @test {
    restitution_record = (MixingRecord){};
    restitution_answer = 1.0f;

    Drop plain = the_drop_of_a_ball(false, false);
    Drop bouncing = the_drop_of_a_ball(false, true);

    assert(plain.final_y > 0.49f && plain.final_y < 0.51f, "without a callback the ball ends on the slab");
    assert(plain.late_peak > 0.49f && plain.late_peak < 0.51f, "and does not leave it over the last second");
    assert(plain.final_vy > -0.01f && plain.final_vy < 0.01f, "with nothing left of the fall that put it there");

    assert(restitution_record.calls >= 1, "installing the callback puts it on the world's contact path");
    assert(bouncing.late_peak > 3.0f && bouncing.late_peak < 4.5f,
        "and the same ball is still reaching most of its drop height in that last second");
    assert(bouncing.final_y > 1.0f && bouncing.final_y < 3.5f, "clear of the slab at the last step");
    assert(bouncing.final_vy < -1.0f && bouncing.final_vy > -12.0f, "and still moving, rather than settled");
}

// Both surfaces reach both callbacks. The four values differ from each other, so a callback
// handed the wrong pair, one value twice, or an already-mixed number cannot pass. Which of the
// two is a and which is b follows box3d's own ordering of the pair, so either order is accepted.
fn void the_mixing_callbacks_are_handed_both_surfaces_own_values() @test {
    friction_record = (MixingRecord){};
    restitution_record = (MixingRecord){};
    friction_answer = 0.5f;
    restitution_answer = 0.5f;

    the_drop_of_a_ball(true, true);

    assert(friction_record.calls >= 1, "the friction callback was asked");
    bool friction_pair = (friction_record.first_a == SLAB_FRICTION && friction_record.first_b == BALL_FRICTION)
                      || (friction_record.first_a == BALL_FRICTION && friction_record.first_b == SLAB_FRICTION);
    assert(friction_pair, "and was handed the slab's friction and the ball's");

    assert(restitution_record.calls >= 1, "the restitution callback was asked");
    bool restitution_pair = (restitution_record.first_a == SLAB_RESTITUTION && restitution_record.first_b == BALL_RESTITUTION)
                         || (restitution_record.first_a == BALL_RESTITUTION && restitution_record.first_b == SLAB_RESTITUTION);
    assert(restitution_pair, "and that one the two restitutions, which are not the two frictions");
}

struct ReplacedDrop {
    Drop drop;
    int calls_while_installed;
    int calls_after_the_null;
}

// The callback is installed and stepped until box3d has actually asked it, so the null that follows
// undoes a live callback rather than an unused one. Without those first steps the install could be
// deleted outright and nothing would notice.
fn ReplacedDrop the_drop_after_a_callback_is_replaced_by_null() {
    b3::BodyId ball;
    b3::WorldId world = a_world_of_two_distinct_surfaces(&ball);
    defer world.destroy();

    world.set_restitution_callback(&answer_restitution);
    for (int i = 0; i < 120; i++) world.step(1.0f / 60.0f, 4);
    int installed = restitution_record.calls;

    world.set_restitution_callback(null);
    restitution_record = (MixingRecord){};

    float late_peak = -1000.0f;
    for (int i = 0; i < 360; i++) {
        world.step(1.0f / 60.0f, 4);
        if (i >= 300 && ball.position().y > late_peak) late_peak = ball.position().y;
    }

    return {
        .drop = {
            .final_y = ball.position().y,
            .final_vy = ball.linear_velocity().y,
            .late_peak = late_peak,
        },
        .calls_while_installed = installed,
        .calls_after_the_null = restitution_record.calls,
    };
}

// A null argument is the documented way back to box3d's own mixing, and it has to be told apart
// from a callback that was never installed: the bouncing drop above proves the same 1.0 callback
// changes the outcome, so a ball that rests again is the default doing the mixing.
fn void a_null_callback_restores_the_mixing_box3d_ships() @test {
    restitution_record = (MixingRecord){};
    restitution_answer = 1.0f;

    ReplacedDrop replaced = the_drop_after_a_callback_is_replaced_by_null();

    assert(replaced.calls_while_installed > 0, "box3d asked the callback while it was installed");
    assert(replaced.calls_after_the_null == 0, "and never again once a null replaced it");
    assert(replaced.drop.final_y > 0.49f && replaced.drop.final_y < 0.51f,
        "so the ball rests, as it does with no callback at all");
    assert(replaced.drop.late_peak > 0.49f && replaced.drop.late_peak < 0.51f,
        "never leaving that rest over the last second");
    assert(replaced.drop.final_vy > -0.01f && replaced.drop.final_vy < 0.01f,
        "with nothing left of the fall a 1.0 mixing would have kept");
}

struct Slide {
    float final_x;
    float final_vx;
    int calls;
}

fn Slide the_slide_of_a_box(bool install, float answer) {
    friction_record = (MixingRecord){};
    friction_answer = answer;

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, -0.5, 0.0};
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::BoxHull slab = b3::make_box_hull(200.0, 0.5, 200.0);
    b3::ShapeDef slab_def = b3::default_shape_def();
    slab_def.base_material.friction = SLAB_FRICTION;
    ground.create_hull_shape(&slab_def, &slab.base)!!;

    body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = {0.0, 0.25, 0.0};
    body_def.linear_velocity = {10.0, 0.0, 0.0};
    b3::BodyId slider = world.create_body(&body_def)!!;
    b3::ShapeDef slider_def = b3::default_shape_def();
    slider_def.density = 1.0;
    slider_def.base_material.friction = BALL_FRICTION;
    b3::BoxHull box = b3::make_box_hull(0.5, 0.25, 0.5);
    slider.create_hull_shape(&slider_def, &box.base)!!;

    // Contact recycling skips the narrow phase, and the mixing with it, whenever the pair has
    // barely moved; turning it off is what makes the call count a reading of the callback.
    world.set_contact_recycle_distance(0.0f);
    if (install) world.set_friction_callback(&answer_friction);

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    return {
        .final_x = slider.position().x,
        .final_vx = slider.linear_velocity().x,
        .calls = friction_record.calls,
    };
}

// Friction is answered in how far the box gets, not in a call count: the same push over the same
// two seconds ends four times further along with the callback answering 0 than with it answering
// 1, and box3d's own mixing of 0.3 and 0.7 lands between the two.
fn void a_friction_callback_decides_how_far_a_pushed_box_slides() @test {
    Slide standard = the_slide_of_a_box(false, 0.0f);
    Slide frictionless = the_slide_of_a_box(true, 0.0f);
    Slide gripping = the_slide_of_a_box(true, 1.0f);

    assert(frictionless.final_x > 19.0f && frictionless.final_x < 21.0f,
        "a callback answering 0 lets the box slide the whole two seconds");
    assert(frictionless.final_vx > 9.5f && frictionless.final_vx < 10.5f, "at the speed it was pushed with");
    assert(gripping.final_x > 4.0f && gripping.final_x < 6.0f, "and one answering 1 stops it in a quarter of that");
    assert(gripping.final_vx > -0.5f && gripping.final_vx < 0.5f, "with nothing left of the push");
    assert(standard.final_x > gripping.final_x && standard.final_x < frictionless.final_x,
        "while box3d's own mixing of the two surfaces falls between them");
    assert(frictionless.calls > 100 && frictionless.calls <= 120, "the callback was asked on the steps the box was touching");
    assert(gripping.calls > 50 && gripping.calls <= 120, "and on the fewer steps the gripping box was still being solved");
}

// What null restores, stated as a formula rather than as "the default": a callback answering
// sqrt(0.3 * 0.7) leaves the slide box3d's own mixing gives, and the same callback 0.05 higher
// moves the box far enough that the first comparison is known to have resolution.
fn void box3d_s_own_friction_mixing_is_the_root_of_the_product() @test {
    float root_of_the_product = math::sqrt(SLAB_FRICTION * BALL_FRICTION);
    Slide standard = the_slide_of_a_box(false, 0.0f);
    Slide reproduced = the_slide_of_a_box(true, root_of_the_product);
    Slide nearby = the_slide_of_a_box(true, root_of_the_product + 0.05f);

    float reproduced_gap = math::abs(reproduced.final_x - standard.final_x);
    float nearby_gap = math::abs(nearby.final_x - standard.final_x);

    assert(standard.final_x > 10.0f && standard.final_x < 11.5f, "the unmixed slide is the one being reproduced");
    assert(reproduced_gap < 0.01f, "which a callback answering the root of the product reproduces");
    assert(nearby_gap > 0.5f && nearby_gap < 5.0f, "while 0.05 more moves the box well past that tolerance");
}

b3::WorldId mixing_world;
int mixing_pre_solve_calls;

// Installs both callbacks on mixing_world. box3d's pre-solve signature is what it carries so the
// same body can be run from inside a step and from between two of them, which is the whole of the
// contrast below: a body that installed nothing would answer both runs with a silent zero.
fn bool set_the_mixing(
  b3::ShapeId shape_id_a,
  b3::ShapeId shape_id_b,
  b3::Pos point,
  b3::Vec3 normal,
  void* context,
) {
    mixing_pre_solve_calls++;
    mixing_world.set_friction_callback(&answer_friction);
    mixing_world.set_restitution_callback(&answer_restitution);
    return true;
}

fn b3::WorldId a_world_that_keeps_solving_one_contact(b3::BodyId* out_ball) {
    b3::WorldId world = a_world_of_two_distinct_surfaces(out_ball);
    world.enable_sleeping(false);
    world.set_contact_recycle_distance(0.0f);
    return world;
}

struct LockedMixing {
    int inside_attempts;
    int inside_friction_calls;
    int inside_restitution_calls;
    int outside_attempts;
    int outside_friction_calls;
    int outside_restitution_calls;
}

fn LockedMixing the_mixing_installed_from_inside_and_from_between_steps() {
    friction_answer = 0.5f;
    restitution_answer = 0.5f;
    LockedMixing readings;

    friction_record = (MixingRecord){};
    restitution_record = (MixingRecord){};
    mixing_pre_solve_calls = 0;
    {
        b3::BodyId ball;
        mixing_world = a_world_that_keeps_solving_one_contact(&ball);
        defer mixing_world.destroy();

        mixing_world.set_pre_solve_callback(&set_the_mixing, null);
        b3::ShapeId[4] storage;
        foreach (shape : ball.shapes(storage[..])) {
            shape.enable_pre_solve_events(true);
        }
        for (int i = 0; i < 240; i++) {
            mixing_world.step(1.0f / 60.0f, 4);
        }
        readings.inside_attempts = mixing_pre_solve_calls;
        readings.inside_friction_calls = friction_record.calls;
        readings.inside_restitution_calls = restitution_record.calls;
    }

    friction_record = (MixingRecord){};
    restitution_record = (MixingRecord){};
    mixing_pre_solve_calls = 0;
    {
        b3::BodyId ball;
        mixing_world = a_world_that_keeps_solving_one_contact(&ball);
        defer mixing_world.destroy();

        for (int i = 0; i < 240; i++) {
            set_the_mixing(
              shape_id_a: {},
              shape_id_b: {},
              point: {},
              normal: {},
              context: null,
            );
            mixing_world.step(1.0f / 60.0f, 4);
        }
        readings.outside_attempts = mixing_pre_solve_calls;
        readings.outside_friction_calls = friction_record.calls;
        readings.outside_restitution_calls = restitution_record.calls;
    }

    return readings;
}

// A locked world silently keeps the mixing it has. One function installs both callbacks, run from
// inside step() in the first world and between two steps in the second; sleeping and contact
// recycling are off in both, so an install that took hold is asked on most of the steps after it.
fn void a_locked_world_keeps_the_mixing_it_has() @test {
    LockedMixing readings = the_mixing_installed_from_inside_and_from_between_steps();

    assert(readings.outside_attempts == 240, "the installer ran once before each step of the unlocked world");
    assert(readings.outside_friction_calls > 100, "and what it installs is asked for most of them");
    assert(readings.outside_restitution_calls > 100, "both halves of it");
    assert(readings.inside_attempts > 100, "the same installer ran on most of the locked world's steps");
    assert(readings.inside_friction_calls == 0, "and installed neither half");
    assert(readings.inside_restitution_calls == 0);
}
