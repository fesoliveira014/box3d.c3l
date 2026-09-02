module b3_test;

import b3;
import std::math;

const int A_DEBUG_SHAPE_CAP = 8;
const int A_DRAWN_SPHERE_COUNT = 3;

struct SpherePlacement {
    b3::Pos position;
    b3::Quat rotation;
    float radius;
}

// Three spheres no two of which share a coordinate, one of them turned a quarter about y, so a
// fixture that drew one shape three times or handed back the identity pose could not pass.
const SpherePlacement[A_DRAWN_SPHERE_COUNT] THREE_SPHERES = {
    {
      .position = { -2.0, 1.0, 0.0 },
      .rotation = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
      .radius = 0.5,
    },
    {
      .position = { 0.5, 3.0, 0.25 },
      .rotation = { .i = 0.0, .j = 0.70710678, .k = 0.0, .l = 0.70710678 },
      .radius = 0.25,
    },
    {
      .position = { 3.0, 0.75, -1.5 },
      .rotation = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
      .radius = 0.75,
    },
};

struct DrawnShape {
    b3::ShapeId shape_id;
    b3::ShapeType type;
    b3::WorldTransform transform;
    b3::Pos body_position;
    int draws;
}

struct DrawTally {
    int offered;
    int withdrawn;
    int shapes_drawn;
    int bounds_drawn;
    int segments_drawn;
    int points_drawn;
    int strings_drawn;
    b3::Aabb bounds_union;
    DrawnShape[A_DEBUG_SHAPE_CAP] shapes;
}

fn void* offer_a_drawable_shape(b3::DebugShape* debug_shape, void* context) {
    DrawTally* tally = (DrawTally*)context;
    if (tally.offered >= A_DEBUG_SHAPE_CAP) return null;

    DrawnShape* drawn = &tally.shapes[tally.offered];
    drawn.shape_id = debug_shape.shape_id;
    drawn.type = debug_shape.type;
    tally.offered++;
    return drawn;
}

fn void withdraw_a_drawable_shape(void* user_shape, void* context) {
    DrawTally* tally = (DrawTally*)context;
    tally.withdrawn++;
}

fn void draw_a_shape(void* user_shape, b3::WorldTransform transform, b3::HexColor color, void* context) {
    DrawTally* tally = (DrawTally*)context;
    DrawnShape* drawn = (DrawnShape*)user_shape;
    drawn.transform = transform;
    drawn.draws++;
    tally.shapes_drawn++;
}

fn void draw_shape_bounds(b3::Aabb aabb, b3::HexColor color, void* context) {
    DrawTally* tally = (DrawTally*)context;
    if (tally.bounds_drawn == 0) {
        tally.bounds_union = aabb;
    } else {
        for (int axis = 0; axis < 3; axis++) {
            if (aabb.lower_bound[axis] < tally.bounds_union.lower_bound[axis]) {
                tally.bounds_union.lower_bound[axis] = aabb.lower_bound[axis];
            }
            if (aabb.upper_bound[axis] > tally.bounds_union.upper_bound[axis]) {
                tally.bounds_union.upper_bound[axis] = aabb.upper_bound[axis];
            }
        }
    }
    tally.bounds_drawn++;
}

fn void draw_a_segment(b3::Pos p1, b3::Pos p2, b3::HexColor color, void* context) {
    DrawTally* tally = (DrawTally*)context;
    tally.segments_drawn++;
}

fn void draw_a_point(b3::Pos p, float size, b3::HexColor color, void* context) {
    DrawTally* tally = (DrawTally*)context;
    tally.points_drawn++;
}

fn void draw_a_string(b3::Pos p, ZString text, b3::HexColor color, void* context) {
    DrawTally* tally = (DrawTally*)context;
    tally.strings_drawn++;
}

// The five callbacks these tests count through. The other four stay box3d's own empty bodies,
// which is what lets a flag be raised over them without a crash.
fn b3::DebugDraw a_renderer_wired_to(DrawTally* tally) {
    b3::DebugDraw draw = b3::default_debug_draw();
    draw.draw_shape_fcn = &draw_a_shape;
    draw.draw_bounds_fcn = &draw_shape_bounds;
    draw.draw_segment_fcn = &draw_a_segment;
    draw.draw_point_fcn = &draw_a_point;
    draw.draw_string_fcn = &draw_a_string;
    draw.context = tally;
    return draw;
}

// The debug shape callbacks are a world definition field, so answering for a shape is decided
// here and cannot be changed afterwards. Nothing asserts: the world would be stranded.
fn b3::WorldId a_world_of_three_spheres(DrawTally* tally, bool answer_for_its_shapes) {
    b3::WorldDef world_def = b3::default_world_def();
    if (answer_for_its_shapes) {
        world_def.create_debug_shape = &offer_a_drawable_shape;
        world_def.destroy_debug_shape = &withdraw_a_drawable_shape;
        world_def.user_debug_shape_context = tally;
    }
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    foreach (SpherePlacement placement : THREE_SPHERES) {
        b3::BodyDef body_def = b3::default_body_def();
        body_def.position = placement.position;
        body_def.rotation = placement.rotation;
        b3::BodyId body = world.create_body(&body_def)!!;
        b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = placement.radius };
        body.create_sphere_shape(&shape_def, &sphere)!!;
    }
    return world;
}

fn float the_distance_between(b3::Pos a, b3::Pos b) {
    b3::Vec3 apart = a - b;
    return math::sqrt(apart.x * apart.x + apart.y * apart.y + apart.z * apart.z);
}

struct DrawPasses {
    int offered_under_the_default_flags;
    int drawn_under_the_default_flags;
    int bounds_under_the_default_flags;
    int bounds_with_bounds_raised;
    int shapes_with_bounds_raised;
    b3::Aabb bounds_union;
    int offered_with_shapes_raised;
    int drawn_with_shapes_raised;
    int offered_by_a_second_pass;
    int drawn_by_a_second_pass;
    int drawn_under_a_zero_mask;
    int withdrawn_by_destroying_the_world;
    DrawnShape[A_DEBUG_SHAPE_CAP] shapes;
}

// Each pass raises exactly one flag over default_debug_draw(), so a reading names the flag it
// came from rather than a combination.
fn DrawPasses the_readings_from_five_draw_passes_over_three_spheres() {
    DrawTally tally;
    b3::WorldId world = a_world_of_three_spheres(&tally, true);

    DrawPasses passes;

    b3::DebugDraw nothing_raised = a_renderer_wired_to(&tally);
    world.draw(&nothing_raised, ulong::max);
    passes.offered_under_the_default_flags = tally.offered;
    passes.drawn_under_the_default_flags = tally.shapes_drawn;
    passes.bounds_under_the_default_flags = tally.bounds_drawn;

    b3::DebugDraw bounds_raised = a_renderer_wired_to(&tally);
    bounds_raised.draw_bounds = true;
    world.draw(&bounds_raised, ulong::max);
    passes.bounds_with_bounds_raised = tally.bounds_drawn;
    passes.shapes_with_bounds_raised = tally.shapes_drawn;
    passes.bounds_union = tally.bounds_union;

    b3::DebugDraw shapes_raised = a_renderer_wired_to(&tally);
    shapes_raised.draw_shapes = true;
    world.draw(&shapes_raised, ulong::max);
    passes.offered_with_shapes_raised = tally.offered;
    passes.drawn_with_shapes_raised = tally.shapes_drawn;

    world.draw(&shapes_raised, ulong::max);
    passes.offered_by_a_second_pass = tally.offered - passes.offered_with_shapes_raised;
    passes.drawn_by_a_second_pass = tally.shapes_drawn - passes.drawn_with_shapes_raised;

    int drawn_before_the_zero_mask = tally.shapes_drawn;
    world.draw(&shapes_raised, 0);
    passes.drawn_under_a_zero_mask = tally.shapes_drawn - drawn_before_the_zero_mask;

    for (int index = 0; index < tally.offered; index++) {
        tally.shapes[index].body_position = tally.shapes[index].shape_id.body().position();
    }
    passes.shapes = tally.shapes;

    world.destroy();
    passes.withdrawn_by_destroying_the_world = tally.withdrawn;
    return passes;
}

fn void a_draw_pass_hands_each_shape_the_pose_its_body_holds() @test {
    DrawPasses passes = the_readings_from_five_draw_passes_over_three_spheres();

    assert(passes.offered_with_shapes_raised == A_DRAWN_SPHERE_COUNT
        && passes.drawn_with_shapes_raised == A_DRAWN_SPHERE_COUNT,
        "raising draw_shapes offers every shape once and draws every answer");

    for (int sphere = 0; sphere < A_DRAWN_SPHERE_COUNT; sphere++) {
        int drawn_there = 0;
        for (int slot = 0; slot < A_DRAWN_SPHERE_COUNT; slot++) {
            if (the_distance_between(passes.shapes[slot].body_position, THREE_SPHERES[sphere].position) < 1.0e-5) {
                drawn_there++;
            }
        }
        assert(drawn_there == 1, "each sphere reached the callbacks once, at its own position and no other's");
    }

    int turned = 0;
    int upright = 0;
    for (int slot = 0; slot < A_DRAWN_SPHERE_COUNT; slot++) {
        DrawnShape drawn = passes.shapes[slot];
        assert(drawn.type == b3::ShapeType.SPHERE, "the offered shape names the geometry it carries");
        assert(drawn.draws == 2, "and was drawn by both passes that raised draw_shapes, and by neither other");
        assert(the_distance_between(drawn.transform.p, drawn.body_position) < 1.0e-5,
            "the pose handed to the callback is the one box3d holds for that shape's body");
        if (drawn.transform.q.j > 0.7 && drawn.transform.q.j < 0.72
            && drawn.transform.q.l > 0.7 && drawn.transform.q.l < 0.72) {
            turned++;
        } else if (drawn.transform.q.l > 0.999 && drawn.transform.q.l < 1.001) {
            upright++;
        }
    }
    assert(turned == 1 && upright == 2, "the rotation reaches the callback with the position, not an identity");

    assert(passes.withdrawn_by_destroying_the_world == A_DRAWN_SPHERE_COUNT,
        "and destroying the world gives every answer back to the callback that made it");
}

fn void the_default_flags_and_a_zero_mask_each_draw_nothing() @test {
    DrawPasses passes = the_readings_from_five_draw_passes_over_three_spheres();

    assert(passes.drawn_with_shapes_raised == A_DRAWN_SPHERE_COUNT
        && passes.bounds_with_bounds_raised == A_DRAWN_SPHERE_COUNT,
        "the same world under a raised flag reaches both callbacks, which is what makes the zeros below readings");
    assert(passes.offered_under_the_default_flags == 0 && passes.drawn_under_the_default_flags == 0
        && passes.bounds_under_the_default_flags == 0,
        "a definition taken straight from default_debug_draw() reaches no callback at all");
    assert(passes.drawn_under_a_zero_mask == 0,
        "and a mask of zero draws nothing however the flags stand");
    assert(passes.offered_by_a_second_pass == 0 && passes.drawn_by_a_second_pass == A_DRAWN_SPHERE_COUNT,
        "box3d keeps what the first pass was answered and asks again for none of it");
}

fn void raising_draw_bounds_draws_one_box_per_shape_and_no_shape() @test {
    DrawPasses passes = the_readings_from_five_draw_passes_over_three_spheres();

    assert(passes.bounds_with_bounds_raised == A_DRAWN_SPHERE_COUNT,
        "one bounding box per shape, drawn for a shape the renderer has never been offered");
    assert(passes.shapes_with_bounds_raised == 0, "draw_bounds is not draw_shapes and draws no shape");

    float lowest_x = THREE_SPHERES[0].position.x - THREE_SPHERES[0].radius;
    float highest_x = THREE_SPHERES[2].position.x + THREE_SPHERES[2].radius;
    assert(passes.bounds_union.lower_bound.x <= lowest_x && passes.bounds_union.lower_bound.x > lowest_x - 1.0,
        "the boxes are the shapes' own, spanning the leftmost sphere and fattened no further than the broad phase does");
    assert(passes.bounds_union.upper_bound.x >= highest_x && passes.bounds_union.upper_bound.x < highest_x + 1.0,
        "and reaching past the rightmost by the same margin");
}

struct TwoSwitches {
    int offered_by_a_world_that_answers;
    int drawn_by_a_world_that_answers;
    int drawn_by_a_world_that_does_not;
    int bounds_by_a_world_that_does_not;
}

fn TwoSwitches the_readings_from_a_world_that_answers_for_its_shapes_and_one_that_does_not() {
    TwoSwitches switches;

    DrawTally answering;
    b3::WorldId answers = a_world_of_three_spheres(&answering, true);
    b3::DebugDraw over_the_answering = a_renderer_wired_to(&answering);
    over_the_answering.draw_shapes = true;
    over_the_answering.draw_bounds = true;
    answers.draw(&over_the_answering, ulong::max);
    answers.destroy();
    switches.offered_by_a_world_that_answers = answering.offered;
    switches.drawn_by_a_world_that_answers = answering.shapes_drawn;

    DrawTally silent;
    b3::WorldId does_not = a_world_of_three_spheres(&silent, false);
    b3::DebugDraw over_the_silent = a_renderer_wired_to(&silent);
    over_the_silent.draw_shapes = true;
    over_the_silent.draw_bounds = true;
    does_not.draw(&over_the_silent, ulong::max);
    does_not.destroy();
    switches.drawn_by_a_world_that_does_not = silent.shapes_drawn;
    switches.bounds_by_a_world_that_does_not = silent.bounds_drawn;

    return switches;
}

fn void drawing_shapes_takes_the_world_definition_switch_as_well_as_the_flag() @test {
    TwoSwitches switches = the_readings_from_a_world_that_answers_for_its_shapes_and_one_that_does_not();

    assert(switches.offered_by_a_world_that_answers == A_DRAWN_SPHERE_COUNT
        && switches.drawn_by_a_world_that_answers == A_DRAWN_SPHERE_COUNT,
        "a world built with create_debug_shape draws every shape under the same flag");
    assert(switches.drawn_by_a_world_that_does_not == 0,
        "and one built without it draws none of them, the flag notwithstanding");
    assert(switches.bounds_by_a_world_that_does_not == A_DRAWN_SPHERE_COUNT,
        "though that same pass drew all three bounding boxes, so it reached every shape");
}

fn void the_default_draw_definition_carries_every_callback_and_no_flag() @test {
    b3::DebugDraw taken = b3::default_debug_draw();
    b3::DebugDraw zeroed;
    float unit = b3::length_units_per_meter();

    assert(taken.draw_shape_fcn != null && taken.draw_segment_fcn != null && taken.draw_transform_fcn != null
        && taken.draw_point_fcn != null && taken.draw_sphere_fcn != null && taken.draw_capsule_fcn != null
        && taken.draw_bounds_fcn != null && taken.draw_box_fcn != null && taken.draw_string_fcn != null,
        "box3d fills all nine with empty bodies of its own");
    assert(zeroed.draw_shape_fcn == null && zeroed.draw_string_fcn == null,
        "where a definition the caller merely zeroed crashes on the first flag it raises");
    assert(taken.force_scale > 0.99 && taken.force_scale < 1.01
        && taken.joint_scale > 0.99 && taken.joint_scale < 1.01,
        "both scales start at one");
    assert(taken.drawing_bounds.lower_bound.x < -99.0 * unit && taken.drawing_bounds.lower_bound.x > -101.0 * unit
        && taken.drawing_bounds.upper_bound.y > 99.0 * unit && taken.drawing_bounds.upper_bound.y < 101.0 * unit,
        "and the culling box reaches a hundred length units either side of the origin");
    assert(!taken.draw_shapes && !taken.draw_joints && !taken.draw_joint_extras && !taken.draw_bounds
        && !taken.draw_mass && !taken.draw_sleep && !taken.draw_body_names && !taken.draw_contacts
        && !taken.draw_anchor_a && !taken.draw_graph_colors && !taken.draw_contact_features
        && !taken.draw_contact_normals && !taken.draw_contact_forces && !taken.draw_islands
        && taken.context == null,
        "every flag is false, so a caller who takes this and draws with it sees nothing");
}

const int A_DRAWN_RAY_COUNT = 2;
const int AN_OVERLAPS_HIT_COUNT = A_DROPPED_BODY_COUNT + 1;

struct DrawnQueries {
    int queries_at_the_frame;
    int segments_over_the_frame;
    int points_over_the_frame;
    int bounds_over_the_frame;
    int strings_over_the_frame;
    int segments_for_one_named_query;
    int bounds_for_one_named_query;
    int strings_for_a_selected_query;
    int segments_over_a_frame_that_issued_none;
}

// The busy session recording_test.c3 builds, replayed to the one frame that carries queries: two
// rays and one overlap, the first ray tagged. The player is destroyed before anything is
// asserted, because a live one holds the process-wide length unit at its recording's.
fn DrawnQueries the_readings_from_drawing_a_replayed_frames_queries() {
    DrawnQueries drawn;

    b3::Recording* recording = b3::create_recording(0);
    defer recording.destroy();

    DroppedPair pair = a_pair_dropped_on_a_slab();
    defer pair.world.destroy();

    if (!a_busy_session_recorded_into(pair, recording)) return drawn;

    b3::RecPlayer*? player = b3::create_player(recording.data(), 1);
    if (catch player) return drawn;

    DrawTally tally;
    b3::DebugDraw draw = a_renderer_wired_to(&tally);

    player.seek_frame(A_QUERY_FRAME);
    drawn.queries_at_the_frame = player.frame_query_count();
    player.draw_frame_queries(&draw, -1, -1);
    drawn.segments_over_the_frame = tally.segments_drawn;
    drawn.points_over_the_frame = tally.points_drawn;
    drawn.bounds_over_the_frame = tally.bounds_drawn;
    drawn.strings_over_the_frame = tally.strings_drawn;

    int segments_before_one = tally.segments_drawn;
    int bounds_before_one = tally.bounds_drawn;
    player.draw_frame_queries(&draw, 0, -1);
    drawn.segments_for_one_named_query = tally.segments_drawn - segments_before_one;
    drawn.bounds_for_one_named_query = tally.bounds_drawn - bounds_before_one;

    int strings_before_a_selection = tally.strings_drawn;
    player.draw_frame_queries(&draw, -1, 0);
    drawn.strings_for_a_selected_query = tally.strings_drawn - strings_before_a_selection;

    int segments_before_an_empty_frame = tally.segments_drawn;
    player.seek_frame(A_QUERY_FRAME + 1);
    player.draw_frame_queries(&draw, -1, -1);
    drawn.segments_over_a_frame_that_issued_none = tally.segments_drawn - segments_before_an_empty_frame;

    player.destroy();
    return drawn;
}

fn void a_replayed_frames_queries_draw_as_the_shapes_they_were_issued_over() @test {
    DrawnQueries drawn = the_readings_from_drawing_a_replayed_frames_queries();

    assert(drawn.queries_at_the_frame == A_RECORDED_QUERY_COUNT,
        "the frame the busy session issued its three queries ahead of");
    assert(drawn.segments_over_the_frame == 2 * A_DRAWN_RAY_COUNT && drawn.points_over_the_frame == A_DRAWN_RAY_COUNT,
        "each ray draws its span and then a point and a normal at the one place it landed");
    assert(drawn.bounds_over_the_frame == 1 + AN_OVERLAPS_HIT_COUNT,
        "and the overlap draws the box it was given plus one around every shape inside it");
    assert(drawn.segments_for_one_named_query == 2 && drawn.bounds_for_one_named_query == 0,
        "naming one query draws that query alone");
    assert(drawn.strings_over_the_frame == 0 && drawn.strings_for_a_selected_query == 1,
        "a label goes only to the selected query, and only while it carries a tag");
    assert(drawn.segments_over_a_frame_that_issued_none == 0,
        "a frame that re-issued no query draws nothing at all");
}
