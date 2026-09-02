module b3_test;

import b3;

fn void the_world_types_match_the_c_layout() @test {
    assert(b3::Capacity::size == 20);
    assert(b3::WorldDef::size == 144);
    assert(b3::WorldDef::alignment == 8);
    assert(b3::Profile::size == 92);
    assert(b3::Counters::size == 200);
}

fn void the_default_definition_carries_the_values_the_c_side_sets() @test {
    b3::WorldDef def = b3::default_world_def();

    assert(def.gravity.x == 0.0);
    assert(def.gravity.y == -10.0);
    assert(def.gravity.z == 0.0);
    assert(def.restitution_threshold == 1.0);
    assert(def.hit_event_threshold == 1.0);
    assert(def.contact_hertz == 30.0);
    assert(def.contact_damping_ratio == 10.0);
    assert(def.contact_speed == 3.0);
    assert(def.maximum_linear_speed == 400.0);
    assert(def.enable_sleep);
    assert(def.enable_continuous);
}

fn void a_zeroed_definition_differs_from_a_default_one() @test {
    b3::WorldDef zeroed;
    b3::WorldDef def = b3::default_world_def();

    assert(zeroed.internal_value != def.internal_value);
}

fn void a_world_runs_and_then_reports_itself_invalid() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;

    assert(world.is_valid());
    for (int i = 0; i < 60; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    world.destroy();
    assert(!world.is_valid(), "a destroyed world must not report itself valid");
}

fn void a_definition_without_the_cookie_is_refused() @test {
    b3::WorldDef zeroed;

    if (catch err = b3::create_world(&zeroed)) {
        assert(err == b3::INVALID_DEFINITION);
        return;
    }
    assert(false, "box3d accepts a zeroed definition in a release build; the binding must not");
}

fn void a_world_releases_everything_it_allocated() @test {
    int baseline = b3::byte_count();

    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    assert(b3::byte_count() > baseline, "creating a world must allocate");

    world.destroy();
    assert(b3::byte_count() == baseline, "destroying it must release everything");
}

fn void the_world_limit_becomes_a_fault() @test {
    int baseline = b3::byte_count();
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId[b3::MAX_WORLDS] worlds;

    for (int i = 0; i < b3::MAX_WORLDS; i++) {
        worlds[i] = b3::create_world(&def)!!;
    }

    if (catch err = b3::create_world(&def)) {
        assert(err == b3::WORLD_LIMIT_REACHED);
    } else {
        assert(false, "the world past the limit must fault");
    }

    for (int i = 0; i < b3::MAX_WORLDS; i++) {
        worlds[i].destroy();
    }
    assert(b3::byte_count() == baseline);
}

fn void a_world_reports_the_properties_its_definition_asked_for() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    assert(world.gravity().y == -10.0);
    assert(world.restitution_threshold() == 1.0);
    assert(world.hit_event_threshold() == 1.0);
    assert(world.maximum_linear_speed() == 400.0);
    assert(world.is_sleeping_enabled());
    assert(world.is_continuous_enabled());
    assert(world.is_warm_starting_enabled());
    assert(world.worker_count() == 1);
    assert(world.awake_body_count() == 0);
}

fn void a_property_survives_a_round_trip() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    b3::Vec3 half_g = { 0.0, -1.5, 0.0 };
    world.set_gravity(half_g);
    assert(world.gravity().y == -1.5);

    world.enable_sleeping(false);
    assert(!world.is_sleeping_enabled());

    world.set_maximum_linear_speed(12.5);
    assert(world.maximum_linear_speed() == 12.5);
}

fn void an_empty_world_has_empty_counters() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    b3::Counters counters = world.counters();
    assert(counters.body_count == 0);
    assert(counters.shape_count == 0);
    assert(counters.contact_count == 0);
    assert(counters.joint_count == 0);

    // The world's capacity is a usage high-water mark, not the definition's request, so an empty
    // world reports zero across every field; a meaningful capacity test needs bodies.
    b3::Capacity capacity = world.max_capacity();
    assert(capacity.static_shape_count == 0);
    assert(capacity.dynamic_shape_count == 0);
    assert(capacity.static_body_count == 0);
    assert(capacity.dynamic_body_count == 0);
    assert(capacity.contact_count == 0);
}

fn void every_settable_property_survives_a_round_trip() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    // Both thresholds default to 1.0, so five pairwise-distinct values are what actually catches a
    // cross-wired setter/getter pair rather than passing by coincidence.
    world.set_restitution_threshold(2.5);
    world.set_hit_event_threshold(3.5);
    world.set_maximum_linear_speed(500.0);
    world.set_contact_recycle_distance(0.25);
    world.set_worker_count(2);

    assert(world.restitution_threshold() == 2.5);
    assert(world.hit_event_threshold() == 3.5);
    assert(world.maximum_linear_speed() == 500.0);
    assert(world.contact_recycle_distance() == 0.25);
    assert(world.worker_count() == 2);
}

fn void each_toggle_moves_only_its_own_flag() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    // All three default to true, so flipping one and checking only it changed is what actually
    // catches a cross-wired toggle rather than passing by coincidence.
    world.enable_continuous(false);
    assert(!world.is_continuous_enabled());
    assert(world.is_warm_starting_enabled());
    assert(world.is_sleeping_enabled());

    world.enable_warm_starting(false);
    assert(!world.is_continuous_enabled());
    assert(!world.is_warm_starting_enabled());
    assert(world.is_sleeping_enabled());

    world.enable_sleeping(false);
    assert(!world.is_continuous_enabled());
    assert(!world.is_warm_starting_enabled());
    assert(!world.is_sleeping_enabled());
}

fn void user_data_round_trips() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    int sentinel;
    world.set_user_data(&sentinel);
    assert(world.user_data() == &sentinel);
}

fn void the_world_count_tracks_live_worlds() @test {
    assert(b3::world_count() == 0);

    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    assert(b3::world_count() == 1);
    assert(b3::max_world_count() >= b3::world_count());

    world.destroy();
    assert(b3::world_count() == 0);
}

fn void a_stepped_world_reports_a_nonzero_profile() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    for (int i = 0; i < 10; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    b3::Profile profile = world.profile();
    assert(profile.step > 0.0);
}

fn void speculative_and_static_tree_rebuild_leave_the_world_valid() @test {
    int baseline = b3::byte_count();
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;

    world.enable_speculative(true);
    world.rebuild_static_tree();
    world.step(1.0f / 60.0f, 4);
    assert(world.is_valid());

    world.destroy();
    assert(b3::byte_count() == baseline);
}

fn void set_contact_tuning_takes_named_arguments() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    // set_contact_tuning cannot be round-tripped: box3d exposes no getter for these three values,
    // and observing an effect needs shapes in contact. The ordering (hertz, damping ratio, contact
    // speed) is verified against the header, not against a read-back; named arguments make the
    // parameter order load-bearing at the call site instead of a silent positional assumption.
    world.set_contact_tuning(hertz: 30.0, damping_ratio: 10.0, contact_speed: 3.0);
}

fn void the_world_bounds_enclose_the_shapes_it_holds() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::BoxHull hull = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeId shape = ground.create_hull_shape(&shape_def, &hull.base)!!;
    world.step(1.0f / 60.0f, 4);

    b3::Aabb bounds = world.bounds();
    b3::Aabb shape_bounds = shape.aabb();

    assert(bounds.lower_bound.x <= shape_bounds.lower_bound.x);
    assert(bounds.lower_bound.y <= shape_bounds.lower_bound.y);
    assert(bounds.lower_bound.z <= shape_bounds.lower_bound.z);
    assert(bounds.upper_bound.x >= shape_bounds.upper_bound.x);
    assert(bounds.upper_bound.y >= shape_bounds.upper_bound.y);
    assert(bounds.upper_bound.z >= shape_bounds.upper_bound.z);
}

int memory_stat_lines;
int memory_stat_lines_from_inside_a_step;
int pre_solves_that_asked_for_the_breakdown;
char[64] the_last_memory_stat_line;
b3::WorldId the_world_dumping_its_memory;

// Counts silently where base_hooks_test.c3's handler prints, the breakdown running to dozens of
// lines per call.
fn void record_a_memory_stat(ZString message) {
    memory_stat_lines++;
    String text = message.str_view();
    usz length = text.len < the_last_memory_stat_line.len - 1 ? text.len : the_last_memory_stat_line.len - 1;
    the_last_memory_stat_line[..] = 0;
    foreach (i, c : text[:length]) {
        the_last_memory_stat_line[i] = c;
    }
}

fn bool ask_for_the_breakdown_mid_step(
  b3::ShapeId shape_id_a,
  b3::ShapeId shape_id_b,
  b3::Pos point,
  b3::Vec3 normal,
  void* context,
) {
    pre_solves_that_asked_for_the_breakdown++;
    int before = memory_stat_lines;
    the_world_dumping_its_memory.dump_memory_stats();
    memory_stat_lines_from_inside_a_step += memory_stat_lines - before;
    return true;
}

struct MemoryBreakdown {
    int lines_for_an_empty_world;
    int lines_for_a_world_holding_bodies;
    bool the_last_line_was_the_total;
    int pre_solves;
    int lines_from_inside_a_step;
}

// The world contact_test.c3 keeps one contact alive in, which is what makes a pre-solve callback
// run at all — and a pre-solve callback is where a world is locked inside its own step.
fn MemoryBreakdown the_readings_from_dumping_a_worlds_memory() {
    MemoryBreakdown breakdown;
    b3::set_log_fn(&record_a_memory_stat);

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId empty = b3::create_world(&world_def)!!;
    memory_stat_lines = 0;
    empty.dump_memory_stats();
    breakdown.lines_for_an_empty_world = memory_stat_lines;
    empty.destroy();

    b3::BodyId ball;
    the_world_dumping_its_memory = a_world_that_keeps_solving_one_contact(&ball);
    memory_stat_lines = 0;
    the_world_dumping_its_memory.dump_memory_stats();
    breakdown.lines_for_a_world_holding_bodies = memory_stat_lines;
    breakdown.the_last_line_was_the_total =
      ((ZString)&the_last_memory_stat_line).str_view().starts_with("total:");

    the_world_dumping_its_memory.set_pre_solve_callback(&ask_for_the_breakdown_mid_step, null);
    b3::ShapeId[4] storage;
    foreach (shape : ball.shapes(storage[..])) {
        shape.enable_pre_solve_events(true);
    }
    memory_stat_lines_from_inside_a_step = 0;
    pre_solves_that_asked_for_the_breakdown = 0;
    for (int i = 0; i < 120; i++) {
        the_world_dumping_its_memory.step(1.0f / 60.0f, 4);
    }
    breakdown.pre_solves = pre_solves_that_asked_for_the_breakdown;
    breakdown.lines_from_inside_a_step = memory_stat_lines_from_inside_a_step;

    the_world_dumping_its_memory.destroy();
    b3::set_log_fn(&capturing_log);
    return breakdown;
}

fn void a_worlds_memory_breakdown_goes_to_the_log_handler_and_never_from_inside_a_step() @test {
    MemoryBreakdown breakdown = the_readings_from_dumping_a_worlds_memory();

    assert(breakdown.lines_for_an_empty_world > 20 && breakdown.lines_for_an_empty_world < 200,
        "a few dozen lines even for a world holding nothing");
    assert(breakdown.lines_for_a_world_holding_bodies == breakdown.lines_for_an_empty_world,
        "and the same number for a world holding bodies: one line per pool, never one per thing in it");
    assert(breakdown.the_last_line_was_the_total, "the last of them carrying the total");
    assert(breakdown.pre_solves > 0,
        "the world was stepped with a callback that asks for the breakdown from inside the step");
    assert(breakdown.lines_from_inside_a_step == 0, "and a world locked in its own step writes nothing");
}
