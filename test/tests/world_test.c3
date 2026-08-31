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
