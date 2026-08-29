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

    b3::Capacity capacity = world.max_capacity();
    assert(capacity.static_body_count >= 0);
}
