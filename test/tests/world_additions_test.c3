module b3_test;

import b3;

fn void the_restitution_pass_count_is_read_back_and_clamped_into_range() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer b3::destroy_world(world);

    assert(world.restitution_iterations() == 2, "the default definition's count, exactly");

    world.set_restitution_iterations(7);
    int seven = world.restitution_iterations();
    world.set_restitution_iterations(b3::MAX_RESTITUTION_ITERATIONS + 40);
    int above = world.restitution_iterations();
    world.set_restitution_iterations(-5);
    int below = world.restitution_iterations();

    assert(seven == 7);
    assert(above == b3::MAX_RESTITUTION_ITERATIONS, "a count past the ceiling lands on it");
    assert(below == 0, "and a negative one on zero, neither being refused");
}

fn void restitution_propagation_is_off_by_default_and_toggles() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer b3::destroy_world(world);

    bool by_default = world.is_restitution_propagation_enabled();
    world.enable_restitution_propagation(true);
    bool enabled = world.is_restitution_propagation_enabled();
    world.enable_restitution_propagation(false);
    bool disabled = world.is_restitution_propagation_enabled();

    assert(!by_default);
    assert(enabled);
    assert(!disabled);
}

fn void the_isotropic_inertia_tolerance_is_read_back_and_clamped_into_zero_to_one() @test {
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer b3::destroy_world(world);

    float by_default = world.isotropic_inertia_tolerance();
    world.set_isotropic_inertia_tolerance(0.6);
    float set = world.isotropic_inertia_tolerance();
    world.set_isotropic_inertia_tolerance(3.0);
    float above = world.isotropic_inertia_tolerance();
    world.set_isotropic_inertia_tolerance(-2.0);
    float below = world.isotropic_inertia_tolerance();

    assert(by_default == (float)0.2, "the default definition's tolerance, exactly");
    assert(set == (float)0.6);
    assert(above == 1.0, "exact: the clamp writes its bound");
    assert(below == 0.0);
}

// Four boxes resting on static ground fill one graph colour, so the solver packs them into a
// single wide constraint whose size is the lane count's: the stack it peaks at tells the widths
// apart, and the peak is a high-water mark, so returning to eight lanes can only raise it.
fn int[2] the_solver_stack_peaks_with_and_then_without_the_fallback() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = b3::create_body(world, &ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::BoxHull slab = b3::make_box_hull(50.0, 1.0, 50.0);
    b3::create_hull_shape(ground, &shape_def, &slab.base)!!;

    b3::BoxHull cube = b3::make_cube_hull(0.5);
    for (int i = 0; i < 4; i++) {
        b3::BodyDef body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = { (float)(i * 2), 1.5, 0.0 };
        b3::BodyId body = b3::create_body(world, &body_def)!!;
        b3::create_hull_shape(body, &shape_def, &cube.base)!!;
    }

    int[2] peaks;
    world.enable_sse2_fallback(true);
    for (int i = 0; i < 3; i++) b3::world_step(world, 1.0f / 60.0f, 4);
    peaks[0] = world.counters().stack_used;

    world.enable_sse2_fallback(false);
    for (int i = 0; i < 3; i++) b3::world_step(world, 1.0f / 60.0f, 4);
    peaks[1] = world.counters().stack_used;
    return peaks;
}

fn void the_sse2_fallback_narrows_the_contact_solver_where_avx2_would_widen_it() @test {
    int[2] peaks = the_solver_stack_peaks_with_and_then_without_the_fallback();

    assert(peaks[0] > 0, "the steps used the stack at all");
    if (b3::is_avx2_available()) {
        assert(peaks[1] > peaks[0], "eight lanes take more stack for four contacts than four lanes");
    } else {
        assert(peaks[1] == peaks[0], "and without AVX2 there was only ever one width");
    }
}
