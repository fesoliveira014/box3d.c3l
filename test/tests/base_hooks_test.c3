module b3_test;

import b3;
import std::io;

int allocation_count;
int free_count;
long allocated_bytes;
char[512] last_log_message;
int log_message_count;

fn void* counting_alloc(int size, int alignment) {
    allocation_count++;
    allocated_bytes += size;
    return mem::malloc_aligned((sz)size, (sz)alignment);
}

fn void counting_free(void* memory) {
    free_count++;
    mem::free_aligned(memory);
}

<*
 Stands in for box3d's own default handler, which is static and cannot be reinstalled once
 replaced, so this one keeps printing what the default printed.
*>
fn void capturing_log(ZString message) {
    log_message_count++;
    String text = message.str_view();
    usz length = text.len < last_log_message.len - 1 ? text.len : last_log_message.len - 1;
    last_log_message[..] = 0;
    foreach (i, c : text[:length]) {
        last_log_message[i] = c;
    }
    io::printfn("Box3D: %s", text);
}

<*
 Stands in for box3d's static default assert handler. Returning non-zero asks box3d to break to
 the debugger, which is what the default does.
*>
fn int reporting_assert(ZString condition, ZString file_name, int line_number) {
    io::printfn("BOX3D ASSERTION: %s, %s, line %d", condition.str_view(), file_name.str_view(), line_number);
    return 1;
}

<*
 The allocator is process-wide, so it is restored before any assertion runs — a failing assertion
 skips the rest of the function and would otherwise leave every later test allocating through it.
*>
fn void a_custom_allocator_serves_every_allocation_a_world_makes() @test {
    int baseline = b3::byte_count();
    allocation_count = 0;
    free_count = 0;
    allocated_bytes = 0;

    b3::set_allocator(&counting_alloc, &counting_free);

    bool simulated = false;
    b3::WorldDef world_def = b3::default_world_def();
    if (try world = b3::create_world(&world_def)) {
        b3::BodyDef body_def = b3::default_body_def();
        body_def.type = b3::BodyType.DYNAMIC;
        body_def.position = { 0.0, 10.0, 0.0 };
        if (try body = b3::create_body(world, &body_def)) {
            b3::ShapeDef shape_def = b3::default_shape_def();
            b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
            if (try shape = b3::create_sphere_shape(body, &shape_def, &sphere)) {
                for (int i = 0; i < 30; i++) {
                    b3::world_step(world, 1.0f / 60.0f, 4);
                }
                simulated = shape.is_valid();
            }
        }
        b3::destroy_world(world);
    }

    b3::set_allocator(null, null);

    int allocations = allocation_count;
    int frees = free_count;
    long bytes = allocated_bytes;

    assert(simulated, "the world, body and shape must all be built through the custom allocator");
    assert(allocations > 0, "box3d must route its allocations through an installed allocator");
    assert(frees == allocations, "a destroyed world must return every block it took");
    assert(bytes > 0);
    assert(b3::byte_count() == baseline, "box3d counts bytes for a custom allocator too");
}

<*
 The world limit is the one diagnostic this package can provoke in a release build, so it is what
 proves a log handler is really called with a message box3d formatted.
*>
fn void an_installed_log_handler_receives_the_diagnostics_box3d_emits() @test {
    int baseline = b3::byte_count();
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId[b3::MAX_WORLDS] worlds;

    for (int i = 0; i < b3::MAX_WORLDS; i++) {
        worlds[i] = b3::create_world(&def)!!;
    }

    log_message_count = 0;
    last_log_message[..] = 0;
    b3::set_log_fn(&capturing_log);

    if (catch err = b3::create_world(&def)) {
        assert(err == b3::WORLD_LIMIT_REACHED);
    } else {
        assert(false, "the world past the limit must fault");
    }

    for (int i = 0; i < b3::MAX_WORLDS; i++) {
        b3::destroy_world(worlds[i]);
    }

    String logged = ((ZString)&last_log_message).str_view();
    assert(log_message_count == 1, "exceeding the world limit emits exactly one diagnostic");
    assert(logged.contains("128"), "the diagnostic names the limit that was hit");
    assert(b3::byte_count() == baseline);
}

<*
 box3d's assertions are compiled out of this Release build, so the handler can never fire. What
 this covers is the setter and the handler's declared signature, not a call through it.
*>
fn void an_assert_handler_can_be_installed() @test {
    b3::set_assert_fn(&reporting_assert);

    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!!;
    defer b3::destroy_world(world);

    assert(world.is_valid(), "a world builds normally with an assert handler installed");
}

fn void the_timing_hooks_measure_elapsed_milliseconds() @test {
    ulong start = b3::ticks();
    b3::sleep(2);
    b3::yield();
    float elapsed = b3::milliseconds(start);

    assert(elapsed >= 1.0, "two milliseconds of sleep must register as at least one");
    assert(elapsed < 2000.0, "a two millisecond sleep must not read as two seconds");
}

fn void the_resetting_timer_moves_the_stamp_it_measured_from() @test {
    ulong stamp = b3::ticks();
    ulong started_at = stamp;
    b3::sleep(2);
    float elapsed = b3::milliseconds_and_reset(&stamp);
    float since_that_call = b3::milliseconds_and_reset(&stamp);

    assert(elapsed >= 1.0 && elapsed < 2000.0, "two milliseconds of sleep register as at least one");
    assert(stamp > started_at, "and the stamp is advanced to the reading, not left where it began");
    assert(since_that_call >= 0.0 && since_that_call < elapsed,
        "so the second call measures the gap since the first rather than the sleep again");
}

<*
 Both settings are process-wide and the length unit scales box3d's slop and margins, so each is
 restored before the assertions run.
*>
fn void the_stall_threshold_round_trips() @test {
    float original = b3::stall_threshold();
    b3::set_stall_threshold(0.25);
    float updated = b3::stall_threshold();
    b3::set_stall_threshold(original);
    float restored = b3::stall_threshold();

    assert(updated == 0.25);
    assert(restored == original);
}

fn void the_length_unit_scale_round_trips() @test {
    float original = b3::length_units_per_meter();
    b3::set_length_units_per_meter(2.0);
    float updated = b3::length_units_per_meter();
    b3::set_length_units_per_meter(original);
    float restored = b3::length_units_per_meter();

    assert(original == 1.0, "the package is built for metres");
    assert(updated == 2.0);
    assert(restored == 1.0);
}
