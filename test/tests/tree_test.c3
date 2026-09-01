module b3_test;

import b3;

fn void the_tree_types_match_the_c_layout() @test {
    assert(b3::TreeNodeChildren::size == 8);
    assert(b3::TreeNodeChildren::alignment == 4);
    assert(b3::TreeNode::size == 48);
    assert(b3::TreeNode::alignment == 8);
    assert(b3::DynamicTree::size == 80);
    assert(b3::DynamicTree::alignment == 8);
}

// The byte count is read while the tree is alive and again by the caller, after the helper's
// defer has run. Nothing here asserts: C3 skips defer on a panicking assertion, and a stranded
// node pool sits on byte_count() for the rest of the run exactly as a stranded world does.
fn int[2] the_byte_count_around_a_live_tree() {
    int baseline = b3::byte_count();

    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    return { baseline, b3::byte_count() };
}

fn void a_destroyed_tree_gives_back_every_byte_it_took() @test {
    int[2] around = the_byte_count_around_a_live_tree();

    assert(around[1] > around[0], "the node pool box3d allocates for a tree must show in the count");
    assert(b3::byte_count() == around[0], "and destroying the tree must give every byte back");
}

fn int[3] the_byte_count_around_a_tree_destroyed_twice() {
    int baseline = b3::byte_count();

    b3::DynamicTree tree = b3::create_tree(16);
    tree.destroy();
    int after_one = b3::byte_count();
    tree.destroy();

    return { baseline, after_one, b3::byte_count() };
}

// Destroying zeroes the struct, so the second call has a null pool and a capacity of 0 to free.
// That makes a repeated destroy harmless on one struct and says nothing about a copy of it,
// which owns the same pool and frees it a second time for real.
fn void destroying_a_tree_a_second_time_frees_nothing() @test {
    int[3] around = the_byte_count_around_a_tree_destroyed_twice();

    assert(around[1] == around[0], "the first destroy gives the pool back");
    assert(around[2] == around[1], "and the second finds nothing left to give");
}

struct TreeCapacities {
    int negative;
    int under_the_floor;
    int at_the_floor;
    int over_the_floor;
}

fn TreeCapacities the_node_capacities_of_four_trees() {
    b3::DynamicTree nonsense = b3::create_tree(-1);
    defer nonsense.destroy();

    b3::DynamicTree small = b3::create_tree(8);
    defer small.destroy();

    b3::DynamicTree floored = b3::create_tree(16);
    defer floored.destroy();

    b3::DynamicTree large = b3::create_tree(64);
    defer large.destroy();

    return {
        .negative = nonsense.node_capacity,
        .under_the_floor = small.node_capacity,
        .at_the_floor = floored.node_capacity,
        .over_the_floor = large.node_capacity,
    };
}

// box3d raises any capacity below 16 to 16 and sizes the pool at twice it less one, so a
// contrast between capacities has to be drawn above the floor: 8 and 16 are the same request.
// The pool sizes are exact arithmetic on the argument, not measurements, so they are compared
// exactly.
fn void a_capacity_under_the_floor_builds_the_floor_s_pool() @test {
    TreeCapacities capacities = the_node_capacities_of_four_trees();

    assert(capacities.under_the_floor == 31, "sixteen leaves and the fifteen nodes joining them");
    assert(capacities.at_the_floor == capacities.under_the_floor, "which is what a request for 8 asks for");
    assert(capacities.negative == capacities.under_the_floor, "and what a negative request asks for too");
    assert(capacities.over_the_floor == 127, "while a request above the floor is sized as itself");
}

struct ProxyReadings {
    int proxy_count;
    ulong[2] user_data;
    b3::Aabb[2] box;
}

fn ProxyReadings the_readings_of_two_proxies() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    b3::Aabb[2] boxes = {
        {
            .lower_bound = { -1.5, -2.25, -3.75 },
            .upper_bound = { 4.5, 5.25, 6.75 },
        },
        {
            .lower_bound = { 10.5, 11.25, 12.75 },
            .upper_bound = { 20.5, 21.25, 22.75 },
        },
    };

    // The first carries a byte pattern that a truncation to four bytes would change; the second
    // carries every bit set, which is also the category bits every query mask matches.
    ulong[2] payloads = { 0x0123456789ABCDEF, ulong::max };

    ProxyReadings read;
    for (int i = 0; i < 2; i++) {
        int proxy = tree.create_proxy(
          aabb: boxes[i],
          category_bits: ulong::max,
          user_data: payloads[i],
        );
        if (try user_data = tree.user_data(proxy)) read.user_data[i] = user_data;
        if (try box = tree.aabb(proxy)) read.box[i] = box;
    }
    read.proxy_count = tree.proxy_count;

    return read;
}

// The two proxies differ in every component of their boxes as well as in their user data, so an
// accessor answering from the wrong node, or reading the union's other arm, cannot pass.
fn void a_proxy_reports_the_box_and_the_user_data_it_was_created_with() @test {
    ProxyReadings read = the_readings_of_two_proxies();

    assert(read.proxy_count == 2, "each proxy read back through the id its creation returned");
    assert(read.user_data[0] == 0x0123456789ABCDEF, "all eight bytes of it, not the low four");
    assert(read.user_data[1] == ulong::max);

    assert(read.box[0].lower_bound.x > -1.51 && read.box[0].lower_bound.x < -1.49);
    assert(read.box[0].lower_bound.y > -2.26 && read.box[0].lower_bound.y < -2.24);
    assert(read.box[0].lower_bound.z > -3.76 && read.box[0].lower_bound.z < -3.74);
    assert(read.box[0].upper_bound.x > 4.49 && read.box[0].upper_bound.x < 4.51);
    assert(read.box[0].upper_bound.y > 5.24 && read.box[0].upper_bound.y < 5.26);
    assert(read.box[0].upper_bound.z > 6.74 && read.box[0].upper_bound.z < 6.76,
        "every component of the first box, so a transposed or swapped pair would show");

    assert(read.box[1].lower_bound.x > 10.49 && read.box[1].lower_bound.x < 10.51);
    assert(read.box[1].upper_bound.z > 22.74 && read.box[1].upper_bound.z < 22.76,
        "and the second box is its own, not a second reading of the first");
}

struct IdGuards {
    fault user_data_negative;
    fault user_data_past_the_end;
    fault aabb_negative;
    fault aabb_past_the_end;
    fault last_slot;
}

fn IdGuards the_guards_on_a_proxy_id() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { 0.0, 0.0, 0.0 }, .upper_bound = { 1.0, 1.0, 1.0 } },
      category_bits: ulong::max,
      user_data: 7,
    );

    IdGuards guards;
    if (catch err = tree.user_data(-1)) guards.user_data_negative = err;
    if (catch err = tree.user_data(tree.node_capacity)) guards.user_data_past_the_end = err;
    if (catch err = tree.aabb(-1)) guards.aabb_negative = err;
    if (catch err = tree.aabb(tree.node_capacity)) guards.aabb_past_the_end = err;
    if (catch err = tree.user_data(tree.node_capacity - 1)) guards.last_slot = err;

    return guards;
}

// box3d's own readers are inline in the header and bound the id with nothing at all, so both
// ends run off the node array there — the negative end into memory before it. What the guard
// restores is the array's bound and only that: the last slot holds no proxy and is read all the
// same, because the tree carries nothing that could tell a caller so.
fn void a_proxy_id_outside_the_node_array_is_refused_at_both_ends() @test {
    IdGuards guards = the_guards_on_a_proxy_id();
    fault none;

    assert(guards.user_data_negative == b3::INDEX_OUT_OF_RANGE, "a negative id names no node");
    assert(guards.user_data_past_the_end == b3::INDEX_OUT_OF_RANGE, "and neither does the one past the last");
    assert(guards.aabb_negative == b3::INDEX_OUT_OF_RANGE);
    assert(guards.aabb_past_the_end == b3::INDEX_OUT_OF_RANGE);
    assert(guards.last_slot == none, "while an id in range is read whether or not a proxy holds it");
}
