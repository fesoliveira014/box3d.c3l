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

fn int[4] the_byte_count_around_a_tree_destroyed_twice() {
    int baseline = b3::byte_count();

    b3::DynamicTree tree = b3::create_tree(16);
    int while_alive = b3::byte_count();
    tree.destroy();
    int after_one = b3::byte_count();
    tree.destroy();

    return { baseline, while_alive, after_one, b3::byte_count() };
}

// Destroying zeroes the struct, so the second call has a null pool and a capacity of 0 to free.
// That makes a repeated destroy harmless on one struct and says nothing about a copy of it,
// which owns the same pool and frees it a second time for real.
fn void destroying_a_tree_a_second_time_frees_nothing() @test {
    int[4] around = the_byte_count_around_a_tree_destroyed_twice();

    assert(around[1] > around[0], "the tree took a pool while it was alive");
    assert(around[2] == around[0], "the first destroy gives every byte of it back");
    assert(around[3] == around[2], "and the second finds nothing left to give");
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
        )!!;
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
    fault aabb_last_slot;
    fault live_proxy;
}

fn IdGuards the_guards_on_a_proxy_id() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    int live = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { 0.0, 0.0, 0.0 }, .upper_bound = { 1.0, 1.0, 1.0 } },
      category_bits: ulong::max,
      user_data: 7,
    )!!;

    IdGuards guards;
    if (catch err = tree.user_data(-1)) guards.user_data_negative = err;
    if (catch err = tree.user_data(tree.node_capacity)) guards.user_data_past_the_end = err;
    if (catch err = tree.aabb(-1)) guards.aabb_negative = err;
    if (catch err = tree.aabb(tree.node_capacity)) guards.aabb_past_the_end = err;
    if (catch err = tree.user_data(tree.node_capacity - 1)) guards.last_slot = err;
    if (catch err = tree.aabb(tree.node_capacity - 1)) guards.aabb_last_slot = err;
    if (catch err = tree.user_data(live)) guards.live_proxy = err;

    return guards;
}

// box3d's own readers are inline in the header and bound the id with nothing at all, so both ends
// run off the node array there — the negative end into memory before it. The last slot is in range
// and holds no proxy, which is the case the leaf bit separates from a live one.
fn void a_proxy_id_outside_the_node_array_is_refused_at_both_ends() @test {
    IdGuards guards = the_guards_on_a_proxy_id();
    fault none;

    assert(guards.user_data_negative == b3::INDEX_OUT_OF_RANGE, "a negative id names no node");
    assert(guards.user_data_past_the_end == b3::INDEX_OUT_OF_RANGE, "and neither does the one past the last");
    assert(guards.aabb_negative == b3::INDEX_OUT_OF_RANGE);
    assert(guards.aabb_past_the_end == b3::INDEX_OUT_OF_RANGE);
    assert(guards.last_slot == b3::NOT_A_PROXY, "an id in range holding no proxy is refused, not read");
    assert(guards.aabb_last_slot == b3::NOT_A_PROXY);
    assert(guards.live_proxy == none, "while the one that does holds is read");
}

struct BoxGuards {
    fault inverted;
    fault infinite;
    fault not_a_number;
    fault zero;
}

fn BoxGuards the_boxes_a_proxy_is_refused_for() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    float infinity = (float)1.0e40;
    float nan = infinity - infinity;

    BoxGuards guards;
    if (catch err = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { 5.0, 5.0, 5.0 }, .upper_bound = { -5.0, -5.0, -5.0 } },
      category_bits: 1,
      user_data: 1,
    )) guards.inverted = err;
    if (catch err = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { -infinity, 0.0, 0.0 }, .upper_bound = { infinity, 1.0, 1.0 } },
      category_bits: 1,
      user_data: 2,
    )) guards.infinite = err;
    if (catch err = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { nan, 0.0, 0.0 }, .upper_bound = { 1.0, 1.0, 1.0 } },
      category_bits: 1,
      user_data: 3,
    )) guards.not_a_number = err;
    if (catch err = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { 0.0, 0.0, 0.0 }, .upper_bound = { 0.0, 0.0, 0.0 } },
      category_bits: 1,
      user_data: 4,
    )) guards.zero = err;

    return guards;
}

// box3d asserts the box and this build compiles the assertion out, so an invalid one reaches the
// tree. None of the three is inert: an inverted box answers no query, an infinite one answers all
// of them, and a NaN one answers every query while box3d's min and max let an ancestor swallow a
// real bound. A zero box is a legitimate point and stays accepted.
fn void a_proxy_box_that_box3d_would_assert_against_is_refused() @test {
    BoxGuards guards = the_boxes_a_proxy_is_refused_for();
    fault none;

    assert(guards.inverted == b3::INVALID_DEFINITION, "lower above upper names no volume");
    assert(guards.infinite == b3::INVALID_DEFINITION, "and an unbounded one would answer every query");
    assert(guards.not_a_number == b3::INVALID_DEFINITION, "and a NaN would eat a bound above it");
    assert(guards.zero == none, "while a zero box is a point, which the tree holds happily");
}

struct MutatorReadings {
    ulong category_at_creation;
    ulong category_after_setting;
    b3::Aabb moved;
    b3::Aabb enlarged;
    fault move_refused;
    fault enlarge_refused;
    fault set_refused;
}

fn MutatorReadings the_readings_after_moving_and_enlarging_one_proxy() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    b3::Aabb elsewhere = {
        .lower_bound = { 6.5, -3.25, 12.75 },
        .upper_bound = { 8.5, -1.25, 14.75 },
    };
    b3::Aabb wider = {
        .lower_bound = { 5.5, -4.25, 11.75 },
        .upper_bound = { 9.5, -0.25, 15.75 },
    };

    int proxy = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { -1.0, -1.0, -1.0 }, .upper_bound = { 1.0, 1.0, 1.0 } },
      category_bits: 2,
      user_data: 11,
    )!!;

    MutatorReadings read;
    if (try bits = tree.category_bits(proxy)) read.category_at_creation = bits;

    if (catch err = tree.move_proxy(proxy, elsewhere)) read.move_refused = err;
    if (try box = tree.aabb(proxy)) read.moved = box;

    if (catch err = tree.enlarge_proxy(proxy, wider)) read.enlarge_refused = err;
    if (try box = tree.aabb(proxy)) read.enlarged = box;

    if (catch err = tree.set_category_bits(proxy, 0xF0)) read.set_refused = err;
    if (try bits = tree.category_bits(proxy)) read.category_after_setting = bits;

    return read;
}

// The two boxes share no component, so a mutator that wrote one field and left the rest, or that
// answered from the box the proxy was created with, cannot pass.
fn void a_proxy_takes_the_box_a_move_and_an_enlarge_give_it() @test {
    MutatorReadings read = the_readings_after_moving_and_enlarging_one_proxy();
    fault none;

    assert(read.move_refused == none, "a finite box is a box the tree accepts");
    assert(read.moved.lower_bound.x > 6.49 && read.moved.lower_bound.x < 6.51);
    assert(read.moved.lower_bound.y > -3.26 && read.moved.lower_bound.y < -3.24);
    assert(read.moved.lower_bound.z > 12.74 && read.moved.lower_bound.z < 12.76);
    assert(read.moved.upper_bound.x > 8.49 && read.moved.upper_bound.x < 8.51);
    assert(read.moved.upper_bound.y > -1.26 && read.moved.upper_bound.y < -1.24);
    assert(read.moved.upper_bound.z > 14.74 && read.moved.upper_bound.z < 14.76,
        "every component of the box the move was given");

    assert(read.enlarge_refused == none);
    assert(read.enlarged.lower_bound.x > 5.49 && read.enlarged.lower_bound.x < 5.51);
    assert(read.enlarged.lower_bound.y > -4.26 && read.enlarged.lower_bound.y < -4.24);
    assert(read.enlarged.lower_bound.z > 11.74 && read.enlarged.lower_bound.z < 11.76);
    assert(read.enlarged.upper_bound.x > 9.49 && read.enlarged.upper_bound.x < 9.51);
    assert(read.enlarged.upper_bound.y > -0.26 && read.enlarged.upper_bound.y < -0.24);
    assert(read.enlarged.upper_bound.z > 15.74 && read.enlarged.upper_bound.z < 15.76,
        "and every component of the wider one the enlarge was given");
}

// The category bits are a mask read straight back out, so they are compared exactly.
fn void a_proxy_keeps_the_category_bits_it_is_given() @test {
    MutatorReadings read = the_readings_after_moving_and_enlarging_one_proxy();
    fault none;

    assert(read.category_at_creation == 2, "the bits the proxy was created with");
    assert(read.set_refused == none);
    assert(read.category_after_setting == 0xF0, "and the ones set over them afterwards");
}

struct ProxyLifetimes {
    int[3] proxy_count;
    int destroyed_id;
    int reused_id;
    fault destroy_refused;
    fault repeated_destroy;
    fault read_of_a_destroyed_proxy;
}

fn ProxyLifetimes the_counts_around_a_destroyed_proxy() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    b3::Aabb unit = {
        .lower_bound = { -1.0, -1.0, -1.0 },
        .upper_bound = { 1.0, 1.0, 1.0 },
    };

    ProxyLifetimes lifetimes;
    int[4] created;
    for (int i = 0; i < 4; i++) {
        created[i] = tree.create_proxy(
          aabb: unit,
          category_bits: 1,
          user_data: (ulong)i,
        )!!;
    }
    lifetimes.proxy_count[0] = tree.proxy_count;

    lifetimes.destroyed_id = created[1];
    if (catch err = tree.destroy_proxy(created[1])) lifetimes.destroy_refused = err;
    lifetimes.proxy_count[1] = tree.proxy_count;
    if (catch err = tree.destroy_proxy(created[1])) lifetimes.repeated_destroy = err;
    if (catch err = tree.user_data(created[1])) lifetimes.read_of_a_destroyed_proxy = err;

    lifetimes.reused_id = tree.create_proxy(
      aabb: unit,
      category_bits: 1,
      user_data: 99,
    )!!;
    lifetimes.proxy_count[2] = tree.proxy_count;

    return lifetimes;
}

// A proxy id carries no generation counter, so the slot a destroy frees is handed straight back
// out and a stale id then names whatever took it. Nothing in the binding can catch that; what it
// does catch is the window before the reuse, where the slot holds no proxy at all.
fn void a_destroyed_proxy_gives_its_id_to_the_next_one() @test {
    ProxyLifetimes lifetimes = the_counts_around_a_destroyed_proxy();
    fault none;

    assert(lifetimes.destroy_refused == none, "a live proxy is a proxy the tree will destroy");
    assert(lifetimes.proxy_count[0] == 4, "four created");
    assert(lifetimes.proxy_count[1] == 3, "three left after one is destroyed");
    assert(lifetimes.proxy_count[2] == 4, "and four again after another is created");
    assert(lifetimes.repeated_destroy == b3::NOT_A_PROXY, "destroying the same id twice is refused");
    assert(lifetimes.read_of_a_destroyed_proxy == b3::NOT_A_PROXY, "and so is reading it");
    assert(lifetimes.reused_id == lifetimes.destroyed_id, "the freed slot goes to the next proxy");
}

struct TreeReports {
    int[3] height;
    float[3] area_ratio;
    int[3] proxy_count;
    b3::Aabb empty_bounds;
    b3::Aabb two_proxy_bounds;
}

fn TreeReports the_reports_of_a_growing_tree() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    TreeReports reports;
    reports.height[0] = tree.height();
    reports.area_ratio[0] = tree.area_ratio();
    reports.proxy_count[0] = tree.proxy_count;
    reports.empty_bounds = tree.root_bounds();

    for (int i = 0; i < 6; i++) {
        tree.create_proxy(
          aabb: (b3::Aabb){
            .lower_bound = { i * 4.0f - 1.0f, -1.0, -1.0 },
            .upper_bound = { i * 4.0f + 1.0f, 1.0, 1.0 },
          },
          category_bits: 1ul << i,
          user_data: (ulong)i,
        )!!;
        if (i == 1) {
            reports.height[1] = tree.height();
            reports.area_ratio[1] = tree.area_ratio();
            reports.proxy_count[1] = tree.proxy_count;
            reports.two_proxy_bounds = tree.root_bounds();
        }
    }
    reports.height[2] = tree.height();
    reports.area_ratio[2] = tree.area_ratio();
    reports.proxy_count[2] = tree.proxy_count;

    return reports;
}

// Heights are the levels below the root, and both values here are the fewest a binary tree over
// that many leaves can have, so a tree that inserted without rebalancing would report more.
fn void the_height_is_the_depth_of_the_tree_below_its_root() @test {
    TreeReports reports = the_reports_of_a_growing_tree();

    assert(reports.proxy_count[0] == 0 && reports.proxy_count[1] == 2 && reports.proxy_count[2] == 6);
    assert(reports.height[0] == 0, "an empty tree has no root to measure from");
    assert(reports.height[1] == 1, "two leaves under one internal node");
    assert(reports.height[2] == 3, "and six leaves three levels below the root");
}

// The ratio sums the internal nodes and skips both the leaves and the root, so a tree whose only
// internal node is its root has nothing to sum and reports the same 0 an empty tree does. The
// value for six is left as a band: which leaves box3d pairs is its own business, and the contrast
// that matters is between having internal nodes to sum and having none.
fn void the_area_ratio_is_zero_until_the_tree_has_a_node_below_its_root() @test {
    TreeReports reports = the_reports_of_a_growing_tree();

    assert(reports.area_ratio[0] == 0.0, "no root, nothing to sum");
    assert(reports.area_ratio[1] == 0.0, "a root over two leaves, still nothing to sum");
    assert(reports.area_ratio[2] > 1.0 && reports.area_ratio[2] < 3.0,
        "and four internal nodes below the root, each with a perimeter of its own");
}

// An empty tree reports the type's zero rather than a marked box, which is why the count is
// asserted alongside it: the bounds alone cannot tell an empty tree from a point at the origin.
fn void the_root_bounds_hold_every_proxy_and_an_empty_tree_reports_zero() @test {
    TreeReports reports = the_reports_of_a_growing_tree();

    assert(reports.proxy_count[0] == 0);
    assert(reports.empty_bounds.lower_bound.x == 0.0 && reports.empty_bounds.upper_bound.x == 0.0);
    assert(reports.empty_bounds.lower_bound.y == 0.0 && reports.empty_bounds.upper_bound.y == 0.0);
    assert(reports.empty_bounds.lower_bound.z == 0.0 && reports.empty_bounds.upper_bound.z == 0.0,
        "the zero box, indistinguishable from a degenerate one at the origin");

    assert(reports.two_proxy_bounds.lower_bound.x > -1.01 && reports.two_proxy_bounds.lower_bound.x < -0.99);
    assert(reports.two_proxy_bounds.lower_bound.y > -1.01 && reports.two_proxy_bounds.lower_bound.y < -0.99);
    assert(reports.two_proxy_bounds.lower_bound.z > -1.01 && reports.two_proxy_bounds.lower_bound.z < -0.99);
    assert(reports.two_proxy_bounds.upper_bound.x > 4.99 && reports.two_proxy_bounds.upper_bound.x < 5.01,
        "the far side of the second proxy, four along from the first");
    assert(reports.two_proxy_bounds.upper_bound.y > 0.99 && reports.two_proxy_bounds.upper_bound.y < 1.01);
    assert(reports.two_proxy_bounds.upper_bound.z > 0.99 && reports.two_proxy_bounds.upper_bound.z < 1.01);
}

struct RebuildCounts {
    int baseline;
    int live;
    int after_an_empty_rebuild;
    int after_a_rebuild;
    int tree_bytes;
    int[3] sorted;
    int[2] height;
}

fn RebuildCounts the_counts_around_a_rebuild() {
    RebuildCounts counts;
    counts.baseline = b3::byte_count();

    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    counts.live = b3::byte_count();
    counts.tree_bytes = tree.byte_count();

    counts.sorted[0] = tree.rebuild(false);
    counts.after_an_empty_rebuild = b3::byte_count();

    for (int i = 0; i < 6; i++) {
        tree.create_proxy(
          aabb: (b3::Aabb){
            .lower_bound = { i * 4.0f - 1.0f, -1.0, -1.0 },
            .upper_bound = { i * 4.0f + 1.0f, 1.0, 1.0 },
          },
          category_bits: 1ul << i,
          user_data: (ulong)i,
        )!!;
    }
    counts.height[0] = tree.height();
    counts.sorted[1] = tree.rebuild(false);
    counts.sorted[2] = tree.rebuild(true);
    counts.height[1] = tree.height();
    counts.after_a_rebuild = b3::byte_count();

    return counts;
}

// A full build sorts every leaf; a partial one keeps the subtrees box3d has not marked enlarged
// and sorts fewer, which is the whole point of asking for one. Neither leaves the tree at a
// different height than it found it.
fn void a_full_rebuild_sorts_every_proxy_and_a_partial_one_sorts_fewer() @test {
    RebuildCounts counts = the_counts_around_a_rebuild();

    assert(counts.sorted[0] == 0, "a tree with no proxies has nothing to sort");
    assert(counts.sorted[2] == 6, "a full build sorts one box per proxy");
    assert(counts.sorted[1] >= 1 && counts.sorted[1] < counts.sorted[2], "and a partial build fewer");
    assert(counts.height[0] == 3 && counts.height[1] == 3, "and neither changes the height it found");
}

// The rebuild arrays are box3d's, allocated on the first rebuild that needs them and released
// with the tree. The tree's own count is arithmetic over its capacities and includes the caller's
// by-value struct, which never was on the heap — the two numbers differ by exactly that struct.
fn void a_tree_counts_its_own_struct_and_the_rebuild_arrays_it_takes() @test {
    RebuildCounts counts = the_counts_around_a_rebuild();

    assert(counts.live > counts.baseline, "the node pool is on the shared count");
    assert(counts.tree_bytes == counts.live - counts.baseline + (int)b3::DynamicTree::size,
        "and the tree counts that pool plus the struct the caller holds");
    assert(counts.after_an_empty_rebuild == counts.live, "a rebuild with nothing to sort takes nothing");
    assert(counts.after_a_rebuild > counts.live, "while one with proxies takes the rebuild arrays");
    assert(b3::byte_count() == counts.baseline, "and destroying the tree gives all of it back");
}

struct ProxyIdGuards {
    fault[3] destroyed;
    fault[3] moved;
    fault[3] enlarged;
    fault[3] set_category;
    fault[3] read_category;
    fault destroy_of_a_live_proxy;
    fault read_of_a_live_proxy;
}

fn ProxyIdGuards the_guards_on_every_call_that_names_a_proxy() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    b3::Aabb unit = {
        .lower_bound = { -1.0, -1.0, -1.0 },
        .upper_bound = { 1.0, 1.0, 1.0 },
    };

    int live = tree.create_proxy(
      aabb: unit,
      category_bits: 1,
      user_data: 1,
    )!!;
    int freed = tree.create_proxy(
      aabb: unit,
      category_bits: 1,
      user_data: 2,
    )!!;

    ProxyIdGuards guards;
    if (catch err = tree.destroy_proxy(freed)) guards.destroy_of_a_live_proxy = err;

    int[3] bad = { -1, tree.node_capacity, freed };
    for (int i = 0; i < 3; i++) {
        if (catch err = tree.destroy_proxy(bad[i])) guards.destroyed[i] = err;
        if (catch err = tree.move_proxy(bad[i], unit)) guards.moved[i] = err;
        if (catch err = tree.enlarge_proxy(bad[i], unit)) guards.enlarged[i] = err;
        if (catch err = tree.set_category_bits(bad[i], 0xF0)) guards.set_category[i] = err;
        if (catch err = tree.category_bits(bad[i])) guards.read_category[i] = err;
    }
    if (catch err = tree.category_bits(live)) guards.read_of_a_live_proxy = err;

    return guards;
}

// Every one of these indexes the node array on the caller's id under an assertion this build
// compiles out, and none of them is harmless without it: a mutator given an id in range but not
// a proxy rewrites the tree's own bookkeeping, and the two ids outside the array reach memory
// the tree does not own at all.
fn void an_id_that_names_no_live_proxy_is_refused_by_every_call_that_takes_one() @test {
    ProxyIdGuards guards = the_guards_on_every_call_that_names_a_proxy();
    fault none;

    assert(guards.destroy_of_a_live_proxy == none, "a live proxy is destroyed, not refused");
    assert(guards.read_of_a_live_proxy == none, "and a live proxy's bits are read, not refused");

    assert(guards.destroyed[0] == b3::INDEX_OUT_OF_RANGE, "a negative id names no node");
    assert(guards.moved[0] == b3::INDEX_OUT_OF_RANGE);
    assert(guards.enlarged[0] == b3::INDEX_OUT_OF_RANGE);
    assert(guards.set_category[0] == b3::INDEX_OUT_OF_RANGE);
    assert(guards.read_category[0] == b3::INDEX_OUT_OF_RANGE);

    assert(guards.destroyed[1] == b3::INDEX_OUT_OF_RANGE, "and neither does the one past the last");
    assert(guards.moved[1] == b3::INDEX_OUT_OF_RANGE);
    assert(guards.enlarged[1] == b3::INDEX_OUT_OF_RANGE);
    assert(guards.set_category[1] == b3::INDEX_OUT_OF_RANGE);
    assert(guards.read_category[1] == b3::INDEX_OUT_OF_RANGE);

    assert(guards.destroyed[2] == b3::NOT_A_PROXY, "a freed slot is in range and holds nothing");
    assert(guards.moved[2] == b3::NOT_A_PROXY);
    assert(guards.enlarged[2] == b3::NOT_A_PROXY);
    assert(guards.set_category[2] == b3::NOT_A_PROXY);
    assert(guards.read_category[2] == b3::NOT_A_PROXY);
}

struct ProxyBoxGuards {
    fault[4] moved;
    fault[4] enlarged;
}

fn ProxyBoxGuards the_boxes_a_move_and_an_enlarge_are_refused_for() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    int proxy = tree.create_proxy(
      aabb: (b3::Aabb){ .lower_bound = { -1.0, -1.0, -1.0 }, .upper_bound = { 1.0, 1.0, 1.0 } },
      category_bits: 1,
      user_data: 1,
    )!!;

    float infinity = (float)1.0e40;
    float nan = infinity - infinity;
    b3::Aabb[4] boxes = {
        { .lower_bound = { 5.0, 5.0, 5.0 }, .upper_bound = { -5.0, -5.0, -5.0 } },
        { .lower_bound = { -infinity, 0.0, 0.0 }, .upper_bound = { infinity, 1.0, 1.0 } },
        { .lower_bound = { nan, 0.0, 0.0 }, .upper_bound = { 1.0, 1.0, 1.0 } },
        { .lower_bound = { 0.0, 0.0, 0.0 }, .upper_bound = { 0.0, 0.0, 0.0 } },
    };

    ProxyBoxGuards guards;
    for (int i = 0; i < 4; i++) {
        if (catch err = tree.move_proxy(proxy, boxes[i])) guards.moved[i] = err;
        if (catch err = tree.enlarge_proxy(proxy, boxes[i])) guards.enlarged[i] = err;
    }

    return guards;
}

// box3d asserts the box on both of these and this build compiles the assertion out, so an invalid
// one reaches a proxy already in the tree: an inverted box takes it out of every query answer and
// an infinite one puts it in all of them. A NaN reaches the ancestors through a move but not
// through an enlarge, which widens on a strict comparison — a rebuild spreads it either way.
fn void a_box_that_box3d_would_assert_against_is_refused_by_both_mutators() @test {
    ProxyBoxGuards guards = the_boxes_a_move_and_an_enlarge_are_refused_for();
    fault none;

    assert(guards.moved[0] == b3::INVALID_DEFINITION, "lower above upper names no volume");
    assert(guards.moved[1] == b3::INVALID_DEFINITION, "an unbounded box would answer every query");
    assert(guards.moved[2] == b3::INVALID_DEFINITION, "a NaN would eat a bound above it");
    assert(guards.moved[3] == none, "and a zero box is a point the tree holds happily");

    assert(guards.enlarged[0] == b3::INVALID_DEFINITION);
    assert(guards.enlarged[1] == b3::INVALID_DEFINITION);
    assert(guards.enlarged[2] == b3::INVALID_DEFINITION);
    assert(guards.enlarged[3] == none);
}

struct QueryHits {
    ulong[8] user_data;
    usz count;
    usz stop_after;
}

fn bool collect_proxy_user_data(int proxy_id, ulong user_data, void* context) {
    QueryHits* hits = (QueryHits*)context;
    if (hits.count < 8) hits.user_data[hits.count] = user_data;
    hits.count++;
    return hits.stop_after == 0 || hits.count < hits.stop_after;
}

fn bool hits_carry_exactly(QueryHits hits, ulong[] expected) {
    if (hits.count != expected.len) return false;
    foreach (want : expected) {
        bool found = false;
        for (usz i = 0; i < hits.count; i++) {
            if (hits.user_data[i] == want) found = true;
        }
        if (!found) return false;
    }
    return true;
}

fn bool hits_carry_exactly_ulongs(ClosestVisits visits, ulong[] expected) {
    if (visits.count != expected.len) return false;
    foreach (want : expected) {
        bool found = false;
        for (usz i = 0; i < visits.count; i++) {
            if (visits.user_data[i] == want) found = true;
        }
        if (!found) return false;
    }
    return true;
}

fn void fill_with_six_proxies(b3::DynamicTree* tree) {
    for (int i = 0; i < 6; i++) {
        tree.create_proxy(
          aabb: (b3::Aabb){
            .lower_bound = { i * 4.0f - 1.0f, -1.0, -1.0 },
            .upper_bound = { i * 4.0f + 1.0f, 1.0, 1.0 },
          },
          category_bits: 1ul << i,
          user_data: (ulong)(i + 1),
        )!!;
    }
}

struct MovedProxyHits {
    QueryHits before;
    QueryHits after;
    int[2] proxy_count;
    int before_leaf_visits;
}

fn MovedProxyHits the_hits_before_and_after_one_proxy_moves() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    b3::Aabb here = {
        .lower_bound = { -1.0, -1.0, -1.0 },
        .upper_bound = { 1.0, 1.0, 1.0 },
    };
    b3::Aabb away = {
        .lower_bound = { 4.0, -1.0, -1.0 },
        .upper_bound = { 6.0, 1.0, 1.0 },
    };
    b3::Aabb window = {
        .lower_bound = { -2.0, -2.0, -2.0 },
        .upper_bound = { 2.0, 2.0, 2.0 },
    };

    tree.create_proxy(
      aabb: here,
      category_bits: ulong::max,
      user_data: 111,
    )!!;
    int moved = tree.create_proxy(
      aabb: away,
      category_bits: ulong::max,
      user_data: 222,
    )!!;

    QueryHits before;
    b3::TreeStats stats = tree.query(
      aabb: window,
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &before,
    );
    int count_before = tree.proxy_count;

    tree.move_proxy(moved, here)!!;

    QueryHits after;
    tree.query(
      aabb: window,
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &after,
    );

    return {
        .before = before,
        .after = after,
        .proxy_count = { count_before, tree.proxy_count },
        .before_leaf_visits = stats.leaf_visits,
    };
}

// The user data is what is asserted rather than the count: a count alone passes for a tree that
// answered with the wrong proxy. The proxy count is asserted beside it so a query that reported
// every proxy in the tree cannot pass for one that reported the two inside the box.
fn void a_query_reports_the_proxies_overlapping_its_box_and_follows_one_that_moves() @test {
    MovedProxyHits hits = the_hits_before_and_after_one_proxy_moves();

    assert(hits.proxy_count[0] == 2 && hits.proxy_count[1] == 2, "a move creates and destroys nothing");
    assert(hits.before.count == 1, "one of the two starts inside the query box");
    assert(hits_carry_exactly(hits.before, (ulong[]){ 111 }), "and it is the one at the origin");
    assert(hits.before_leaf_visits == 1, "which box3d counts as one leaf visit");
    assert(hits.after.count == 2, "and both are inside it once the far one is moved in");
    assert(hits_carry_exactly(hits.after, (ulong[]){ 111, 222 }),
        "each carrying the user data its own proxy was created with");
}

struct MaskedHits {
    QueryHits loose;
    QueryHits strict;
    QueryHits unfiltered;
}

fn MaskedHits the_hits_of_six_proxies_under_one_mask() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();
    fill_with_six_proxies(&tree);

    b3::Aabb everywhere = {
        .lower_bound = { -100.0, -100.0, -100.0 },
        .upper_bound = { 100.0, 100.0, 100.0 },
    };

    MaskedHits hits;
    tree.query(
      aabb: everywhere,
      mask_bits: 0b101,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &hits.loose,
    );
    tree.query(
      aabb: everywhere,
      mask_bits: 0b101,
      require_all_bits: true,
      fcn: &collect_proxy_user_data,
      context: &hits.strict,
    );
    tree.query(
      aabb: everywhere,
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &hits.unfiltered,
    );

    return hits;
}

// Each proxy carries a single category bit, so a mask of two bits is a value at which the two
// flags actually differ: any bit in common reports the two proxies the mask names, while
// demanding every bit reports none, since no proxy carries more than one.
fn void a_query_mask_reports_two_proxies_loosely_and_none_strictly() @test {
    MaskedHits hits = the_hits_of_six_proxies_under_one_mask();

    assert(hits.unfiltered.count == 6, "every proxy is inside the query box to begin with");
    assert(hits.loose.count == 2, "and two of them carry a bit the mask names");
    assert(hits_carry_exactly(hits.loose, (ulong[]){ 1, 3 }), "the ones created with bits one and four");
    assert(hits.strict.count == 0, "while none of them carries both bits");
}

struct StoppedHits {
    QueryHits stopped;
    QueryHits every;
    int stopped_leaf_visits;
    int every_leaf_visits;
}

fn StoppedHits the_hits_when_a_callback_stops_the_traversal() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();
    fill_with_six_proxies(&tree);

    b3::Aabb everywhere = {
        .lower_bound = { -100.0, -100.0, -100.0 },
        .upper_bound = { 100.0, 100.0, 100.0 },
    };

    QueryHits stopped = { .stop_after = 1 };
    b3::TreeStats stopped_stats = tree.query(
      aabb: everywhere,
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &stopped,
    );

    QueryHits every;
    b3::TreeStats every_stats = tree.query(
      aabb: everywhere,
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &every,
    );

    return {
        .stopped = stopped,
        .every = every,
        .stopped_leaf_visits = stopped_stats.leaf_visits,
        .every_leaf_visits = every_stats.leaf_visits,
    };
}

// The same tree and the same query box answer both, so the only difference is what the callback
// returned. Without the contrast a traversal that visited one proxy for its own reasons would
// pass for one the callback stopped.
fn void a_query_callback_returning_false_ends_the_traversal() @test {
    StoppedHits hits = the_hits_when_a_callback_stops_the_traversal();

    assert(hits.every.count == 6, "a callback that keeps returning true is offered every proxy");
    assert(hits.every_leaf_visits == 6);
    assert(hits.stopped.count == 1, "and one that returns false is not called again");
    assert(hits.stopped_leaf_visits == 1, "which box3d counts as the one leaf it reached");
}

struct ClosestVisits {
    float[8] bound_seen;
    ulong[8] user_data;
    usz count;
}

fn float visit_without_narrowing(float search_bound, int proxy_id, ulong user_data, void* context) {
    ClosestVisits* visits = (ClosestVisits*)context;
    if (visits.count < 8) {
        visits.bound_seen[visits.count] = search_bound;
        visits.user_data[visits.count] = user_data;
    }
    visits.count++;
    return search_bound;
}

fn float visit_and_narrow_to_half(float search_bound, int proxy_id, ulong user_data, void* context) {
    ClosestVisits* visits = (ClosestVisits*)context;
    if (visits.count < 8) {
        visits.bound_seen[visits.count] = search_bound;
        visits.user_data[visits.count] = user_data;
    }
    visits.count++;
    return 0.5f;
}

struct ClosestReadings {
    ClosestVisits wide;
    ClosestVisits narrowed;
    ClosestVisits masked;
    ClosestVisits near_the_far_end;
    int strict_leaf_visits;
    float initial_bound;
    float wide_bound;
    float narrowed_bound;
    int wide_leaf_visits;
    int narrowed_leaf_visits;
}

fn ClosestReadings the_visits_of_two_closest_queries() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();
    fill_with_six_proxies(&tree);

    float initial = 1.0e9f;

    ClosestVisits wide;
    b3::ClosestSearch wide_result = tree.query_closest(
      point: { 0.0, 0.0, 0.0 },
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &visit_without_narrowing,
      context: &wide,
      search_bound: initial,
    );

    ClosestVisits narrowed;
    b3::ClosestSearch narrowed_result = tree.query_closest(
      point: { 0.0, 0.0, 0.0 },
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &visit_and_narrow_to_half,
      context: &narrowed,
      search_bound: initial,
    );

    // A bound this tight admits only the proxies within it of the point, so the point itself is
    // what decides the answer rather than merely the order.
    ClosestVisits near_the_far_end;
    tree.query_closest(
      point: { 20.0, 0.0, 0.0 },
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &visit_without_narrowing,
      context: &near_the_far_end,
      search_bound: 20.0,
    );

    ClosestVisits masked;
    b3::ClosestSearch masked_result = tree.query_closest(
      point: { 20.0, 0.0, 0.0 },
      mask_bits: 0x5,
      require_all_bits: false,
      fcn: &visit_without_narrowing,
      context: &masked,
      search_bound: initial,
    );

    ClosestVisits strict;
    b3::ClosestSearch strict_result = tree.query_closest(
      point: { 20.0, 0.0, 0.0 },
      mask_bits: 0x5,
      require_all_bits: true,
      fcn: &visit_without_narrowing,
      context: &strict,
      search_bound: initial,
    );

    return {
        .near_the_far_end = near_the_far_end,
        .masked = masked,
        .strict_leaf_visits = strict_result.stats.leaf_visits,
        .wide = wide,
        .narrowed = narrowed,
        .initial_bound = initial,
        .wide_bound = wide_result.search_bound,
        .narrowed_bound = narrowed_result.search_bound,
        .wide_leaf_visits = wide_result.stats.leaf_visits,
        .narrowed_leaf_visits = narrowed_result.stats.leaf_visits,
    };
}

// The tree prunes on the bound it is handed and never measures anything itself, so the callback
// that returns its argument unchanged is offered every proxy and gets its own starting value
// back, while the one that returns something smaller cuts the rest of the tree off at the first
// leaf. Both bounds are values this test chose and are exactly representable as floats, so they
// are compared exactly rather than banded.
fn void a_closest_query_narrows_only_through_what_its_callback_returns() @test {
    ClosestReadings readings = the_visits_of_two_closest_queries();

    assert(readings.wide.count == 6, "a callback that never narrows is offered every proxy");
    assert(readings.wide_leaf_visits == 6, "one leaf visit per callback");
    assert(readings.wide_bound == readings.initial_bound, "and the bound comes back as it went in");
    assert(readings.wide.bound_seen[0] == readings.initial_bound, "the first call is handed that same bound");
    assert(readings.wide.user_data[0] != 0, "each visit naming its own proxy, none carrying the array's zero");

    assert(readings.narrowed.count == 1, "a callback that narrows cuts off the proxies beyond the bound");
    assert(readings.narrowed_leaf_visits == 1);
    assert(readings.narrowed_bound == 0.5f, "and the bound it returned is the one that comes back");
    assert(readings.narrowed.bound_seen[0] == readings.initial_bound,
        "its own first call having been handed the starting bound too");
}

struct EmptyTreeAnswers {
    usz hits;
    int query_node_visits;
    usz closest_visits;
    int closest_node_visits;
    float initial_bound;
    float closest_bound;
    int proxy_count;
}

fn EmptyTreeAnswers the_answers_an_empty_tree_gives() {
    b3::DynamicTree tree = b3::create_tree(16);
    defer tree.destroy();

    b3::Aabb everywhere = {
        .lower_bound = { -100.0, -100.0, -100.0 },
        .upper_bound = { 100.0, 100.0, 100.0 },
    };

    float initial = 1.0e9f;

    QueryHits hits;
    b3::TreeStats stats = tree.query(
      aabb: everywhere,
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &collect_proxy_user_data,
      context: &hits,
    );

    ClosestVisits visits;
    b3::ClosestSearch closest = tree.query_closest(
      point: { 0.0, 0.0, 0.0 },
      mask_bits: ulong::max,
      require_all_bits: false,
      fcn: &visit_and_narrow_to_half,
      context: &visits,
      search_bound: initial,
    );

    return {
        .hits = hits.count,
        .query_node_visits = stats.node_visits,
        .closest_visits = visits.count,
        .closest_node_visits = closest.stats.node_visits,
        .initial_bound = initial,
        .closest_bound = closest.search_bound,
        .proxy_count = tree.proxy_count,
    };
}

// Both traversals leave before they read a root, which is what keeps them off the -1 box3d marks
// an empty root with. A callback that would have narrowed the bound is used, so a bound that came
// back changed would mean the traversal had reached a node.
fn void an_empty_tree_answers_both_traversals_without_a_single_hit() @test {
    EmptyTreeAnswers answers = the_answers_an_empty_tree_gives();

    assert(answers.proxy_count == 0, "nothing was ever inserted");
    assert(answers.hits == 0, "so the box query calls nothing");
    assert(answers.query_node_visits == 0, "and visits no node at all");
    assert(answers.closest_visits == 0, "the closest query calls nothing either");
    assert(answers.closest_node_visits == 0);
    assert(answers.initial_bound == 1.0e9f, "the bound the query was given is the one under test");
    assert(answers.closest_bound == 1.0e9f, "and an empty tree gives it straight back");
}

// The closest traversal reads the same mask flags the box query does, and its point is the thing
// every other reading here holds at the origin. A query from twenty metres out under a two-bit
// mask separates all three from the defaults a mis-wired forward would fall back to.
fn void a_closest_query_filters_on_the_mask_and_searches_from_the_point_it_was_given() @test {
    ClosestReadings readings = the_visits_of_two_closest_queries();

    assert(readings.masked.count == 2, "two proxies carry a bit the mask names");
    assert(readings.masked.user_data[0] != 0 && readings.masked.user_data[1] != 0,
        "each naming itself rather than the array's zero");
    assert(readings.strict_leaf_visits == 0, "and none of them carries both bits");

    assert(readings.near_the_far_end.count == 2, "a tight bound reaches only what is near the point");
    assert(hits_carry_exactly_ulongs(readings.near_the_far_end, (ulong[]){ 6, 5 }),
        "the two at the far end, which a query from the origin would never offer");
}
