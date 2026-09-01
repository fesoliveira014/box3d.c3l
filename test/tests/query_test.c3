module b3_test;

import b3;
import std::math;

fn b3::WorldId a_world_holding_one_sphere(b3::ShapeId* out_shape) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, 0.0, 0.0};
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    *out_shape = body.create_sphere_shape(&shape_def, &sphere)!!;

    return world;
}

fn void a_default_query_filter_considers_every_shape() @test {
    b3::QueryFilter filter = b3::default_query_filter();
    assert(filter.category_bits == ulong::max, "a default query belongs to every category");
    assert(filter.mask_bits == ulong::max, "and accepts every category");
    assert(filter.id == 0, "an untagged query carries no recording id");
}

fn void a_ray_fired_at_a_sphere_reports_where_it_hit() @test {
    b3::ShapeId sphere_shape;
    b3::WorldId world = a_world_holding_one_sphere(&sphere_shape);
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();
    b3::RayResult hit = world.cast_ray_closest({0.0, 4.0, 0.0}, {0.0, -8.0, 0.0}, filter)!!;

    assert(hit.shape_id.index1 == sphere_shape.index1, "the ray hit the sphere");
    assert(hit.fraction > 0.4374f && hit.fraction < 0.4376f, "the ray travelled 3.5 of its 8 units");
    assert(hit.point.y > 0.499f && hit.point.y < 0.501f, "the hit point is on the sphere surface");
    assert(hit.point.x > -0.001f && hit.point.x < 0.001f, "and on the axis the ray travelled");

    float normal_length = math::sqrt(
      hit.normal.x * hit.normal.x + hit.normal.y * hit.normal.y + hit.normal.z * hit.normal.z);
    assert(normal_length > 0.999f && normal_length < 1.001f, "the surface normal is a unit vector");
    assert(hit.normal.y > 0.999f && hit.normal.y < 1.001f, "pointing back along the ray");
}

fn void a_ray_that_hits_nothing_reports_no_hit() @test {
    b3::ShapeId sphere_shape;
    b3::WorldId world = a_world_holding_one_sphere(&sphere_shape);
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();
    if (catch err = world.cast_ray_closest({10.0, 4.0, 0.0}, {0.0, -8.0, 0.0}, filter)) {
        assert(err == b3::NO_HIT, "a ray past everything reports NO_HIT");
        return;
    }
    assert(false, "a ray that passes ten units wide of the sphere must not report a hit");
}

fn void a_query_mask_excludes_a_shape() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, 0.0, 0.0};
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.filter.category_bits = 0x2;
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    body.create_sphere_shape(&shape_def, &sphere)!!;

    b3::QueryFilter accepting = b3::default_query_filter();
    accepting.mask_bits = 0x2;
    assert(@ok(world.cast_ray_closest({0.0, 4.0, 0.0}, {0.0, -8.0, 0.0}, accepting)),
        "a query whose mask covers the shape category hits it");

    b3::QueryFilter rejecting = b3::default_query_filter();
    rejecting.mask_bits = 0x4;
    assert(!@ok(world.cast_ray_closest({0.0, 4.0, 0.0}, {0.0, -8.0, 0.0}, rejecting)),
        "a query whose mask misses the shape category does not");
}

fn void a_ray_cast_at_one_shape_agrees_with_the_world_cast() @test {
    b3::ShapeId sphere_shape;
    b3::WorldId world = a_world_holding_one_sphere(&sphere_shape);
    defer world.destroy();

    b3::CastOutput hit = sphere_shape.ray_cast({0.0, 4.0, 0.0}, {0.0, -8.0, 0.0})!!;
    assert(hit.fraction > 0.4374f && hit.fraction < 0.4376f, "the same 3.5 of 8 units");
    assert(hit.point.y > 0.499f && hit.point.y < 0.501f, "reaching the sphere surface");

    if (catch err = sphere_shape.ray_cast({10.0, 4.0, 0.0}, {0.0, -8.0, 0.0})) {
        assert(err == b3::NO_HIT, "a shape a ray passes wide of reports NO_HIT");
        return;
    }
    assert(false, "a ray ten units wide of the shape must not report a hit");
}

fn void a_cast_at_one_body_reports_the_hit() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.STATIC;
    body_def.position = {0.0, 0.0, 0.0};
    b3::BodyId body = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
    body.create_sphere_shape(&shape_def, &sphere)!!;

    b3::QueryFilter filter = b3::default_query_filter();
    b3::WorldTransform transform = body.transform();

    b3::BodyCastResult hit = body.cast_ray(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -8.0, 0.0},
      filter: filter,
      max_fraction: 1.0f,
      body_transform: transform,
    )!!;
    assert(hit.fraction > 0.4374f && hit.fraction < 0.4376f, "the body cast agrees with the world cast");
    assert(hit.point.y > 0.499f && hit.point.y < 0.501f, "at the sphere surface");

    b3::Vec3[1] proxy_points = { {0.0, 0.0, 0.0} };
    b3::ShapeProxy proxy = {
      .points = &proxy_points[0],
      .count = 1,
      .radius = 0.25,
    };

    b3::BodyCastResult swept = body.cast_shape(
      origin: {0.0, 4.0, 0.0},
      proxy: &proxy,
      translation: {0.0, -8.0, 0.0},
      filter: filter,
      max_fraction: 1.0f,
      can_encroach: false,
      body_transform: transform,
    )!!;
    assert(swept.fraction > 0.4055f && swept.fraction < 0.4075f,
        "a sphere of radius 0.25 stops a quarter unit sooner, less box3d's speculative margin");

    bool at_the_centre = body.overlap_shape(
      origin: {0.0, 0.0, 0.0},
      proxy: &proxy,
      filter: filter,
      body_transform: transform,
    );
    assert(at_the_centre, "a probe at the sphere centre overlaps it");

    bool well_above = body.overlap_shape(
      origin: {0.0, 9.0, 0.0},
      proxy: &proxy,
      filter: filter,
      body_transform: transform,
    );
    assert(!well_above, "and a probe nine units above it does not");
}

fn void the_closest_point_on_a_body_ignores_the_shape_radius() @test {
    b3::ShapeId sphere_shape;
    b3::WorldId world = a_world_holding_one_sphere(&sphere_shape);
    defer world.destroy();

    b3::BodyId body = sphere_shape.body();
    b3::ClosestPoint closest = body.closest_point({0.0, 4.0, 0.0});

    assert(closest.distance > 3.99f && closest.distance < 4.01f,
        "the distance runs to the sphere centre, not its surface");
    assert(closest.point.y > -0.01f && closest.point.y < 0.01f,
        "and the point is the centre for the same reason");
}

struct RayVisitLog {
    int calls;
    int stop_after;
}

fn float log_visit_and_continue(
  b3::ShapeId shape_id,
  b3::Pos point,
  b3::Vec3 normal,
  float fraction,
  ulong user_material_id,
  int triangle_index,
  int child_index,
  void* context,
) {
    RayVisitLog* log = (RayVisitLog*)context;
    log.calls++;
    if (log.stop_after > 0 && log.calls >= log.stop_after) return 0.0f;
    return 1.0f;
}

fn float log_visit_and_filter(
  b3::ShapeId shape_id,
  b3::Pos point,
  b3::Vec3 normal,
  float fraction,
  ulong user_material_id,
  int triangle_index,
  int child_index,
  void* context,
) {
    RayVisitLog* log = (RayVisitLog*)context;
    log.calls++;
    return -1.0f;
}

fn bool log_overlap(b3::ShapeId shape_id, void* context) {
    RayVisitLog* log = (RayVisitLog*)context;
    log.calls++;
    return true;
}

fn bool hits_match_the_expected_fractions(b3::CastHit[] hits, float[3] expected) {
    bool[3] matched;
    foreach (hit : hits) {
        bool found = false;
        for (int j = 0; j < 3; j++) {
            if (!matched[j] && math::is_approx(hit.fraction, expected[j], 0.001f)) {
                matched[j] = true;
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

fn b3::WorldId a_world_holding_three_spheres_in_a_line() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    float[3] heights = {0.0f, -2.0f, -4.0f};
    for (int i = 0; i < 3; i++) {
        b3::BodyDef body_def = b3::default_body_def();
        body_def.type = b3::BodyType.STATIC;
        body_def.position = {0.0, heights[i], 0.0};
        b3::BodyId body = world.create_body(&body_def)!!;

        b3::ShapeDef shape_def = b3::default_shape_def();
        b3::Sphere sphere = {.center = {0.0, 0.0, 0.0}, .radius = 0.5};
        body.create_sphere_shape(&shape_def, &sphere)!!;
    }
    return world;
}

fn void a_cast_callback_controls_how_far_the_ray_goes() @test {
    b3::WorldId world = a_world_holding_three_spheres_in_a_line();
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();

    RayVisitLog every = { .calls = 0, .stop_after = 0 };
    b3::TreeStats stats = world.cast_ray(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -12.0, 0.0},
      filter: filter,
      fcn: &log_visit_and_continue,
      context: &every,
    );
    assert(every.calls == 3, "returning one visits every sphere along the ray");
    assert(stats.node_visits > 0, "the node traversal counter is reported");
    assert(stats.leaf_visits == 3, "one leaf per sphere along the unclipped ray");

    RayVisitLog stopped = { .calls = 0, .stop_after = 1 };
    world.cast_ray(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -12.0, 0.0},
      filter: filter,
      fcn: &log_visit_and_continue,
      context: &stopped,
    );
    assert(stopped.calls == 1, "returning zero terminates the cast at the first shape");

    RayVisitLog filtered = { .calls = 0, .stop_after = 0 };
    world.cast_ray(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -12.0, 0.0},
      filter: filter,
      fcn: &log_visit_and_filter,
      context: &filtered,
    );
    assert(filtered.calls == 3, "returning minus one rejects a shape without ending the cast");
}

fn void an_overlap_callback_sees_the_shapes_in_the_box() @test {
    b3::WorldId world = a_world_holding_three_spheres_in_a_line();
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();
    b3::Aabb box = {
      .lower_bound = {-1.0, -5.0, -1.0},
      .upper_bound = {1.0, 1.0, 1.0},
    };

    RayVisitLog log = { .calls = 0, .stop_after = 0 };
    world.overlap_aabb(
      aabb: box,
      filter: filter,
      fcn: &log_overlap,
      context: &log,
    );
    assert(log.calls == 3, "all three spheres fall inside the box");

    b3::Vec3[1] proxy_points = { {0.0, 0.0, 0.0} };
    b3::ShapeProxy proxy = {
      .points = &proxy_points[0],
      .count = 1,
      .radius = 0.25,
    };

    RayVisitLog probe = { .calls = 0, .stop_after = 0 };
    world.overlap_shape(
      origin: {0.0, 0.0, 0.0},
      proxy: &proxy,
      filter: filter,
      fcn: &log_overlap,
      context: &probe,
    );
    assert(probe.calls == 1, "a probe at the origin overlaps only the sphere there");
}

fn void collecting_a_cast_fills_the_caller_buffer() @test {
    b3::WorldId world = a_world_holding_three_spheres_in_a_line();
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();

    b3::CastHit[8] roomy;
    b3::CastHit[] all = world.cast_ray_all(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -12.0, 0.0},
      filter: filter,
      buffer: roomy[..],
    );
    assert(all.len == 3, "a buffer with room collects every sphere");
    float[3] expected_fractions = { 0.291667f, 0.458333f, 0.625f };
    assert(hits_match_the_expected_fractions(all, expected_fractions),
        "the three collected fractions match the three spheres' fractions, each exactly once");

    b3::CastHit[2] cramped;
    b3::CastHit[] some = world.cast_ray_all(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -12.0, 0.0},
      filter: filter,
      buffer: cramped[..],
    );
    assert(some.len == 2, "a full buffer ends the cast rather than dropping hits");

    b3::CastHit[] none = world.cast_ray_all(
      origin: {0.0, 4.0, 0.0},
      translation: {0.0, -12.0, 0.0},
      filter: filter,
      buffer: roomy[:0],
    );
    assert(none.len == 0, "an empty buffer collects nothing and writes nothing");
}

fn void collecting_an_overlap_fills_the_caller_buffer() @test {
    b3::WorldId world = a_world_holding_three_spheres_in_a_line();
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();
    b3::Aabb box = {
      .lower_bound = {-1.0, -5.0, -1.0},
      .upper_bound = {1.0, 1.0, 1.0},
    };

    b3::ShapeId[8] roomy;
    b3::ShapeId[] found = world.overlap_aabb_all(box, filter, roomy[..]);
    assert(found.len == 3, "all three spheres are collected");

    b3::ShapeId[1] cramped;
    b3::ShapeId[] one = world.overlap_aabb_all(box, filter, cramped[..]);
    assert(one.len == 1, "a full buffer ends the query");

    b3::Vec3[1] proxy_points = { {0.0, 0.0, 0.0} };
    b3::ShapeProxy proxy = {
      .points = &proxy_points[0],
      .count = 1,
      .radius = 0.25,
    };
    b3::ShapeId[] probed = world.overlap_shape_all(
      origin: {0.0, 0.0, 0.0},
      proxy: &proxy,
      filter: filter,
      buffer: roomy[..],
    );
    assert(probed.len == 1, "a probe at the origin collects only the sphere there");
}

fn void collecting_a_shape_cast_reports_the_swept_fraction() @test {
    b3::ShapeId sphere_shape;
    b3::WorldId world = a_world_holding_one_sphere(&sphere_shape);
    defer world.destroy();

    b3::QueryFilter filter = b3::default_query_filter();
    b3::Vec3[1] proxy_points = { {0.0, 0.0, 0.0} };
    b3::ShapeProxy proxy = {
      .points = &proxy_points[0],
      .count = 1,
      .radius = 0.25,
    };

    b3::CastHit[4] buffer;
    b3::CastHit[] hits = world.cast_shape_all(
      origin: {0.0, 4.0, 0.0},
      proxy: &proxy,
      translation: {0.0, -8.0, 0.0},
      filter: filter,
      buffer: buffer[..],
    );
    assert(hits.len == 1, "the swept sphere reaches the resting one");
    assert(hits[0].fraction > 0.4055f && hits[0].fraction < 0.4075f,
        "stopping a quarter unit early, less box3d's speculative margin");
    assert(hits[0].shape_id.index1 == sphere_shape.index1, "and names the shape it reached");
    assert(hits[0].point.y > 0.49f && hits[0].point.y < 0.51f, "the hit point is on the resting sphere's surface");
    assert(hits[0].normal.y > 0.99f && hits[0].normal.y < 1.01f, "and the surface normal points back along the sweep");
}
