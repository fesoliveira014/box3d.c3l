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
