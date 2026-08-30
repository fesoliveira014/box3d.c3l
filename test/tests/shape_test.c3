module b3_test;

import b3;
import std::math;

fn void the_shape_types_match_the_c_layout() @test {
    assert(b3::Filter::size == 24);
    assert(b3::SurfaceMaterial::size == 40);
    assert(b3::ShapeType::size == 4);
    assert(b3::ShapeDef::size == 120);
    assert(b3::ShapeDef::alignment == 8);
    assert(b3::Sphere::size == 16);
    assert(b3::Capsule::size == 28);
    assert(b3::HullData::size == 144);
    assert(b3::BoxHull::size == 640);
}

fn void the_shape_type_values_are_contiguous_from_zero() @test {
    assert((int)b3::ShapeType.CAPSULE == 0);
    assert((int)b3::ShapeType.COMPOUND == 1);
    assert((int)b3::ShapeType.HEIGHT_FIELD == 2);
    assert((int)b3::ShapeType.HULL == 3);
    assert((int)b3::ShapeType.MESH == 4);
    assert((int)b3::ShapeType.SPHERE == 5);
}

fn void the_default_shape_definition_carries_the_values_the_c_side_sets() @test {
    b3::ShapeDef def = b3::default_shape_def();

    assert(def.density == 1000.0);
    assert(def.explosion_scale == 1.0);
    assert(def.update_body_mass);
    assert(def.invoke_contact_creation);
    assert(def.enable_speculative_contact);
    assert(!def.is_sensor);
    assert(!def.enable_sensor_events);
    assert(def.base_material.friction == (float)0.6);
    assert(def.base_material.restitution == 0.0);
    assert(def.filter.category_bits == ulong::max);
    assert(def.filter.mask_bits == ulong::max);
    assert(def.filter.group_index == 0);
}

fn void a_zeroed_shape_definition_differs_from_a_default_one() @test {
    b3::ShapeDef zeroed;
    b3::ShapeDef def = b3::default_shape_def();

    assert(zeroed.internal_value != def.internal_value);
}

fn void the_default_filter_and_material_match_the_c_side() @test {
    b3::Filter filter = b3::default_filter();
    b3::SurfaceMaterial material = b3::default_surface_material();

    assert(filter.category_bits == ulong::max);
    assert(filter.mask_bits == ulong::max);
    assert(filter.group_index == 0);
    assert(material.friction == (float)0.6);
    assert(material.restitution == 0.0);
    assert(material.rolling_resistance == 0.0);
    assert(material.user_material_id == 0);
    assert(material.custom_color == 0);
}

fn void a_shape_definition_without_the_cookie_is_refused() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef zeroed;
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };

    if (catch err = body.create_sphere_shape(&zeroed, &sphere)) {
        assert(err == b3::INVALID_DEFINITION);
        return;
    }
    assert(false, "box3d accepts a zeroed shape definition in a release build; the binding must not");
}

fn void a_box_hull_crosses_the_abi_by_value() @test {
    b3::BoxHull box = b3::make_box_hull(0.5, 0.5, 0.5);

    assert(box.base.volume == 1.0);
    assert(box.base.surface_area == 6.0);
    assert(box.base.inner_radius == 0.5);
    assert(box.base.vertex_count == 8);
    assert(box.base.edge_count == 24);
    assert(box.base.face_count == 6);
    assert(box.base.byte_count == b3::BoxHull::size);

    b3::BoxHull copy = box;
    assert(copy.base.volume == 1.0, "a box hull addresses its arrays by offset, so a copy stays valid");
    assert(copy.base.vertex_count == 8);

    b3::BoxHull cube = b3::make_cube_hull(0.5);
    assert(cube.base.volume == box.base.volume);

    b3::BoxHull offset = b3::make_offset_box_hull(
      hx: 0.5,
      hy: 0.5,
      hz: 0.5,
      offset: { 1.0, 2.0, 3.0 },
    );
    assert(offset.base.center.x == 1.0);
    assert(offset.base.center.y == 2.0);
    assert(offset.base.center.z == 3.0);

    b3::Transform local = { .p = { 2.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
    b3::BoxHull transformed = b3::make_transformed_box_hull(
      hx: 0.5,
      hy: 0.5,
      hz: 0.5,
      transform: local,
    );
    assert(transformed.base.center.x == 2.0);
    assert(transformed.base.volume == 1.0);

    b3::BoxHull scaled = b3::make_scaled_box_hull(
      half_widths: { 0.5, 0.5, 0.5 },
      transform: local,
      post_scale: { 2.0, 1.0, 1.0 },
    );
    assert(scaled.base.center.x == 4.0, "post scale applies to the baked translation too");
    assert(scaled.base.volume == 2.0);
}

fn void a_shape_knows_the_body_and_world_it_belongs_to() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.25, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    assert(shape.is_valid());
    assert(shape.type() == b3::ShapeType.SPHERE);
    assert(shape.body().index1 == body.index1);
    assert(shape.world().index1 == world.index1);
    assert(!shape.is_sensor());
}

fn void a_sensor_definition_produces_a_sensor_shape() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    def.is_sensor = true;
    def.enable_sensor_events = true;
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId sensor = body.create_sphere_shape(&def, &sphere)!!;

    assert(sensor.is_sensor());
    assert(sensor.are_sensor_events_enabled());
}

fn void a_capsule_and_a_hull_attach_to_their_bodies() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;

    b3::BodyId capsule_body = world.create_body(&body_def)!!;
    b3::ShapeDef capsule_def = b3::default_shape_def();
    b3::Capsule capsule = { .center1 = { 0.0, -0.5, 0.0 }, .center2 = { 0.0, 0.5, 0.0 }, .radius = 0.25 };
    b3::ShapeId capsule_shape = capsule_body.create_capsule_shape(&capsule_def, &capsule)!!;
    assert(capsule_shape.type() == b3::ShapeType.CAPSULE);

    b3::BodyId hull_body = world.create_body(&body_def)!!;
    b3::ShapeDef hull_def = b3::default_shape_def();
    b3::BoxHull hull = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::ShapeId hull_shape = hull_body.create_hull_shape(&hull_def, &hull.base)!!;
    assert(hull_shape.type() == b3::ShapeType.HULL);

    b3::BodyId baked_body = world.create_body(&body_def)!!;
    b3::ShapeDef baked_def = b3::default_shape_def();
    b3::Transform local = { .p = { 2.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
    b3::ShapeId baked = baked_body.create_transformed_hull_shape(
      def: &baked_def,
      hull: &hull.base,
      transform: local,
      scale: { 2.0, 1.0, 1.0 },
    )!!;
    assert(baked.type() == b3::ShapeType.HULL);
    assert(math::abs(baked_body.mass() - 2000.0) < 0.001, "scaling a unit box by two doubles its mass");
}

fn void destroying_a_shape_takes_its_mass_with_it() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    assert(body.mass() > 0.0);
    shape.destroy(true);
    assert(!shape.is_valid());
    assert(body.mass() == 0.0);
}

fn void a_world_with_shapes_releases_everything_it_allocated() @test {
    int baseline = b3::byte_count();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;
    b3::ShapeDef def = b3::default_shape_def();
    b3::BoxHull hull = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::ShapeId shape = body.create_hull_shape(&def, &hull.base)!!;
    assert(shape.is_valid());

    world.destroy();
    assert(b3::byte_count() == baseline, "a destroyed world releases every shape it owned");
}

fn void shape_properties_round_trip() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    shape.set_friction(0.25);
    shape.set_restitution(0.5);
    shape.set_density(2000.0, true);
    shape.set_name("hull plating");
    assert(shape.friction() == 0.25);
    assert(shape.restitution() == 0.5);
    assert(shape.density() == 2000.0);
    assert(shape.name().str_view() == "hull plating");

    b3::Filter filter = b3::default_filter();
    filter.category_bits = 4;
    filter.mask_bits = 12;
    filter.group_index = -3;
    shape.set_filter(filter, true);
    b3::Filter back = shape.filter();
    assert(back.category_bits == 4);
    assert(back.mask_bits == 12);
    assert(back.group_index == -3);

    b3::SurfaceMaterial material = b3::default_surface_material();
    material.friction = 0.125;
    material.rolling_resistance = 0.25;
    material.user_material_id = 7;
    shape.set_surface_material(material);
    assert(shape.surface_material().friction == 0.125);
    assert(shape.surface_material().rolling_resistance == 0.25);
    assert(shape.surface_material().user_material_id == 7);

    int marker = 42;
    shape.set_user_data(&marker);
    assert(*(int*)shape.user_data() == 42);
}

fn void the_shape_event_flags_start_disabled_and_round_trip() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    assert(!shape.are_sensor_events_enabled());
    assert(!shape.are_contact_events_enabled());
    assert(!shape.are_hit_events_enabled());
    assert(!shape.are_pre_solve_events_enabled());

    shape.enable_sensor_events(true);
    shape.enable_contact_events(true);
    shape.enable_hit_events(true);
    shape.enable_pre_solve_events(true);
    assert(shape.are_sensor_events_enabled());
    assert(shape.are_contact_events_enabled());
    assert(shape.are_hit_events_enabled());
    assert(shape.are_pre_solve_events_enabled());

    shape.enable_contact_events(false);
    assert(!shape.are_contact_events_enabled());
}

fn void a_sphere_round_trips_its_geometry() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.25, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    b3::Sphere back = shape.sphere()!!;
    assert(back.center.x == 0.25);
    assert(back.radius == 0.5);

    b3::Sphere bigger = { .center = { 0.0, 0.0, 0.0 }, .radius = 1.0 };
    shape.set_sphere(&bigger);
    assert(shape.sphere()!!.radius == 1.0);
}

fn void a_capsule_round_trips_its_geometry() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Capsule capsule = { .center1 = { 0.0, -0.5, 0.0 }, .center2 = { 0.0, 0.5, 0.0 }, .radius = 0.25 };
    b3::ShapeId shape = body.create_capsule_shape(&def, &capsule)!!;

    b3::Capsule back = shape.capsule()!!;
    assert(back.radius == 0.25);
    assert(back.center2.y == 0.5);

    b3::Capsule longer = { .center1 = { 0.0, -1.0, 0.0 }, .center2 = { 0.0, 1.0, 0.0 }, .radius = 0.25 };
    shape.set_capsule(&longer);
    assert(shape.capsule()!!.center2.y == 1.0);
}

fn void a_hull_round_trips_its_geometry() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::BoxHull unit = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::ShapeId shape = body.create_hull_shape(&def, &unit.base)!!;

    assert(shape.hull()!!.volume == 1.0);
    assert(shape.hull()!!.vertex_count == 8);

    b3::BoxHull wide = b3::make_box_hull(1.0, 1.0, 1.0);
    shape.set_hull(&wide.base);
    assert(shape.hull()!!.volume == 8.0);
}

fn void replacing_a_shape_geometry_leaves_the_body_mass_alone() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;
    float before = body.mass();

    b3::Sphere bigger = { .center = { 0.0, 0.0, 0.0 }, .radius = 1.0 };
    shape.set_sphere(&bigger);
    assert(body.mass() == before, "box3d recomputes body mass only when asked");

    body.apply_mass_from_shapes();
    float expected = before * 8.0;
    assert(math::abs(body.mass() - expected) < expected * 0.01, "eight times the volume once the body is told to recompute");
}

fn void asking_a_shape_for_geometry_it_does_not_have_is_a_fault() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Capsule capsule = { .center1 = { 0.0, -0.5, 0.0 }, .center2 = { 0.0, 0.5, 0.0 }, .radius = 0.25 };
    b3::ShapeId shape = body.create_capsule_shape(&def, &capsule)!!;

    if (catch err = shape.sphere()) {
        assert(err == b3::WRONG_SHAPE_TYPE);
    } else {
        assert(false, "a capsule must not answer a sphere query");
    }

    if (catch err = shape.hull()) {
        assert(err == b3::WRONG_SHAPE_TYPE);
    } else {
        assert(false, "a capsule must not answer a hull query");
    }
}

fn void setting_a_capsule_shape_to_sphere_geometry_turns_it_into_a_sphere() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Capsule capsule = { .center1 = { 0.0, -0.5, 0.0 }, .center2 = { 0.0, 0.5, 0.0 }, .radius = 0.25 };
    b3::ShapeId shape = body.create_capsule_shape(&def, &capsule)!!;

    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    shape.set_sphere(&sphere);

    assert(shape.type() == b3::ShapeType.SPHERE);
    assert(shape.sphere()!!.radius == 0.5);
}

fn void a_sphere_gives_its_body_the_mass_and_inertia_its_density_implies() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    float expected = (float)(1000.0 * 4.0 / 3.0 * math::PI * 0.125);
    assert(math::abs(body.mass() - expected) < 0.001);

    b3::MassData mass_data = shape.compute_mass_data();
    float inertia = (float)(0.4 * expected * 0.25);
    assert(math::abs(mass_data.mass - expected) < 0.001);
    assert(math::abs(mass_data.inertia[0].x - inertia) < 0.001);
    assert(math::abs(mass_data.inertia[1].y - inertia) < 0.001);
    assert(math::abs(mass_data.inertia[2].z - inertia) < 0.001);
}

fn void a_capsule_gives_its_body_the_mass_of_a_cylinder_and_two_hemispheres() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Capsule capsule = { .center1 = { 0.0, -0.5, 0.0 }, .center2 = { 0.0, 0.5, 0.0 }, .radius = 0.25 };
    b3::ShapeId shape = body.create_capsule_shape(&def, &capsule)!!;
    assert(shape.is_valid());

    float expected = (float)(1000.0 * (math::PI * 0.0625 + 4.0 / 3.0 * math::PI * 0.015625));
    assert(math::abs(body.mass() - expected) < 0.001);
}

fn void the_shape_bounds_carry_the_speculative_margin() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    b3::Aabb bounds = shape.aabb();
    float margin = -bounds.lower_bound.x - 0.5f;
    assert(margin > 0.0 && margin < 0.1, "box3d fattens shape bounds by a small speculative margin");
    assert(bounds.upper_bound.x == -bounds.lower_bound.x);

    b3::Vec3 closest = shape.closest_point({ 0.0, 20.0, 0.0 });
    assert(math::abs(closest.y - 0.5) < 0.001, "the closest point to a distant target lies on the surface");
}

fn void wind_pushes_a_shape_downwind() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = b3::BodyType.DYNAMIC;
    body_def.position = { 0.0, 50.0, 0.0 };
    b3::BodyId body = world.create_body(&body_def)!!;

    b3::ShapeDef def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = body.create_sphere_shape(&def, &sphere)!!;

    for (int i = 0; i < 60; i++) {
        shape.apply_wind(
          wind: { 20.0, 0.0, 0.0 },
          drag: 1.0,
          lift: 0.0,
          max_speed: 100.0,
          wake: true,
        );
        world.step(1.0f / 60.0f, 4);
    }

    float downwind = body.linear_velocity().x;
    assert(downwind > 0.2 && downwind < 0.6, "a second of wind at speed 20 with drag 1 moves a falling sphere about 0.375 m/s downwind");
}

fn void a_box_comes_to_rest_on_static_ground() @test {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::BoxHull ground_hull = b3::make_box_hull(10.0, 0.5, 10.0);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    b3::ShapeId ground_shape = ground.create_hull_shape(&ground_shape_def, &ground_hull.base)!!;
    assert(ground_shape.type() == b3::ShapeType.HULL);

    b3::BodyDef crate_def = b3::default_body_def();
    crate_def.type = b3::BodyType.DYNAMIC;
    crate_def.position = { 0.0, 5.0, 0.0 };
    b3::BodyId crate = world.create_body(&crate_def)!!;
    b3::BoxHull crate_hull = b3::make_box_hull(0.5, 0.5, 0.5);
    b3::ShapeDef crate_shape_def = b3::default_shape_def();
    b3::ShapeId crate_shape = crate.create_hull_shape(&crate_shape_def, &crate_hull.base)!!;
    assert(math::abs(crate.mass() - 1000.0) < 0.001);

    for (int i = 0; i < 240; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    b3::Pos resting = crate.position();
    assert(math::abs(resting.y - 1.0) < 0.01, "a unit box on half-metre-thick ground rests one metre up");
    assert(math::abs(resting.x) < 0.01);
    assert(!crate.is_awake(), "a box at rest falls asleep");
    assert(crate_shape.is_valid());
}
