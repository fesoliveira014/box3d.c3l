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

fn void the_shape_type_values_match_the_c_enum() @test {
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

    b3::BoxHull offset = b3::make_offset_box_hull(0.5, 0.5, 0.5, { 1.0, 2.0, 3.0 });
    assert(offset.base.center.x == 1.0);
    assert(offset.base.center.y == 2.0);
    assert(offset.base.center.z == 3.0);

    b3::Transform local = { .p = { 2.0, 0.0, 0.0 }, .q = { 0.0, 0.0, 0.0, 1.0 } };
    b3::BoxHull transformed = b3::make_transformed_box_hull(0.5, 0.5, 0.5, local);
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
