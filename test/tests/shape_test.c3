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
