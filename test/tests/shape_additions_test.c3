module b3_test;

import b3;

struct CompoundShapeGeometry {
    b3::CompoundData* from_the_compound;
    fault from_the_sphere;
}

fn CompoundShapeGeometry the_compound_two_shapes_report(b3::CompoundData* compound) {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer b3::destroy_world(world);

    b3::BodyDef body_def = b3::default_body_def();
    b3::BodyId body = b3::create_body(world, &body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::ShapeId baked = b3::create_baked_compound_shape(body, &shape_def, compound)!!;
    b3::Sphere ball = { .center = { 0.0, 3.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId sphere = b3::create_sphere_shape(body, &shape_def, &ball)!!;

    CompoundShapeGeometry read = { .from_the_compound = baked.compound()!! };
    if (catch err = sphere.compound()) read.from_the_sphere = err;
    return read;
}

fn void a_compound_shape_hands_back_the_compound_it_borrows() @test {
    b3::CompoundData* compound = a_compound_of_two_unlike_spheres()!!;
    defer b3::destroy_compound(compound);

    CompoundShapeGeometry read = the_compound_two_shapes_report(compound);

    assert(read.from_the_compound == compound, "the very pointer the shape was built from");
    assert(read.from_the_sphere == b3::WRONG_SHAPE_TYPE, "and a sphere has none to give");
}
