module b3_test;

import b3;

fn void the_geometry_definition_types_match_the_c_layout() @test {
    assert(b3::MeshDef::size == 40);
    assert(b3::MeshDef::alignment == 8);
    assert(b3::HeightFieldDef::size == 48);
    assert(b3::HeightFieldDef::alignment == 8);
}

struct CannedMeshes {
    bool grid;
    bool wave;
    bool torus;
    bool box;
    bool hollow_box;
    bool platform;
    bool soup;
}

// Every helper here destroys what it built before returning, and none of them asserts: C3 skips
// defer on a panicking assertion, and a stranded mesh sits on byte_count() for the rest of the
// run exactly as a stranded world does.
fn CannedMeshes the_canned_builders_that_produced_a_mesh() {
    b3::Vec3 center = { 0.0, 0.0, 0.0 };
    b3::Vec3 extent = { 1.0, 1.0, 1.0 };

    b3::MeshData*? grid = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    );
    defer if (try grid) grid.destroy();

    b3::MeshData*? wave = b3::create_wave_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      amplitude: 0.5,
      row_frequency: 0.5,
      column_frequency: 0.5,
    );
    defer if (try wave) wave.destroy();

    b3::MeshData*? torus = b3::create_torus_mesh(
      radial_resolution: 8,
      tubular_resolution: 8,
      radius: 1.0,
      thickness: 0.25,
    );
    defer if (try torus) torus.destroy();

    b3::MeshData*? box = b3::create_box_mesh(center, extent, true);
    defer if (try box) box.destroy();

    b3::MeshData*? hollow_box = b3::create_hollow_box_mesh(center, extent);
    defer if (try hollow_box) hollow_box.destroy();

    b3::MeshData*? platform = b3::create_platform_mesh(
      center: center,
      height: 1.0,
      top_width: 1.0,
      bottom_width: 2.0,
    );
    defer if (try platform) platform.destroy();

    b3::MeshData*? soup = a_single_triangle_mesh();
    defer if (try soup) soup.destroy();

    return {
        .grid = @ok(grid),
        .wave = @ok(wave),
        .torus = @ok(torus),
        .box = @ok(box),
        .hollow_box = @ok(hollow_box),
        .platform = @ok(platform),
        .soup = @ok(soup),
    };
}

// The three vertices span a unit right triangle in the x-z plane, so the one triangle has area
// and box3d keeps it. box3d copies the arrays, which is why locals are enough.
fn b3::MeshData*? a_single_triangle_mesh() {
    b3::Vec3[3] vertices = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 } };
    int[3] indices = { 0, 1, 2 };

    b3::MeshDef def;
    def.vertices = &vertices[0];
    def.indices = &indices[0];
    def.vertex_count = 3;
    def.triangle_count = 1;

    return b3::create_mesh(&def);
}

fn void every_mesh_builder_produces_a_mesh() @test {
    CannedMeshes built = the_canned_builders_that_produced_a_mesh();

    assert(built.grid, "a four by four grid of cells");
    assert(built.wave, "a four by four grid with a wave over it");
    assert(built.torus, "a torus of eight by eight segments");
    assert(built.box, "a closed box of unit half-extents");
    assert(built.hollow_box, "the same box wound inwards");
    assert(built.platform, "a truncated pyramid");
    assert(built.soup, "and a triangle soup of exactly one triangle");
}

fn bool[3] the_height_field_builders_that_produced_a_field() {
    b3::Vec3 scale = { 1.0, 1.0, 1.0 };

    b3::HeightFieldData*? grid = b3::create_height_field_grid(
      row_count: 16,
      column_count: 16,
      scale: scale,
      make_holes: false,
    );
    defer if (try grid) grid.destroy();

    b3::HeightFieldData*? wave = b3::create_height_field_wave(
      row_count: 16,
      column_count: 16,
      scale: scale,
      row_frequency: 0.5,
      column_frequency: 0.5,
      make_holes: false,
    );
    defer if (try wave) wave.destroy();

    b3::HeightFieldData*? measured = a_flat_three_by_three_height_field();
    defer if (try measured) measured.destroy();

    return { @ok(grid), @ok(wave), @ok(measured) };
}

fn b3::HeightFieldData*? a_flat_three_by_three_height_field() {
    float[9] heights;

    b3::HeightFieldDef def;
    def.heights = &heights[0];
    def.scale = (b3::Vec3){ 1.0, 1.0, 1.0 };
    def.count_x = 3;
    def.count_z = 3;
    def.global_minimum_height = -1.0;
    def.global_maximum_height = 1.0;

    return b3::create_height_field(&def);
}

fn void every_height_field_builder_produces_a_field() @test {
    bool[3] built = the_height_field_builders_that_produced_a_field();

    assert(built[0], "a sixteen by sixteen flat grid");
    assert(built[1], "the same grid with two waves crossed over it");
    assert(built[2], "and a three by three field built from measured heights");
}

// The byte count is read while the mesh is alive and again by the caller, after the helper's
// defer has run. Taking the second reading here would need the destroy out of that defer.
fn int[2] the_byte_count_around_a_live_grid_mesh() {
    int baseline = b3::byte_count();

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    return { baseline, b3::byte_count() };
}

fn int[2] the_byte_count_around_a_live_height_field() {
    int baseline = b3::byte_count();

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 16,
      column_count: 16,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer field.destroy();

    return { baseline, b3::byte_count() };
}

// The baseline is taken inside the run rather than assumed to be zero: other tests in the same
// process may still hold a world when this one runs.
fn void a_destroyed_mesh_gives_back_every_byte_it_took() @test {
    int[2] around = the_byte_count_around_a_live_grid_mesh();

    assert(around[1] > around[0], "a mesh box3d allocates must show up in the byte count");
    assert(b3::byte_count() == around[0], "and destroying it must give every byte back");
}

fn void a_destroyed_height_field_gives_back_every_byte_it_took() @test {
    int[2] around = the_byte_count_around_a_live_height_field();

    assert(around[1] > around[0], "a height field box3d allocates must show up in the byte count");
    assert(b3::byte_count() == around[0], "and destroying it must give every byte back");
}

struct Refusal {
    fault refusal;
    int baseline;
    int after;
}

fn Refusal the_refusal_of_a_mesh_definition(b3::MeshDef* def) {
    int baseline = b3::byte_count();

    b3::MeshData*? mesh = b3::create_mesh(def);
    defer if (try mesh) mesh.destroy();

    if (catch err = mesh) return { .refusal = err, .baseline = baseline, .after = b3::byte_count() };
    return { .refusal = {}, .baseline = baseline, .after = b3::byte_count() };
}

// box3d takes this refusal before it clones anything, so the byte count is the evidence that
// the definition was turned away rather than half built. That does not hold of every refusal:
// geometry rejected for holding no triangle with area is rejected after the clone and box3d
// never frees it, so a test must not reach that path.
fn void a_zeroed_mesh_definition_is_refused_without_allocating() @test {
    b3::MeshDef zeroed;

    Refusal refused = the_refusal_of_a_mesh_definition(&zeroed);

    assert(refused.refusal == b3::DEGENERATE_GEOMETRY, "a definition naming no vertices names no mesh");
    assert(refused.after == refused.baseline, "and box3d turns it away before allocating");
}

fn void a_mesh_definition_naming_fewer_than_three_vertices_is_refused() @test {
    b3::Vec3[3] vertices = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 } };
    int[3] indices = { 0, 1, 2 };

    b3::MeshDef def;
    def.vertices = &vertices[0];
    def.indices = &indices[0];
    def.vertex_count = 2;
    def.triangle_count = 1;

    Refusal refused = the_refusal_of_a_mesh_definition(&def);

    assert(refused.refusal == b3::DEGENERATE_GEOMETRY, "a triangle needs three vertices");
    assert(refused.after == refused.baseline);
}

// box3d asserts that a grid holds at least four points and that its global range runs upwards,
// and this release build compiles both assertions out, so it builds a height field nothing can
// stand on instead. A missing height array it does not check on any build: it reads
// count_x * count_z floats straight through the pointer.
fn fault the_refusal_of_a_height_field_definition(b3::HeightFieldDef* def) {
    b3::HeightFieldData*? field = b3::create_height_field(def);
    defer if (try field) field.destroy();

    if (catch err = field) return err;
    return {};
}

fn void a_height_field_definition_too_small_to_hold_a_cell_is_refused() @test {
    float[9] heights;

    b3::HeightFieldDef def;
    def.heights = &heights[0];
    def.scale = (b3::Vec3){ 1.0, 1.0, 1.0 };
    def.count_x = 1;
    def.count_z = 3;
    def.global_minimum_height = -1.0;
    def.global_maximum_height = 1.0;

    assert(the_refusal_of_a_height_field_definition(&def) == b3::DEGENERATE_GEOMETRY,
        "one grid line along x leaves no cell between two of them");

    def.count_x = 3;
    def.count_z = 1;
    assert(the_refusal_of_a_height_field_definition(&def) == b3::DEGENERATE_GEOMETRY,
        "and the same holds along z");
}

fn void a_height_field_definition_without_heights_is_refused() @test {
    b3::HeightFieldDef def;
    def.scale = (b3::Vec3){ 1.0, 1.0, 1.0 };
    def.count_x = 3;
    def.count_z = 3;
    def.global_minimum_height = -1.0;
    def.global_maximum_height = 1.0;

    assert(the_refusal_of_a_height_field_definition(&def) == b3::INVALID_DEFINITION,
        "box3d reads nine floats through this pointer without testing it");
}

fn void a_height_field_definition_whose_global_range_runs_backwards_is_refused() @test {
    float[9] heights;

    b3::HeightFieldDef def;
    def.heights = &heights[0];
    def.scale = (b3::Vec3){ 1.0, 1.0, 1.0 };
    def.count_x = 3;
    def.count_z = 3;
    def.global_minimum_height = 1.0;
    def.global_maximum_height = -1.0;

    assert(the_refusal_of_a_height_field_definition(&def) == b3::INVALID_DEFINITION,
        "every height would be quantised against a range of negative width");
}

fn fault the_refusal_of_a_grid_of(int row_count, int column_count) {
    b3::HeightFieldData*? field = b3::create_height_field_grid(
      row_count: row_count,
      column_count: column_count,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    );
    defer if (try field) field.destroy();

    if (catch err = field) return err;
    return {};
}

fn fault the_refusal_of_a_wave_of(int row_count, int column_count) {
    b3::HeightFieldData*? field = b3::create_height_field_wave(
      row_count: row_count,
      column_count: column_count,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      row_frequency: 0.5,
      column_frequency: 0.5,
      make_holes: false,
    );
    defer if (try field) field.destroy();

    if (catch err = field) return err;
    return {};
}

fn void a_height_field_builder_too_small_to_hold_a_cell_is_refused() @test {
    fault none;

    assert(the_refusal_of_a_grid_of(row_count: 1, column_count: 16) == b3::DEGENERATE_GEOMETRY,
        "box3d builds a one-row grid with no cell in it; the binding must not");
    assert(the_refusal_of_a_grid_of(row_count: 16, column_count: 1) == b3::DEGENERATE_GEOMETRY);
    assert(the_refusal_of_a_wave_of(row_count: 1, column_count: 16) == b3::DEGENERATE_GEOMETRY,
        "and the wave builder answers the same way");
    assert(the_refusal_of_a_wave_of(row_count: 16, column_count: 1) == b3::DEGENERATE_GEOMETRY);
    assert(the_refusal_of_a_grid_of(row_count: 16, column_count: 16) == none,
        "a grid that does hold cells is built");
}

fn int[2] the_tree_heights_of_two_flat_grid_meshes() {
    b3::MeshData* one_cell = b3::create_grid_mesh(
      x_count: 1,
      z_count: 1,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer one_cell.destroy();

    b3::MeshData* sixteen_cells = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer sixteen_cells.destroy();

    return { one_cell.tree_height(), sixteen_cells.tree_height() };
}

// Both meshes are flat, so any reading of a geometric height would be 0 for both. The two
// numbers differ because what is reported is the depth of the bounding volume hierarchy: two
// triangles fit in the root leaf, thirty-two need three levels below it.
fn void the_reported_height_is_the_hierarchy_depth_and_not_a_dimension() @test {
    int[2] heights = the_tree_heights_of_two_flat_grid_meshes();

    assert(heights[0] == 0, "a one by one grid of cells is a single leaf");
    assert(heights[1] == 3, "a four by four grid of cells is three levels deep");
}

struct RolledBall {
    float x;
    float y;
    float k;
    bool awake;
}

// The grid runs sixteen unit cells along x and z from the origin corner, so a ball dropped at
// (4, 1, 8) is over it for the whole run and leaves it with metres to spare. The control keeps
// the field and drops only the shape, so the two runs differ in exactly one thing. Every
// resource is destroyed here and nothing is asserted: C3 skips defer on a panicking assertion,
// and a stranded height field sits on byte_count() for the rest of the process.
fn RolledBall the_ball_after_two_seconds(bool attach_the_height_field) {
    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 16,
      column_count: 16,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer field.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::ShapeDef shape_def = b3::default_shape_def();

    if (attach_the_height_field) {
        b3::BodyDef ground_def = b3::default_body_def();
        b3::BodyId ground = world.create_body(&ground_def)!!;
        ground.create_height_field_shape(&shape_def, field)!!;
    }

    b3::BodyDef ball_def = b3::default_body_def();
    ball_def.type = b3::BodyType.DYNAMIC;
    ball_def.position = (b3::Pos){ 4.0, 1.0, 8.0 };
    b3::BodyId ball = world.create_body(&ball_def)!!;

    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    ball.create_sphere_shape(&shape_def, &sphere)!!;
    ball.set_linear_velocity((b3::Vec3){ 4.0, 0.0, 0.0 });

    for (int i = 0; i < 120; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    b3::Pos resting = ball.position();
    return {
        .x = resting.x,
        .y = resting.y,
        .k = ball.rotation().k,
        .awake = ball.is_awake(),
    };
}

// The spin is what makes this a rolling test rather than a positional one: a ball skating across
// a frictionless plane would hold the same height and much the same travel and never turn at all,
// so the k component is the half of the reading the geometry has to earn.
fn void a_ball_rolls_across_a_height_field() @test {
    RolledBall rolled = the_ball_after_two_seconds(attach_the_height_field: true);
    RolledBall fell = the_ball_after_two_seconds(attach_the_height_field: false);

    assert(rolled.awake, "a reading taken after the ball fell asleep would not be a reading");
    assert(rolled.x > 10.0 && rolled.x < 10.15, "rolling costs it most of a metre of the travel");
    assert(rolled.y > 0.49 && rolled.y < 0.51, "and leaves it resting its own radius above the field");
    assert(rolled.k > 0.99 && rolled.k < 1.0, "turned about z, which is what a slide would not do");

    assert(fell.awake);
    assert(fell.x > 11.9 && fell.x < 12.1, "with no shape under it the ball keeps all four metres per second");
    assert(fell.y > -19.1 && fell.y < -19.0, "and falls through where the field would have been");
    assert(fell.k > -0.01 && fell.k < 0.01, "with nothing to turn it");
}

fn b3::ShapeType[2] the_types_of_a_mesh_and_a_height_field_shape() {
    b3::Vec3 unit = { 1.0, 1.0, 1.0 };

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 4,
      column_count: 4,
      scale: unit,
      make_holes: false,
    )!!;
    defer field.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    b3::ShapeId mesh_shape = ground.create_mesh_shape(&shape_def, mesh, unit)!!;
    b3::ShapeId field_shape = ground.create_height_field_shape(&shape_def, field)!!;

    return { mesh_shape.type(), field_shape.type() };
}

fn void the_two_geometry_shapes_report_their_own_types() @test {
    b3::ShapeType[2] types = the_types_of_a_mesh_and_a_height_field_shape();

    assert(types[0] == b3::ShapeType.MESH);
    assert(types[1] == b3::ShapeType.HEIGHT_FIELD);
}

fn fault[2] the_refusals_of_a_zeroed_shape_definition() {
    b3::Vec3 unit = { 1.0, 1.0, 1.0 };

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 4,
      column_count: 4,
      scale: unit,
      make_holes: false,
    )!!;
    defer field.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef zeroed;

    fault[2] refusals;
    if (catch err = ground.create_mesh_shape(&zeroed, mesh, unit)) refusals[0] = err;
    if (catch err = ground.create_height_field_shape(&zeroed, field)) refusals[1] = err;
    return refusals;
}

fn void a_geometry_shape_without_the_definition_cookie_is_refused() @test {
    fault[2] refusals = the_refusals_of_a_zeroed_shape_definition();

    assert(refusals[0] == b3::INVALID_DEFINITION,
        "box3d accepts a zeroed shape definition in a release build; the binding must not");
    assert(refusals[1] == b3::INVALID_DEFINITION);
}

fn fault[2] the_refusals_of_a_null_geometry() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    fault[2] refusals;
    if (catch err = ground.create_mesh_shape(&shape_def, null, (b3::Vec3){ 1.0, 1.0, 1.0 })) refusals[0] = err;
    if (catch err = ground.create_height_field_shape(&shape_def, null)) refusals[1] = err;
    return refusals;
}

// The definition here is a real one, so only the null guard can refuse these. Without it box3d
// reads a version field straight through the pointer and the process dies.
fn void a_geometry_shape_built_on_nothing_is_refused() @test {
    fault[2] refusals = the_refusals_of_a_null_geometry();

    assert(refusals[0] == b3::INVALID_DEFINITION, "a mesh shape needs a mesh");
    assert(refusals[1] == b3::INVALID_DEFINITION, "and a height field shape needs a field");
}

// Five readings around one mesh shape. The world is destroyed here rather than in a defer so the
// count after it is gone can be read while the mesh is still alive.
fn int[5] the_byte_counts_around_a_mesh_shape() {
    b3::Vec3 unit = { 1.0, 1.0, 1.0 };
    int baseline = b3::byte_count();

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();
    int with_mesh = b3::byte_count();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    int with_world = b3::byte_count();

    b3::ShapeDef shape_def = b3::default_shape_def();
    ground.create_mesh_shape(&shape_def, mesh, unit)!!;
    int with_shape = b3::byte_count();

    world.destroy();
    return { baseline, with_mesh, with_world, with_shape, b3::byte_count() };
}

fn int[5] the_byte_counts_around_a_height_field_shape() {
    int baseline = b3::byte_count();

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 16,
      column_count: 16,
      scale: (b3::Vec3){ 1.0, 1.0, 1.0 },
      make_holes: false,
    )!!;
    defer field.destroy();
    int with_field = b3::byte_count();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    int with_world = b3::byte_count();

    b3::ShapeDef shape_def = b3::default_shape_def();
    ground.create_height_field_shape(&shape_def, field)!!;
    int with_shape = b3::byte_count();

    world.destroy();
    return { baseline, with_field, with_world, with_shape, b3::byte_count() };
}

// A copy would cost at least what the mesh itself costs, and the world would carry it away. The
// shape costs a fraction of that and the mesh survives the world, which is the whole lifetime
// contract: the shape borrows the pointer, and destroying the mesh first would leave it reading
// freed memory. That last half cannot be pinned by a test without a use-after-free in the suite,
// so it lives in create_mesh_shape's docstring.
fn void a_mesh_shape_borrows_the_mesh_rather_than_copying_it() @test {
    int[5] around = the_byte_counts_around_a_mesh_shape();
    int mesh_bytes = around[1] - around[0];
    int shape_bytes = around[3] - around[2];

    assert(mesh_bytes > 0, "the mesh box3d allocates shows up in the byte count");
    assert(shape_bytes > 0 && shape_bytes < mesh_bytes, "attaching it costs a shape, not a second mesh");
    assert(around[4] == around[1], "and the mesh outlives the world that was reading it");
}

fn void a_height_field_shape_borrows_the_field_rather_than_copying_it() @test {
    int[5] around = the_byte_counts_around_a_height_field_shape();
    int field_bytes = around[1] - around[0];
    int shape_bytes = around[3] - around[2];

    assert(field_bytes > 0, "the field box3d allocates shows up in the byte count");
    assert(shape_bytes > 0 && shape_bytes < field_bytes, "attaching it costs a shape, not a second field");
    assert(around[4] == around[1], "and the field outlives the world that was reading it");
}

fn void the_mesh_a_shape_lends_matches_the_c_layout() @test {
    assert(b3::Mesh::size == 24);
    assert(b3::Mesh::alignment == 8);
}

fn fault[4] the_refusals_of_the_geometry_getters_on_another_type() {
    b3::Vec3 unit = { 1.0, 1.0, 1.0 };

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 4,
      column_count: 4,
      scale: unit,
      make_holes: false,
    )!!;
    defer field.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId sphere_shape = ground.create_sphere_shape(&shape_def, &sphere)!!;
    b3::ShapeId mesh_shape = ground.create_mesh_shape(&shape_def, mesh, unit)!!;
    b3::ShapeId field_shape = ground.create_height_field_shape(&shape_def, field)!!;

    fault[4] refusals;
    if (catch err = sphere_shape.mesh()) refusals[0] = err;
    if (catch err = sphere_shape.height_field()) refusals[1] = err;
    if (catch err = mesh_shape.height_field()) refusals[2] = err;
    if (catch err = field_shape.mesh()) refusals[3] = err;
    return refusals;
}

// box3d reads the union member the name asks for and asserts the type in a debug build only, so
// without these guards a sphere would hand back its centre and radius read as a mesh pointer.
fn void the_geometry_getters_refuse_a_shape_of_another_type() @test {
    fault[4] refusals = the_refusals_of_the_geometry_getters_on_another_type();

    assert(refusals[0] == b3::WRONG_SHAPE_TYPE, "a sphere has no mesh to lend");
    assert(refusals[1] == b3::WRONG_SHAPE_TYPE, "and no height field either");
    assert(refusals[2] == b3::WRONG_SHAPE_TYPE, "a mesh shape is not a height field shape");
    assert(refusals[3] == b3::WRONG_SHAPE_TYPE, "nor the other way about");
}

struct LentGeometry {
    bool mesh_is_the_pointer_it_was_built_on;
    bool field_is_the_pointer_it_was_built_on;
    float[3] scale;
}

fn LentGeometry the_geometry_two_shapes_lend_back() {
    b3::Vec3 unit = { 1.0, 1.0, 1.0 };
    b3::Vec3 stretched = { 2.0, 0.5, 3.0 };

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::HeightFieldData* field = b3::create_height_field_grid(
      row_count: 4,
      column_count: 4,
      scale: unit,
      make_holes: false,
    )!!;
    defer field.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    b3::ShapeId mesh_shape = ground.create_mesh_shape(&shape_def, mesh, stretched)!!;
    b3::ShapeId field_shape = ground.create_height_field_shape(&shape_def, field)!!;

    b3::Mesh lent = mesh_shape.mesh()!!;
    return {
        .mesh_is_the_pointer_it_was_built_on = lent.data == mesh,
        .field_is_the_pointer_it_was_built_on = field_shape.height_field()!! == field,
        .scale = { lent.scale.x, lent.scale.y, lent.scale.z },
    };
}

// The pointer identity is the reading that separates borrowing from copying, and the scale is
// non-uniform so a getter that answered from a default rather than from the shape would show it.
fn void a_shape_lends_back_the_very_geometry_it_was_built_on() @test {
    LentGeometry lent = the_geometry_two_shapes_lend_back();

    assert(lent.mesh_is_the_pointer_it_was_built_on, "the shape borrowed the mesh rather than copying it");
    assert(lent.field_is_the_pointer_it_was_built_on, "and the same holds of the height field");
    assert(lent.scale[0] > 1.99 && lent.scale[0] < 2.01, "the scale comes back as it was given");
    assert(lent.scale[1] > 0.49 && lent.scale[1] < 0.51);
    assert(lent.scale[2] > 2.99 && lent.scale[2] < 3.01);
}

struct MeshMaterials {
    int count;
    float[3] friction;
    ulong[3] user_id;
}

// The count is returned rather than asserted, and the writes are skipped unless it is the three
// the definition asked for: an index used before its bound is checked is the whole hazard here.
fn MeshMaterials the_materials_of_a_three_material_mesh_shape() {
    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 3,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;

    b3::SurfaceMaterial[3] materials;
    for (int i = 0; i < 3; i++) {
        materials[i] = b3::default_surface_material();
    }

    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.materials = &materials[0];
    shape_def.material_count = 3;

    b3::ShapeId shape = ground.create_mesh_shape(&shape_def, mesh, (b3::Vec3){ 1.0, 1.0, 1.0 })!!;

    MeshMaterials read = { .count = shape.mesh_material_count() };
    if (read.count != 3) return read;

    for (int i = 0; i < 3; i++) {
        b3::SurfaceMaterial written = b3::default_surface_material();
        written.friction = 0.125f * (float)(i + 1);
        written.user_material_id = (ulong)(700 + i);
        if (catch shape.set_mesh_material(written, i)) return read;
    }

    for (int i = 0; i < 3; i++) {
        if (try back = shape.mesh_surface_material(i)) {
            read.friction[i] = back.friction;
            read.user_id[i] = back.user_material_id;
        }
    }
    return read;
}

fn void the_material_accessors_round_trip_through_the_index() @test {
    MeshMaterials read = the_materials_of_a_three_material_mesh_shape();

    assert(read.count == 3, "a definition naming three materials gives the shape three");
    assert(read.friction[0] > 0.124 && read.friction[0] < 0.126);
    assert(read.friction[1] > 0.249 && read.friction[1] < 0.251);
    assert(read.friction[2] > 0.374 && read.friction[2] < 0.376);
    assert(read.user_id[0] == 700 && read.user_id[1] == 701 && read.user_id[2] == 702,
        "each index holds what was written to it and not its neighbour's");
}

struct SphereMaterials {
    int count;
    fault set_in_range;
    fault get_in_range;
    fault set_above;
    fault set_below;
    fault get_above;
    fault get_below;
    float friction_by_index;
    float friction_as_the_base;
}

// A sphere shape, because the material accessors carry no type guard and the count they answer
// with is the one every shape has. Writing index 1 of a one-element array is what the guard is
// for: box3d's bounds assertion is compiled out here and the write would land past the end.
fn SphereMaterials the_materials_of_a_sphere_shape() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = ground.create_sphere_shape(&shape_def, &sphere)!!;

    SphereMaterials read = { .count = shape.mesh_material_count() };
    if (read.count != 1) return read;

    b3::SurfaceMaterial written = b3::default_surface_material();
    written.friction = 0.375;
    if (catch err = shape.set_mesh_material(written, 0)) read.set_in_range = err;
    if (catch err = shape.set_mesh_material(written, 1)) read.set_above = err;
    if (catch err = shape.set_mesh_material(written, -1)) read.set_below = err;

    b3::SurfaceMaterial? by_index = shape.mesh_surface_material(0);
    if (catch err = by_index) read.get_in_range = err;
    if (try by_index) read.friction_by_index = by_index.friction;

    if (catch err = shape.mesh_surface_material(1)) read.get_above = err;
    if (catch err = shape.mesh_surface_material(-1)) read.get_below = err;

    read.friction_as_the_base = shape.surface_material().friction;
    return read;
}

// Index 0 is the material set_surface_material() writes, so the base reading is what shows the
// write landed in the shape's own array rather than somewhere that merely reads back.
fn void the_material_accessors_do_not_ask_what_shape_this_is() @test {
    SphereMaterials read = the_materials_of_a_sphere_shape();
    fault none;

    assert(read.count == 1, "a shape built from a definition naming no materials still holds one");
    assert(read.set_in_range == none, "a sphere is not a mesh, and index 0 is written all the same");
    assert(read.get_in_range == none);
    assert(read.friction_by_index > 0.374 && read.friction_by_index < 0.376);
    assert(read.friction_as_the_base > 0.374 && read.friction_as_the_base < 0.376,
        "and index 0 is the base surface material, not a copy of it");
}

fn void an_index_outside_the_material_array_is_refused() @test {
    SphereMaterials read = the_materials_of_a_sphere_shape();

    assert(read.count == 1, "one material, so 1 is the first index past the end");
    assert(read.set_above == b3::INDEX_OUT_OF_RANGE, "writing past the end would corrupt the shape");
    assert(read.set_below == b3::INDEX_OUT_OF_RANGE, "and a negative index writes before its start");
    assert(read.get_above == b3::INDEX_OUT_OF_RANGE, "reading past the end returns whatever follows it");
    assert(read.get_below == b3::INDEX_OUT_OF_RANGE);
}

struct ReplacedGeometry {
    b3::ShapeType type_before;
    b3::ShapeType type_after;
    bool lends_the_new_mesh;
    int material_count_before;
    int material_count_after;
    float mass_before;
    float mass_after_set;
    float mass_after_apply;
}

fn ReplacedGeometry the_shape_around_a_replaced_geometry() {
    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ball_def = b3::default_body_def();
    ball_def.type = b3::BodyType.DYNAMIC;
    b3::BodyId ball = world.create_body(&ball_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = ball.create_sphere_shape(&shape_def, &sphere)!!;

    ReplacedGeometry replaced = {
        .type_before = shape.type(),
        .material_count_before = shape.mesh_material_count(),
        .mass_before = ball.mass(),
    };

    shape.set_mesh(mesh, (b3::Vec3){ 1.0, 1.0, 1.0 })!!;

    replaced.type_after = shape.type();
    replaced.material_count_after = shape.mesh_material_count();
    replaced.mass_after_set = ball.mass();
    if (try lent = shape.mesh()) replaced.lends_the_new_mesh = lent.data == mesh;

    ball.apply_mass_from_shapes();
    replaced.mass_after_apply = ball.mass();
    return replaced;
}

// The sphere is half a metre across at box3d's default density, which is where the mass band
// comes from. It has to be read twice: unchanged across the replacement, and gone once the body
// is asked to recompute, because a mesh contributes none.
fn void a_replaced_geometry_carries_the_shape_type_and_leaves_the_mass_alone() @test {
    ReplacedGeometry replaced = the_shape_around_a_replaced_geometry();

    assert(replaced.type_before == b3::ShapeType.SPHERE);
    assert(replaced.type_after == b3::ShapeType.MESH, "the geometry a shape is given is also its type");
    assert(replaced.lends_the_new_mesh, "and mesh() reports the mesh it was just handed");
    assert(replaced.material_count_before == 1 && replaced.material_count_after == 1,
        "replacing the geometry leaves the material array as it was");
    assert(replaced.mass_before > 523.0 && replaced.mass_before < 524.0);
    assert(replaced.mass_after_set > 523.0 && replaced.mass_after_set < 524.0,
        "which the body keeps until it is asked to recompute");
    assert(replaced.mass_after_apply > -0.01 && replaced.mass_after_apply < 0.01,
        "and then a mesh leaves it with none");
}

// The world is destroyed here rather than in a defer so the count with both meshes still alive
// can be read after it is gone. Both meshes are destroyed by the defers, which is the reading
// the caller takes: the replaced one was still there to destroy.
fn int[5] the_byte_counts_around_a_replaced_mesh() {
    b3::Vec3 unit = { 1.0, 1.0, 1.0 };
    int baseline = b3::byte_count();

    b3::MeshData* first = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer first.destroy();

    b3::MeshData* second = b3::create_grid_mesh(
      x_count: 8,
      z_count: 8,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer second.destroy();
    int with_both = b3::byte_count();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::ShapeId shape = ground.create_mesh_shape(&shape_def, first, unit)!!;
    int with_shape = b3::byte_count();

    shape.set_mesh(second, unit)!!;
    int after_replacing = b3::byte_count();

    world.destroy();
    return { baseline, with_both, with_shape, after_replacing, b3::byte_count() };
}

// Replacing the geometry frees nothing: box3d drops the pointer it was holding and takes the new
// one, so a caller that replaces a mesh is left owning both and owes destroy() on each.
fn void a_replaced_mesh_is_still_the_callers_to_destroy() @test {
    int[5] around = the_byte_counts_around_a_replaced_mesh();

    assert(around[1] > around[0], "the two meshes box3d allocates show up in the byte count");
    assert(around[3] == around[2], "and replacing one with the other gives nothing back");
    assert(around[4] == around[1], "both outlive the world that was reading one of them");
    assert(b3::byte_count() == around[0], "and both were still there to destroy");
}

fn fault the_refusal_of_a_replacement_that_is_not_there() {
    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };
    b3::ShapeId shape = ground.create_sphere_shape(&shape_def, &sphere)!!;

    if (catch err = shape.set_mesh(null, (b3::Vec3){ 1.0, 1.0, 1.0 })) return err;
    return {};
}

// The shape here is a real one, so only the null guard can refuse this. Without it box3d takes
// the pointer and computes the shape's new bounds through it, and the process dies.
fn void a_geometry_replaced_by_nothing_is_refused() @test {
    assert(the_refusal_of_a_replacement_that_is_not_there() == b3::INVALID_DEFINITION,
        "a mesh shape needs a mesh, whether it is built or replaced");
}
