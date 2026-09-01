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
// (4, 1, 8) is over it for the whole run and leaves it with metres to spare. The control builds
// the field but no ground body at all, which is still the one-variable change the reading needs:
// a static body carrying no shape has no proxy and makes no contact either. Every resource is
// destroyed here and nothing is asserted: C3 skips defer on a panicking assertion, and a
// stranded height field sits on byte_count() for the rest of the process.
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

// Only the height band would take a frictionless skater. Measured on this fixture with the base
// material's friction zeroed, it keeps the whole ballistic travel — 11.999992, the control's
// number to the bit — and never turns, so the travel band and the spin band each reject it on
// their own. Neither is redundant to the other.
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

// A sphere and a capsule child sit where their own geometry puts them, so the children below take
// the identity transform.
const b3::Transform CHILD_TRANSFORM = {
  .p = { 0.0, 0.0, 0.0 },
  .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
};

fn void the_compound_definition_types_match_the_c_layout() @test {
    assert(b3::CompoundDef::size == 64);
    assert(b3::CompoundDef::alignment == 8);
    assert(b3::CompoundSphereDef::size == 56);
    assert(b3::CompoundCapsuleDef::size == 72);
    assert(b3::CompoundHullDef::size == 80);
    assert(b3::CompoundMeshDef::size == 64);
}

// Same shape as the mesh helpers above: the count is read while the compound is alive, the
// caller reads it again once the defer has run, and nothing here asserts, because a panic would
// strand the compound on byte_count() for the rest of the process.
fn int[2] the_byte_count_around_a_live_two_sphere_compound() {
    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { -0.5, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
      {
        .sphere = { .center = { 0.5, 0.0, 0.0 }, .radius = 0.5 },
        .material = b3::default_surface_material(),
      },
    };

    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;

    int baseline = b3::byte_count();

    b3::CompoundData* compound = b3::create_compound(&def)!!;
    defer compound.destroy();

    return { baseline, b3::byte_count() };
}

fn void a_destroyed_compound_gives_back_every_byte_it_took() @test {
    int[2] around = the_byte_count_around_a_live_two_sphere_compound();

    assert(around[1] > around[0], "a compound box3d bakes must show up in the byte count");
    assert(b3::byte_count() == around[0], "and destroying it must give every byte back");
}

struct BuiltCompound {
    bool built;
    int baseline;
    int live;
}

fn BuiltCompound the_compound_an_empty_definition_builds() {
    b3::CompoundDef empty;

    int baseline = b3::byte_count();

    b3::CompoundData*? compound = b3::create_compound(&empty);
    defer if (try compound) compound.destroy();

    return {
      .built = @ok(compound),
      .baseline = baseline,
      .live = b3::byte_count(),
    };
}

// A definition naming no children is a request box3d honours, not one it turns away — which is
// the opposite of what create_mesh() does with a definition naming no triangles. The compound
// that comes back is empty and is owned like any other.
fn void an_empty_compound_definition_is_accepted() @test {
    BuiltCompound empty = the_compound_an_empty_definition_builds();

    assert(empty.built, "a compound with no children is still a compound");
    assert(empty.live > empty.baseline, "which box3d allocates for like any other");
    assert(b3::byte_count() == empty.baseline, "and which the caller still owes destroy() on");
}

// The mesh outlives the compound here only because the defers unwind in reverse: box3d clones
// the mesh into the compound, so the order is a convention of this fixture, not a requirement.
fn BuiltCompound the_compound_every_child_kind_builds() {
    b3::SurfaceMaterial material = b3::default_surface_material();
    b3::BoxHull cube = b3::make_cube_hull(0.5);

    int baseline = b3::byte_count();

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::CompoundSphereDef[1] spheres = {
      {
        .sphere = { .center = { -1.0, 0.0, 0.0 }, .radius = 0.5 },
        .material = material,
      },
    };
    b3::CompoundCapsuleDef[1] capsules = {
      {
        .capsule = { .center1 = { 1.0, -0.5, 0.0 }, .center2 = { 1.0, 0.5, 0.0 }, .radius = 0.25 },
        .material = material,
      },
    };
    b3::CompoundHullDef[1] hulls = {
      {
        .hull = &cube.base,
        .transform = CHILD_TRANSFORM,
        .material = material,
      },
    };
    b3::CompoundMeshDef[1] meshes = {
      {
        .mesh_data = mesh,
        .transform = CHILD_TRANSFORM,
        .scale = { 1.0, 1.0, 1.0 },
        .materials = &material,
        .material_count = 1,
      },
    };

    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 1;
    def.capsules = &capsules[0];
    def.capsule_count = 1;
    def.hulls = &hulls[0];
    def.hull_count = 1;
    def.meshes = &meshes[0];
    def.mesh_count = 1;

    b3::CompoundData*? compound = b3::create_compound(&def);
    defer if (try compound) compound.destroy();

    return {
      .built = @ok(compound),
      .baseline = baseline,
      .live = b3::byte_count(),
    };
}

// One child of each kind, so every definition struct the binding declares is filled in and
// handed to box3d at least once. What the children report is the child accessors' business.
fn void a_compound_takes_a_child_of_every_kind() @test {
    BuiltCompound every = the_compound_every_child_kind_builds();

    assert(every.built, "spheres, capsules, hulls and meshes bake into one compound");
    assert(every.live > every.baseline, "which box3d allocates for");
    assert(b3::byte_count() == every.baseline, "and which gives its bytes back, mesh included");
}

fn Refusal the_refusal_of_a_compound_definition(b3::CompoundDef* def) {
    int baseline = b3::byte_count();

    b3::CompoundData*? compound = b3::create_compound(def);
    defer if (try compound) compound.destroy();

    if (catch err = compound) return { .refusal = err, .baseline = baseline, .after = b3::byte_count() };
    return { .refusal = {}, .baseline = baseline, .after = b3::byte_count() };
}

// box3d reads each child array without testing it, so without these guards the definition is a
// dead process rather than a refusal. The byte count is the evidence nothing was taken first.
fn void a_compound_definition_naming_an_array_it_does_not_carry_is_refused() @test {
    b3::CompoundDef spheres;
    spheres.sphere_count = 2;
    b3::CompoundDef capsules;
    capsules.capsule_count = 2;
    b3::CompoundDef hulls;
    hulls.hull_count = 2;
    b3::CompoundDef meshes;
    meshes.mesh_count = 2;

    Refusal no_spheres = the_refusal_of_a_compound_definition(&spheres);

    assert(no_spheres.refusal == b3::INVALID_DEFINITION, "two spheres need somewhere to read two spheres from");
    assert(no_spheres.after == no_spheres.baseline, "and the definition is turned away before allocating");
    assert(the_refusal_of_a_compound_definition(&capsules).refusal == b3::INVALID_DEFINITION);
    assert(the_refusal_of_a_compound_definition(&hulls).refusal == b3::INVALID_DEFINITION);
    assert(the_refusal_of_a_compound_definition(&meshes).refusal == b3::INVALID_DEFINITION);
}

// A negative count passes box3d's only check and then becomes the byte count of an allocation,
// which is where the process ends.
fn void a_compound_definition_with_a_negative_count_is_refused() @test {
    b3::CompoundDef spheres;
    spheres.sphere_count = -1;
    b3::CompoundDef capsules;
    capsules.capsule_count = -1;
    b3::CompoundDef hulls;
    hulls.hull_count = -1;
    b3::CompoundDef meshes;
    meshes.mesh_count = -1;

    Refusal refused = the_refusal_of_a_compound_definition(&spheres);

    assert(refused.refusal == b3::INVALID_DEFINITION, "a count below zero names no children");
    assert(refused.after == refused.baseline, "and is turned away before allocating");
    assert(the_refusal_of_a_compound_definition(&capsules).refusal == b3::INVALID_DEFINITION);
    assert(the_refusal_of_a_compound_definition(&hulls).refusal == b3::INVALID_DEFINITION);
    assert(the_refusal_of_a_compound_definition(&meshes).refusal == b3::INVALID_DEFINITION);
}

fn void a_compound_hull_child_naming_no_hull_is_refused() @test {
    b3::CompoundHullDef[1] hulls = {
      {
        .hull = null,
        .transform = CHILD_TRANSFORM,
        .material = b3::default_surface_material(),
      },
    };

    b3::CompoundDef def;
    def.hulls = &hulls[0];
    def.hull_count = 1;

    Refusal refused = the_refusal_of_a_compound_definition(&def);

    assert(refused.refusal == b3::INVALID_DEFINITION, "box3d computes the child's bounds through this pointer");
    assert(refused.after == refused.baseline, "and the definition is turned away before allocating");
}

// The same dereference the hull child guard stands in for, one level down: box3d computes the
// child's bounds through the mesh pointer and reads the material array without testing either.
fn void a_compound_mesh_child_naming_no_geometry_is_refused() @test {
    b3::SurfaceMaterial material = b3::default_surface_material();

    b3::CompoundMeshDef[1] no_mesh = {
      {
        .mesh_data = null,
        .transform = CHILD_TRANSFORM,
        .scale = { 1.0, 1.0, 1.0 },
        .materials = &material,
        .material_count = 1,
      },
    };
    b3::CompoundMeshDef[1] no_materials = {
      {
        .mesh_data = null,
        .transform = CHILD_TRANSFORM,
        .scale = { 1.0, 1.0, 1.0 },
        .materials = null,
        .material_count = 1,
      },
    };

    b3::CompoundDef missing_mesh;
    missing_mesh.meshes = &no_mesh[0];
    missing_mesh.mesh_count = 1;
    b3::CompoundDef missing_materials;
    missing_materials.meshes = &no_materials[0];
    missing_materials.mesh_count = 1;

    Refusal refused = the_refusal_of_a_compound_definition(&missing_mesh);

    assert(refused.refusal == b3::INVALID_DEFINITION, "a mesh child needs a mesh");
    assert(refused.after == refused.baseline, "and is turned away before allocating");
    assert(the_refusal_of_a_compound_definition(&missing_materials).refusal == b3::INVALID_DEFINITION,
        "and the materials its triangles index into");
}

fn Refusal the_refusal_of_a_mesh_child_carrying(int material_count) {
    b3::SurfaceMaterial[8] materials;
    for (int i = 0; i < 8; i++) materials[i] = b3::default_surface_material();

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 1,
      identify_edges: true,
    )!!;
    defer mesh.destroy();

    b3::CompoundMeshDef[1] meshes = {
      {
        .mesh_data = mesh,
        .transform = CHILD_TRANSFORM,
        .scale = { 1.0, 1.0, 1.0 },
        .materials = &materials[0],
        .material_count = material_count,
      },
    };

    b3::CompoundDef def;
    def.meshes = &meshes[0];
    def.mesh_count = 1;

    return the_refusal_of_a_compound_definition(&def);
}

// box3d writes one index per material into a fixed array of MAX_COMPOUND_MESH_MATERIALS and
// keeps writing past the end of it for a larger count, so this refusal stands in for a heap it
// would otherwise corrupt. A count of zero is refused too: box3d asserts against it, and the
// assertion is compiled out of this build.
fn void a_compound_mesh_child_carrying_the_wrong_number_of_materials_is_refused() @test {
    fault none;

    Refusal too_many = the_refusal_of_a_mesh_child_carrying(b3::MAX_COMPOUND_MESH_MATERIALS + 1);
    Refusal no_materials = the_refusal_of_a_mesh_child_carrying(0);
    Refusal allowed = the_refusal_of_a_mesh_child_carrying(b3::MAX_COMPOUND_MESH_MATERIALS);

    assert(too_many.refusal == b3::INVALID_DEFINITION, "a mesh child has room for four materials");
    assert(too_many.after == too_many.baseline, "and is turned away before allocating");
    assert(no_materials.refusal == b3::INVALID_DEFINITION, "and needs at least one");
    assert(allowed.refusal == none, "while a count between the two bounds builds");
}

// The one definition box3d itself refuses, and it refuses on the counts alone: the arrays are
// never read, which is what makes a test for it affordable.
fn void a_compound_definition_naming_more_children_than_box3d_holds_is_refused() @test {
    b3::CompoundDef def;
    def.sphere_count = b3::MAX_CHILD_SHAPES / 2;
    def.capsule_count = b3::MAX_CHILD_SHAPES / 2;

    Refusal refused = the_refusal_of_a_compound_definition(&def);

    assert(refused.refusal == b3::CHILD_LIMIT_REACHED, "the ceiling is on the children together, not per kind");
    assert(refused.after == refused.baseline, "and it is reached before anything is allocated");
}

// Two spheres that differ in everything a child can differ in: where they sit, how big they are
// and how rough their surface is. A compound that swapped them would then be caught by the child
// index, by the material index and by the hit point alike, rather than by none of the three.
fn b3::CompoundData*? a_compound_of_two_unlike_spheres() {
    b3::SurfaceMaterial rough = b3::default_surface_material();
    rough.friction = 0.3;
    b3::SurfaceMaterial slick = b3::default_surface_material();
    slick.friction = 0.9;

    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { -0.5, 0.0, 0.0 }, .radius = 0.5 },
        .material = rough,
      },
      {
        .sphere = { .center = { 1.5, 0.0, 0.0 }, .radius = 0.25 },
        .material = slick,
      },
    };

    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;

    return b3::create_compound(&def);
}

struct AttachedCompound {
    bool attached;
    fault refusal;
    b3::ShapeType type;
}

fn AttachedCompound the_compound_shape_a_body_of_type_takes(b3::BodyType type) {
    b3::CompoundData* compound = a_compound_of_two_unlike_spheres()!!;
    defer compound.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef body_def = b3::default_body_def();
    body_def.type = type;
    b3::BodyId body = world.create_body(&body_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();

    b3::ShapeId? shape = body.create_baked_compound_shape(&shape_def, compound);
    if (catch err = shape) return { .refusal = err };
    return { .attached = true, .type = shape.type() };
}

// The three readings differ in the body type and in nothing else, which is what makes the refusal
// attributable to it. A caller who does not know the constraint gets a null identifier back and
// nothing else to go on.
fn void a_baked_compound_shape_is_taken_only_by_a_static_body() @test {
    AttachedCompound on_static = the_compound_shape_a_body_of_type_takes(b3::BodyType.STATIC);
    AttachedCompound on_kinematic = the_compound_shape_a_body_of_type_takes(b3::BodyType.KINEMATIC);
    AttachedCompound on_dynamic = the_compound_shape_a_body_of_type_takes(b3::BodyType.DYNAMIC);

    assert(on_static.attached, "a static body bakes a compound into one shape");
    assert(!on_kinematic.attached, "a kinematic one is turned away");
    assert(on_kinematic.refusal == b3::INVALID_HANDLE, "with nothing but a null identifier to say so");
    assert(!on_dynamic.attached, "and so is a dynamic one, which is how the obvious fixture fails");
    assert(on_dynamic.refusal == b3::INVALID_HANDLE);
}

fn void a_baked_compound_shape_reports_its_own_type() @test {
    AttachedCompound on_static = the_compound_shape_a_body_of_type_takes(b3::BodyType.STATIC);

    assert(on_static.attached);
    assert(on_static.type == b3::ShapeType.COMPOUND, "one shape, whatever its children are");
}

struct CompoundCast {
    bool hit;
    int child_index;
    int material_index;
    float y;
    float normal_x;
    float normal_y;
    float normal_z;
}

struct CompoundShapeReading {
    int material_count;
    float[2] friction;
    CompoundCast over_the_wide_child;
    CompoundCast over_the_narrow_child;
    bool hit_between_them;
}

fn CompoundCast the_cast_straight_down_at(b3::ShapeId shape, float x) {
    b3::WorldCastOutput? hit = shape.ray_cast((b3::Pos){ x, 3.0, 0.0 }, (b3::Vec3){ 0.0, -6.0, 0.0 });
    if (try hit) {
        return {
          .hit = true,
          .child_index = hit.child_index,
          .material_index = hit.material_index,
          .y = hit.point.y,
          .normal_x = hit.normal.x,
          .normal_y = hit.normal.y,
          .normal_z = hit.normal.z,
        };
    }
    return {};
}

// Three rays and the shape's own material array. The materials are only read once the count says
// there are two of them: an index used before its bound is checked is the hazard the index guard
// exists for, and a helper that asserted instead would strand the compound on byte_count().
fn CompoundShapeReading the_children_a_compound_shape_reports() {
    b3::CompoundData* compound = a_compound_of_two_unlike_spheres()!!;
    defer compound.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::ShapeId shape = ground.create_baked_compound_shape(&shape_def, compound)!!;

    CompoundShapeReading read = {
      .material_count = shape.mesh_material_count(),
      .over_the_wide_child = the_cast_straight_down_at(shape, -0.5),
      .over_the_narrow_child = the_cast_straight_down_at(shape, 1.5),
      .hit_between_them = the_cast_straight_down_at(shape, 0.5).hit,
    };

    if (read.material_count == 2) {
        for (int i = 0; i < 2; i++) {
            if (try material = shape.mesh_surface_material(i)) read.friction[i] = material.friction;
        }
    }
    return read;
}

// A compound shape reports the children it was built from. Each ray is answered by one child and
// names it, and the gap between the two is answered by neither, so the shape is the pair of
// children rather than anything covering both.
fn void a_compound_shape_reports_the_children_it_was_built_from() @test {
    CompoundShapeReading read = the_children_a_compound_shape_reports();

    assert(read.over_the_wide_child.hit, "a ray dropped over the first sphere meets it");
    assert(read.over_the_wide_child.child_index == 0, "and is answered by the first child");
    assert(read.over_the_wide_child.y > 0.49 && read.over_the_wide_child.y < 0.51,
        "at its own radius above the centre the definition gave it");
    assert(read.over_the_wide_child.normal_y > 0.99 && read.over_the_wide_child.normal_y < 1.01,
        "square on the top of the sphere");
    assert(read.over_the_wide_child.normal_x > -0.01 && read.over_the_wide_child.normal_x < 0.01,
        "with nothing sideways in it, which a transposed vector would show");
    assert(read.over_the_wide_child.normal_z > -0.01 && read.over_the_wide_child.normal_z < 0.01);

    assert(read.over_the_narrow_child.hit, "and the second sphere is two metres away");
    assert(read.over_the_narrow_child.child_index == 1, "answered by the second child");
    assert(read.over_the_narrow_child.y > 0.24 && read.over_the_narrow_child.y < 0.26,
        "at half the first one's radius, which a swap of the two would not report");
    assert(read.over_the_narrow_child.normal_y > 0.99 && read.over_the_narrow_child.normal_y < 1.01);
    assert(read.over_the_narrow_child.normal_x > -0.01 && read.over_the_narrow_child.normal_x < 0.01);
    assert(read.over_the_narrow_child.normal_z > -0.01 && read.over_the_narrow_child.normal_z < 0.01);

    assert(!read.hit_between_them, "and the gap between them belongs to neither child");

    assert(read.material_count == 2, "one material per distinct child surface");
    assert(read.friction[0] > 0.29 && read.friction[0] < 0.31, "the rough sphere's, first");
    assert(read.friction[1] > 0.89 && read.friction[1] < 0.91, "and the slick one's after it");
    assert(read.over_the_wide_child.material_index == 0, "each child naming the index of its own");
    assert(read.over_the_narrow_child.material_index == 1);
}

fn fault[2] the_refusals_of_a_baked_compound_shape() {
    b3::CompoundData* compound = a_compound_of_two_unlike_spheres()!!;
    defer compound.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;

    b3::ShapeDef zeroed;
    b3::ShapeDef shape_def = b3::default_shape_def();

    fault[2] refusals;
    if (catch err = ground.create_baked_compound_shape(&zeroed, compound)) refusals[0] = err;
    if (catch err = ground.create_baked_compound_shape(&shape_def, null)) refusals[1] = err;
    return refusals;
}

// The body here is a static one, so neither refusal can be the body type's. Without the null
// guard box3d reads the compound's version field straight through the pointer and the process
// dies, exactly as it does for a mesh.
fn void a_baked_compound_shape_without_a_definition_or_a_compound_is_refused() @test {
    fault[2] refusals = the_refusals_of_a_baked_compound_shape();

    assert(refusals[0] == b3::INVALID_DEFINITION,
        "box3d accepts a zeroed shape definition in a release build; the binding must not");
    assert(refusals[1] == b3::INVALID_DEFINITION, "and a compound shape needs a compound");
}

fn int[5] the_byte_counts_around_a_compound_shape() {
    int baseline = b3::byte_count();

    b3::CompoundData* compound = a_compound_of_two_unlike_spheres()!!;
    defer compound.destroy();
    int with_compound = b3::byte_count();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    b3::BodyDef ground_def = b3::default_body_def();
    b3::BodyId ground = world.create_body(&ground_def)!!;
    int with_world = b3::byte_count();

    b3::ShapeDef shape_def = b3::default_shape_def();
    ground.create_baked_compound_shape(&shape_def, compound)!!;
    int with_shape = b3::byte_count();

    world.destroy();
    return { baseline, with_compound, with_world, with_shape, b3::byte_count() };
}

fn void a_compound_shape_borrows_the_compound_rather_than_copying_it() @test {
    int[5] around = the_byte_counts_around_a_compound_shape();
    int compound_bytes = around[1] - around[0];
    int shape_bytes = around[3] - around[2];

    assert(compound_bytes > 0, "the compound box3d bakes shows up in the byte count");
    assert(shape_bytes > 0 && shape_bytes < compound_bytes, "attaching it costs a shape, not a second compound");
    assert(around[4] == around[1], "and the compound outlives the world that was reading it");
}

struct RestingBalls {
    float low_x;
    float low_y;
    bool low_asleep;
    float high_x;
    float high_y;
    bool high_asleep;
}

// Two box platforms baked into one compound, four metres apart and at different heights, with a
// ball dropped over each. A compound of the two spheres above would be no fixture for this: a
// ball landing on a sphere rolls off it. The control attaches no shape at all, which is the one
// variable this reading needs — a static body carrying nothing makes no contact either.
fn RestingBalls the_balls_after_three_seconds(bool attach_the_compound) {
    b3::SurfaceMaterial material = b3::default_surface_material();
    b3::BoxHull low = b3::make_box_hull(1.0, 0.25, 1.0);
    b3::BoxHull high = b3::make_box_hull(1.0, 0.5, 1.0);

    b3::CompoundHullDef[2] hulls = {
      {
        .hull = &low.base,
        .transform = { .p = { -2.0, 0.0, 0.0 }, .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 } },
        .material = material,
      },
      {
        .hull = &high.base,
        .transform = { .p = { 2.0, 1.0, 0.0 }, .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 } },
        .material = material,
      },
    };

    b3::CompoundDef def;
    def.hulls = &hulls[0];
    def.hull_count = 2;

    b3::CompoundData* compound = b3::create_compound(&def)!!;
    defer compound.destroy();

    b3::WorldDef world_def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&world_def)!!;
    defer world.destroy();

    b3::ShapeDef shape_def = b3::default_shape_def();

    if (attach_the_compound) {
        b3::BodyDef ground_def = b3::default_body_def();
        b3::BodyId ground = world.create_body(&ground_def)!!;
        ground.create_baked_compound_shape(&shape_def, compound)!!;
    }

    b3::Sphere sphere = { .center = { 0.0, 0.0, 0.0 }, .radius = 0.5 };

    b3::BodyDef over_the_low_def = b3::default_body_def();
    over_the_low_def.type = b3::BodyType.DYNAMIC;
    over_the_low_def.position = (b3::Pos){ -2.0, 3.0, 0.0 };
    b3::BodyId over_the_low = world.create_body(&over_the_low_def)!!;
    over_the_low.create_sphere_shape(&shape_def, &sphere)!!;

    b3::BodyDef over_the_high_def = b3::default_body_def();
    over_the_high_def.type = b3::BodyType.DYNAMIC;
    over_the_high_def.position = (b3::Pos){ 2.0, 4.0, 0.0 };
    b3::BodyId over_the_high = world.create_body(&over_the_high_def)!!;
    over_the_high.create_sphere_shape(&shape_def, &sphere)!!;

    for (int i = 0; i < 180; i++) {
        world.step(1.0f / 60.0f, 4);
    }

    b3::Pos low_resting = over_the_low.position();
    b3::Pos high_resting = over_the_high.position();
    return {
      .low_x = low_resting.x,
      .low_y = low_resting.y,
      .low_asleep = !over_the_low.is_awake(),
      .high_x = high_resting.x,
      .high_y = high_resting.y,
      .high_asleep = !over_the_high.is_awake(),
    };
}

// The two resting heights are a metre and a quarter apart, and each is its own platform's top
// plus the ball's radius, so a compound that had baked only one of the children would fail one
// band while passing the other.
fn void a_ball_comes_to_rest_on_the_compound_child_beneath_it() @test {
    RestingBalls rested = the_balls_after_three_seconds(attach_the_compound: true);
    RestingBalls fell = the_balls_after_three_seconds(attach_the_compound: false);

    assert(rested.low_asleep && rested.high_asleep, "a reading taken while they were still moving would not be a reading");
    assert(rested.low_y > 0.74 && rested.low_y < 0.76, "the quarter-metre platform holds its ball at three quarters");
    assert(rested.high_y > 1.99 && rested.high_y < 2.01, "and the half-metre one, a metre higher, holds its own at two");
    assert(rested.low_x > -2.01 && rested.low_x < -1.99, "each ball resting where it was dropped");
    assert(rested.high_x > 1.99 && rested.high_x < 2.01);

    assert(!fell.low_asleep && !fell.high_asleep, "with no shape under them neither ball ever settles");
    assert(fell.low_y > -42.2 && fell.low_y < -41.9, "and both fall the three seconds through");
    assert(fell.high_y > -41.2 && fell.high_y < -40.9);
}

const b3::Transform HULL_CHILD_TRANSFORM = {
  .p = { 7.0, 2.0, -1.0 },
  .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
};

const b3::Transform MESH_CHILD_TRANSFORM = {
  .p = { -5.0, 3.0, 0.0 },
  .q = { .i = 0.0, .j = 0.0, .k = 0.0, .l = 1.0 },
};

// One child of every kind, each placed somewhere different and each naming a material. The
// definition is filled in spheres first and capsules last, so a reading that followed the order
// the caller wrote rather than the order box3d bakes would report the spheres first. Three
// distinct materials serve the five children: the hull takes the first sphere's and the mesh's
// second slot takes the second sphere's, so the indices also show the sharing.
fn b3::CompoundData*? a_compound_of_one_child_of_every_kind() {
    b3::SurfaceMaterial rough = b3::default_surface_material();
    rough.friction = 0.3;
    b3::SurfaceMaterial slick = b3::default_surface_material();
    slick.friction = 0.9;
    b3::SurfaceMaterial mid = b3::default_surface_material();
    mid.friction = 0.55;

    b3::BoxHull cube = b3::make_cube_hull(0.5);

    b3::MeshData* mesh = b3::create_grid_mesh(
      x_count: 4,
      z_count: 4,
      cell_width: 1.0,
      material_count: 2,
      identify_edges: true,
    )!;
    defer mesh.destroy();

    b3::SurfaceMaterial[2] mesh_materials = { mid, slick };

    b3::CompoundSphereDef[2] spheres = {
      {
        .sphere = { .center = { -3.0, 1.0, 2.0 }, .radius = 0.5 },
        .material = rough,
      },
      {
        .sphere = { .center = { 4.0, 0.0, 0.0 }, .radius = 0.25 },
        .material = slick,
      },
    };
    b3::CompoundMeshDef[1] meshes = {
      {
        .mesh_data = mesh,
        .transform = MESH_CHILD_TRANSFORM,
        .scale = { 2.0, 1.0, 3.0 },
        .materials = &mesh_materials[0],
        .material_count = 2,
      },
    };
    b3::CompoundHullDef[1] hulls = {
      {
        .hull = &cube.base,
        .transform = HULL_CHILD_TRANSFORM,
        .material = rough,
      },
    };
    b3::CompoundCapsuleDef[1] capsules = {
      {
        .capsule = { .center1 = { 1.0, -0.5, 0.0 }, .center2 = { 1.0, 0.5, 0.0 }, .radius = 0.25 },
        .material = mid,
      },
    };

    b3::CompoundDef def;
    def.spheres = &spheres[0];
    def.sphere_count = 2;
    def.meshes = &meshes[0];
    def.mesh_count = 1;
    def.hulls = &hulls[0];
    def.hull_count = 1;
    def.capsules = &capsules[0];
    def.capsule_count = 1;

    return b3::create_compound(&def);
}

struct FlatChild {
    b3::ShapeType type;
    float transform_x;
    int[b3::MAX_COMPOUND_MESH_MATERIALS] material_indices;
}

struct FlatChildren {
    int capsule_count;
    int hull_count;
    int mesh_count;
    int sphere_count;
    int child_count;
    FlatChild[5] children;
    float capsule_radius;
    float sphere_center_x;
}

fn FlatChildren the_children_a_compound_reports_in_order() {
    b3::CompoundData* compound = a_compound_of_one_child_of_every_kind()!!;
    defer compound.destroy();

    FlatChildren read = {
      .capsule_count = compound.capsule_count,
      .hull_count = compound.hull_count,
      .mesh_count = compound.mesh_count,
      .sphere_count = compound.sphere_count,
      .child_count = compound.child_count(),
    };

    for (int i = 0; i < read.child_count && i < read.children.len; i++) {
        if (try child = compound.child(i)) {
            read.children[i] = {
              .type = child.type,
              .transform_x = child.transform.p.x,
              .material_indices = child.material_indices,
            };
        }
    }

    if (try capsule = compound.child(0)) read.capsule_radius = capsule.capsule.radius;
    if (try sphere = compound.child(3)) read.sphere_center_x = sphere.sphere.center.x;
    return read;
}

// The flat index runs capsules, hulls, meshes, spheres, whatever order the definition was filled
// in. A capsule and a sphere get the identity transform however they are placed, because their
// placement lives in their own geometry; only the hull and the mesh carry one of their own.
fn void the_flat_child_index_runs_in_the_order_box3d_baked() @test {
    FlatChildren read = the_children_a_compound_reports_in_order();

    assert(read.capsule_count == 1 && read.hull_count == 1);
    assert(read.mesh_count == 1 && read.sphere_count == 2);
    assert(read.child_count == 5, "the flat index runs over every kind together");

    assert(read.children[0].type == b3::ShapeType.CAPSULE, "the capsule the definition was given last comes first");
    assert(read.children[1].type == b3::ShapeType.HULL);
    assert(read.children[2].type == b3::ShapeType.MESH);
    assert(read.children[3].type == b3::ShapeType.SPHERE, "and the spheres it was given first come last");
    assert(read.children[4].type == b3::ShapeType.SPHERE);

    assert(read.children[0].transform_x > -0.01 && read.children[0].transform_x < 0.01,
        "a capsule child is placed by its own two centres, so its transform is the identity one");
    assert(read.children[3].transform_x > -0.01 && read.children[3].transform_x < 0.01,
        "and so is a sphere child's, three metres from the origin though this one sits");
    assert(read.children[1].transform_x > 6.99 && read.children[1].transform_x < 7.01,
        "where a hull child carries the transform the definition gave it");
    assert(read.children[2].transform_x > -5.01 && read.children[2].transform_x < -4.99,
        "and so does a mesh child");

    assert(read.capsule_radius > 0.24 && read.capsule_radius < 0.26,
        "the arm the tag names carries the child's own geometry");
    assert(read.sphere_center_x > -3.01 && read.sphere_center_x < -2.99,
        "the sphere's centre being the placement its identity transform does not carry");
}

// A mesh child fills every slot it has and the ones it never uses read 0, which is a real
// material index rather than a marker, so the child cannot say how many of them mean anything.
// The other three kinds name one material and leave the rest of the array at that same 0.
fn void a_mesh_child_carries_a_material_index_per_slot() @test {
    FlatChildren read = the_children_a_compound_reports_in_order();

    assert(read.children[2].type == b3::ShapeType.MESH);
    assert(read.children[2].material_indices[0] == 0, "the mesh names its first material");
    assert(read.children[2].material_indices[1] == 2, "and its second, which the second sphere shares");
    assert(read.children[2].material_indices[2] == 0, "while the two slots it does not use read as index 0");
    assert(read.children[2].material_indices[3] == 0);

    assert(read.children[1].material_indices[0] == 1, "a hull child names one material in slot 0");
    assert(read.children[1].material_indices[1] == 0, "and leaves the rest at the same unusable 0");
}

struct PerKindChildren {
    float capsule_radius;
    float capsule_center1_x;
    float capsule_center1_y;
    int capsule_material_index;
    float hull_transform_x;
    float hull_transform_y;
    float hull_transform_z;
    int hull_material_index;
    float mesh_transform_x;
    float mesh_scale_x;
    float mesh_scale_z;
    int[b3::MAX_COMPOUND_MESH_MATERIALS] mesh_material_indices;
    float[2] sphere_center_x;
    float[2] sphere_radius;
    int[2] sphere_material_index;
}

fn PerKindChildren the_children_the_per_kind_accessors_report() {
    b3::CompoundData* compound = a_compound_of_one_child_of_every_kind()!!;
    defer compound.destroy();

    // Seeded away from zero so a material index that reads 0 proves the accessor ran, rather than
    // agreeing with the struct's own default.
    PerKindChildren read;
    read.capsule_material_index = -1;
    read.hull_material_index = -1;
    read.mesh_material_indices = { -1, -1, -1, -1 };
    read.sphere_material_index = { -1, -1 };

    if (try capsule = compound.capsule(0)) {
        read.capsule_radius = capsule.capsule.radius;
        read.capsule_center1_x = capsule.capsule.center1.x;
        read.capsule_center1_y = capsule.capsule.center1.y;
        read.capsule_material_index = capsule.material_index;
    }
    if (try hull = compound.hull(0)) {
        read.hull_transform_x = hull.transform.p.x;
        read.hull_transform_y = hull.transform.p.y;
        read.hull_transform_z = hull.transform.p.z;
        read.hull_material_index = hull.material_index;
    }
    if (try mesh = compound.mesh(0)) {
        read.mesh_transform_x = mesh.transform.p.x;
        read.mesh_scale_x = mesh.scale.x;
        read.mesh_scale_z = mesh.scale.z;
        read.mesh_material_indices = mesh.material_indices;
    }
    for (int i = 0; i < compound.sphere_count && i < read.sphere_radius.len; i++) {
        if (try sphere = compound.sphere(i)) {
            read.sphere_center_x[i] = sphere.sphere.center.x;
            read.sphere_radius[i] = sphere.sphere.radius;
            read.sphere_material_index[i] = sphere.material_index;
        }
    }
    return read;
}

// Each of the four per-kind accessors is indexed by its own count, so index 0 means a different
// child in each of them. What comes back is what the definition carried, geometry and material
// index alike.
fn void every_per_kind_accessor_reports_the_child_the_definition_gave_it() @test {
    PerKindChildren read = the_children_the_per_kind_accessors_report();

    assert(read.capsule_radius > 0.24 && read.capsule_radius < 0.26);
    assert(read.capsule_center1_x > 0.99 && read.capsule_center1_x < 1.01);
    assert(read.capsule_center1_y > -0.51 && read.capsule_center1_y < -0.49,
        "the capsule's first centre, which a transposed vector would not report");
    assert(read.capsule_material_index == 0);

    assert(read.hull_transform_x > 6.99 && read.hull_transform_x < 7.01);
    assert(read.hull_transform_y > 1.99 && read.hull_transform_y < 2.01);
    assert(read.hull_transform_z > -1.01 && read.hull_transform_z < -0.99,
        "all three components of the hull's placement");
    assert(read.hull_material_index == 1);

    assert(read.mesh_transform_x > -5.01 && read.mesh_transform_x < -4.99);
    assert(read.mesh_scale_x > 1.99 && read.mesh_scale_x < 2.01);
    assert(read.mesh_scale_z > 2.99 && read.mesh_scale_z < 3.01,
        "a non-uniform scale, so a mesh child reporting one axis for another would show");
    assert(read.mesh_material_indices[0] == 0 && read.mesh_material_indices[1] == 2);

    assert(read.sphere_center_x[0] > -3.01 && read.sphere_center_x[0] < -2.99);
    assert(read.sphere_radius[0] > 0.49 && read.sphere_radius[0] < 0.51);
    assert(read.sphere_material_index[0] == 1, "the first sphere sharing the hull's material");
    assert(read.sphere_center_x[1] > 3.99 && read.sphere_center_x[1] < 4.01);
    assert(read.sphere_radius[1] > 0.24 && read.sphere_radius[1] < 0.26,
        "the two spheres differing in size as well as in place, so a swap would be caught");
    assert(read.sphere_material_index[1] == 2);
}

struct CompoundMaterialsRead {
    int count;
    int length;
    float[4] friction;
}

fn CompoundMaterialsRead the_materials_a_compound_reports() {
    b3::CompoundData* compound = a_compound_of_one_child_of_every_kind()!!;
    defer compound.destroy();

    b3::SurfaceMaterial[] materials = compound.materials();

    CompoundMaterialsRead read = {
      .count = compound.material_count,
      .length = materials.len,
    };
    for (int i = 0; i < materials.len && i < read.friction.len; i++) {
        read.friction[i] = materials[i].friction;
    }
    return read;
}

// Five children, three materials: box3d keeps one entry per distinct material and the children
// name it by index. Distinct is byte for byte, so two children agreeing on friction but not on
// user_material_id would still take an entry each.
fn void a_compound_reports_one_material_per_distinct_child_surface() @test {
    CompoundMaterialsRead read = the_materials_a_compound_reports();

    assert(read.count == 3, "the hull shares the first sphere's material and the mesh the second's");
    assert(read.length == read.count, "the slice being as long as the count says");
    assert(read.friction[0] > 0.54 && read.friction[0] < 0.56, "the capsule's, first, as its child is");
    assert(read.friction[1] > 0.29 && read.friction[1] < 0.31, "then the hull and first sphere's");
    assert(read.friction[2] > 0.89 && read.friction[2] < 0.91, "then the mesh and second sphere's");
}

fn fault[10] the_refusals_of_the_child_accessors() {
    b3::CompoundData* compound = a_compound_of_one_child_of_every_kind()!!;
    defer compound.destroy();

    fault[10] refusals;
    if (catch err = compound.child(-1)) refusals[0] = err;
    if (catch err = compound.child(compound.child_count())) refusals[1] = err;
    if (catch err = compound.capsule(-1)) refusals[2] = err;
    if (catch err = compound.capsule(compound.capsule_count)) refusals[3] = err;
    if (catch err = compound.hull(-1)) refusals[4] = err;
    if (catch err = compound.hull(compound.hull_count)) refusals[5] = err;
    if (catch err = compound.mesh(-1)) refusals[6] = err;
    if (catch err = compound.mesh(compound.mesh_count)) refusals[7] = err;
    if (catch err = compound.sphere(-1)) refusals[8] = err;
    if (catch err = compound.sphere(compound.sphere_count)) refusals[9] = err;
    return refusals;
}

// box3d bounds every one of these with an assertion the release build compiles out, so both ends
// reach memory that is not a child and answer with plausible geometry. The negative end is the
// worse of the two for the flat accessor, which subtracts each kind's count as it walks and so
// reads further before the array the further out the index is.
fn void an_index_outside_a_compound_is_refused_at_both_ends() @test {
    fault[10] refusals = the_refusals_of_the_child_accessors();

    assert(refusals[0] == b3::INDEX_OUT_OF_RANGE, "a negative flat index names no child");
    assert(refusals[1] == b3::INDEX_OUT_OF_RANGE, "and neither does the one past the last");
    assert(refusals[2] == b3::INDEX_OUT_OF_RANGE, "a negative capsule index names no capsule");
    assert(refusals[3] == b3::INDEX_OUT_OF_RANGE, "and neither does the one past the last");
    assert(refusals[4] == b3::INDEX_OUT_OF_RANGE);
    assert(refusals[5] == b3::INDEX_OUT_OF_RANGE);
    assert(refusals[6] == b3::INDEX_OUT_OF_RANGE);
    assert(refusals[7] == b3::INDEX_OUT_OF_RANGE);
    assert(refusals[8] == b3::INDEX_OUT_OF_RANGE);
    assert(refusals[9] == b3::INDEX_OUT_OF_RANGE);
}

struct EmptyCompoundRead {
    int child_count;
    int material_count;
    int materials_length;
    fault first_child;
}

fn EmptyCompoundRead the_readings_of_an_empty_compound() {
    b3::CompoundDef empty;
    b3::CompoundData* compound = b3::create_compound(&empty)!!;
    defer compound.destroy();

    EmptyCompoundRead read = {
      .child_count = compound.child_count(),
      .material_count = compound.material_count,
      .materials_length = compound.materials().len,
    };
    if (catch err = compound.child(0)) read.first_child = err;
    return read;
}

// box3d hands back a non-null material pointer for a compound with no children at all, one that
// addresses the first byte past the allocation, so the count is the only thing that reports the
// emptiness and the slice is what carries it to a caller.
fn void an_empty_compound_reports_no_children_and_no_materials() @test {
    EmptyCompoundRead read = the_readings_of_an_empty_compound();

    assert(read.child_count == 0);
    assert(read.material_count == 0, "a compound with no children has no materials either");
    assert(read.materials_length == 0, "which the slice reports as a length, never as a pointer");
    assert(read.first_child == b3::INDEX_OUT_OF_RANGE, "and it has no child 0 to read");
}

fn void the_compound_child_types_match_the_c_layout() @test {
    assert(b3::CompoundData::size == 144);
    assert(b3::CompoundData::alignment == 8);
    assert(b3::DynamicTree::size == 80);
    assert(b3::ChildShape::size == 80);
    assert(b3::ChildShape::alignment == 8);
    assert(b3::CompoundCapsule::size == 32);
    assert(b3::CompoundHull::size == 40);
    assert(b3::CompoundMesh::size == 64);
    assert(b3::CompoundSphere::size == 20);
}
