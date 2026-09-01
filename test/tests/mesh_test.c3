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
