# Cookbook

[Documentation](index.md) › Cookbook

Short recipes for common tasks. Each assumes a live `b3::WorldId` named `world` unless it says
otherwise, and `import std::io;` where it prints. Each recipe links to the API page that owns its
calls; exact signatures and fault lists are in the docstrings.

## Fixed-timestep loop

Accumulate frame time and step the world in fixed increments, capping the steps per frame:

```c3
const float TIME_STEP = 1.0f / 60.0f;
const int SUB_STEPS = 4;
const int MAX_STEPS_PER_FRAME = 5;

fn float advance(b3::WorldId world, float accumulator, float frame_seconds) {
    accumulator += frame_seconds;
    int steps = 0;
    while (accumulator >= TIME_STEP && steps < MAX_STEPS_PER_FRAME) {
        b3::world_step(world, TIME_STEP, SUB_STEPS);
        handle_events(world);
        accumulator -= TIME_STEP;
        steps++;
    }
    return accumulator;
}
```

- Read events inside the loop, once per step. Each event slice holds the last step only.
- The cap keeps a slow frame from requesting ever more steps. Time beyond it carries over.

API: [worlds](api/worlds.md).

## Apply a force or an impulse

Forces act over the next step; impulses change velocity at once:

```c3
fn void push(b3::BodyId body) {
    b3::body_apply_force_to_center(body, { 0.0f, 50.0f, 0.0f }, true);
    b3::body_apply_torque(body, { 0.0f, 2.0f, 0.0f }, true);

    b3::Pos hit_point = body.world_point({ 0.5f, 0.0f, 0.0f });
    b3::body_apply_linear_impulse(body, { 0.0f, 0.0f, 3.0f }, hit_point, true);
}
```

- box3d clears accumulated forces and torques after every step. Apply a continuous force each step.
- With `wake` false, a call on a sleeping body is discarded.
- The application point is a world position; `body.world_point()` converts from body-local.

API: [bodies](api/bodies.md).

## Ray cast: closest hit

```c3
b3::QueryFilter filter = b3::default_query_filter();
b3::Pos origin = { 0.0f, 10.0f, 0.0f };
b3::Vec3 translation = { 0.0f, -20.0f, 0.0f };

if (try hit = b3::world_cast_ray_closest(world, origin, translation, filter)) {
    io::printfn("hit at y %.3f, fraction %.3f", hit.point.y, hit.fraction);
}
```

- The ray runs from `origin` to `origin + translation`; `fraction` scales `translation`, not a unit
  direction. A miss is `NO_HIT`.
- A ray that starts inside a shape does not report that shape.
- World casts report sensor shapes too. Exclude them with filter bits or a callback.

API: [queries and casts](api/queries_and_casts.md).

## Ray cast with a callback

The callback sees every shape along the ray and steers the cast with its return value. This one
skips sensors and keeps the nearest solid shape:

```c3
struct NearestSolid {
    b3::ShapeId shape;
    b3::Pos point;
    bool found;
}

fn float keep_nearest_solid(
  b3::ShapeId shape,
  b3::Pos point,
  b3::Vec3 normal,
  float fraction,
  ulong user_material_id,
  int triangle_index,
  int child_index,
  void* context,
) {
    if (shape.is_sensor()) return -1.0f;
    NearestSolid* nearest = context;
    nearest.shape = shape;
    nearest.point = point;
    nearest.found = true;
    return fraction;
}

fn void cast_with_callback(b3::WorldId world) {
    NearestSolid nearest;
    b3::world_cast_ray(
      world: world,
      origin: { 0.0f, 10.0f, 0.0f },
      translation: { 0.0f, -20.0f, 0.0f },
      filter: b3::default_query_filter(),
      fcn: &keep_nearest_solid,
      context: &nearest,
    );
    if (nearest.found) io::printfn("nearest solid at y %.3f", nearest.point.y);
}
```

- Shapes arrive in no particular order. Return `-1` to skip a shape, `0` to stop, `1` to continue
  unclipped, or the hit's `fraction` to clip the ray there so only nearer shapes follow.
- The callback runs on the calling thread, inside the call. It must not modify the world.
- `b3::world_cast_ray_all` collects every hit into a caller buffer instead.

API: [queries and casts](api/queries_and_casts.md).

## Overlap query

Collect candidates in a box, or the shapes that truly overlap a convex probe:

```c3
b3::Aabb box = { .lower_bound = { -1.0f, 0.0f, -1.0f }, .upper_bound = { 1.0f, 2.0f, 1.0f } };
b3::ShapeId[64] buffer;
foreach (shape : b3::world_overlap_aabb_all(world, box, b3::default_query_filter(), buffer[..])) {
    io::printfn("broad-phase candidate on body %s", shape.body().name());
}

b3::Vec3[1] centre = { { 0.0f, 0.0f, 0.0f } };
b3::ShapeProxy ball = { .points = &centre, .count = 1, .radius = 0.75f };
b3::ShapeId[] touching = b3::world_overlap_shape_all(
  world: world,
  origin: { 0.0f, 1.0f, 0.0f },
  proxy: &ball,
  filter: b3::default_query_filter(),
  buffer: buffer[..],
);
io::printfn("%d shapes within 0.75 of (0, 1, 0)", touching.len);
```

- The box query tests broad-phase bounds, which are larger than the shapes. It reports
  candidates, not intersections.
- A proxy is a point cloud plus a radius: one point is a sphere, two a capsule. The points are read
  during the call only, and `count` must be between 1 and `MAX_SHAPE_CAST_POINTS`.
- The result is a prefix of `buffer`. A result as long as the buffer may have been cut short.
- A compound-shaped body far from the probe's origin can be missed without a signal.

API: [queries and casts](api/queries_and_casts.md).

## Contact events

Ask for events in the shape definition, then read the slices after each step:

```c3
fn b3::ShapeDef reporting_shape_def() {
    b3::ShapeDef def = b3::default_shape_def();
    def.enable_contact_events = true;
    def.enable_hit_events = true;
    return def;
}

fn void contact_events(b3::WorldId world) {
    foreach (event : world.contact_begin_touch_events()) {
        io::printfn("begin: %s / %s", event.shape_id_a.body().name(), event.shape_id_b.body().name());
    }
    foreach (event : world.contact_end_touch_events()) {
        if (!event.shape_id_a.is_valid() || !event.shape_id_b.is_valid()) continue;
        io::printfn("end: %s / %s", event.shape_id_a.body().name(), event.shape_id_b.body().name());
    }
    foreach (event : world.contact_hit_events()) {
        io::printfn("hit at %.2f m/s", event.approach_speed);
    }
}
```

- One of the two shapes enabling contact events, or hit events, is enough. Both are off by default.
- Hit events fire only above `world.hit_event_threshold()`, 1 m/s at the default length unit.
- An end event can name shapes that are already gone: destroying a body or shape ends its
  contacts. Test `is_valid()` before using them.
- An end event caused between steps, such as by destroying a body, appears after the next step.
- The slices belong to the world and are replaced by the next `world_step`.

API: [worlds and events](api/worlds.md).

## Sensor events

A sensor shape reports which shapes enter and leave it, and does not collide:

```c3
fn b3::ShapeId? create_trigger(b3::WorldId world) {
    b3::BodyDef body_def = b3::default_body_def();
    body_def.position = { 0.0f, 1.0f, 0.0f };
    b3::BodyId body = b3::create_body(world, &body_def)!;

    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    b3::BoxHull box = b3::make_box_hull(2.0f, 1.0f, 2.0f);
    return b3::create_hull_shape(body, &shape_def, &box.base);
}

fn b3::ShapeId? create_visitor(b3::BodyId body) {
    b3::ShapeDef shape_def = b3::default_shape_def();
    shape_def.enable_sensor_events = true;
    b3::Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = 0.25f };
    return b3::create_sphere_shape(body, &shape_def, &sphere);
}

fn void sensor_events(b3::WorldId world) {
    foreach (event : world.sensor_begin_touch_events()) {
        io::printfn("entered: %s", event.visitor_shape_id.body().name());
    }
    foreach (event : world.sensor_end_touch_events()) {
        if (!event.visitor_shape_id.is_valid()) continue;
        io::printfn("left: %s", event.visitor_shape_id.body().name());
    }
}
```

- Both shapes must set `enable_sensor_events`: the visitor as well as the sensor.
- Only convex visitors are detected: spheres, capsules and hulls. Meshes, height fields and
  compounds pass through unreported, as do shapes on the sensor's own body.
- `shape.sensor_data(buffer)` lists the current overlaps instead of the changes.

API: [worlds and events](api/worlds.md), [shapes](api/shapes.md).

## Revolute joint with a motor

A hinge about the z axis of the two local frames, driven by a motor:

```c3
fn b3::RevoluteJointId? create_wheel_hinge(b3::WorldId world, b3::BodyId chassis, b3::BodyId wheel) {
    b3::RevoluteJointDef def = b3::default_revolute_joint_def();
    def.body_id_a = chassis;
    def.body_id_b = wheel;
    def.local_frame_a.p = { 1.0f, -0.5f, 0.0f };
    def.local_frame_b.p = { 0.0f, 0.0f, 0.0f };
    def.enable_motor = true;
    def.max_motor_torque = 50.0f;
    def.motor_speed = 4.0f;
    return b3::create_revolute_joint(world, &def);
}

fn void drive(b3::RevoluteJointId hinge, float speed) {
    hinge.set_motor_speed(speed);
    b3::joint_wake_bodies(hinge);
}
```

From a plain `JointId`, narrow first:

```c3
fn void? narrow(b3::JointId joint) {
    b3::RevoluteJointId hinge = joint.as_revolute()!;
    io::printfn("motor torque %.2f", hinge.motor_torque());
}
```

- The default revolute definition is a bare hinge: `max_motor_torque` is 0, so an enabled motor
  turns nothing until it is raised.
- Local frames are measured from each body's origin, not its centre of mass. The hinge axis is the
  frames' z axis.
- `set_motor_speed` does not wake sleeping bodies; `b3::joint_wake_bodies` does.
- `motor_torque()` is the motor's own reading. The shared `constraint_torque()` counts the hinge
  axis twice.

API: [joints](api/joints.md).

## Mesh shape from triangles

Build a mesh, attach it, and destroy the shape before the mesh:

```c3
fn void? mesh_floor(b3::WorldId world) {
    b3::Vec3[4] vertices = {
        { -10.0f, 0.0f, -10.0f },
        { 10.0f, 0.0f, -10.0f },
        { 10.0f, 0.0f, 10.0f },
        { -10.0f, 0.0f, 10.0f },
    };
    int[6] indices = { 0, 2, 1, 0, 3, 2 };
    b3::MeshDef mesh_def = {
        .vertices = &vertices,
        .indices = &indices,
        .vertex_count = vertices.len,
        .triangle_count = indices.len / 3,
        .identify_edges = true,
    };
    b3::MeshData* mesh = b3::create_mesh(&mesh_def)!;
    defer b3::destroy_mesh(mesh);

    b3::BodyDef body_def = b3::default_body_def();
    b3::BodyId floor = b3::create_body(world, &body_def)!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::ShapeId shape = b3::create_mesh_shape(floor, &shape_def, mesh, { 1.0f, 1.0f, 1.0f })!;
    defer b3::destroy_shape(shape, false);

    for (int i = 0; i < 60; i++) b3::world_step(world, TIME_STEP, SUB_STEPS);
}
```

- The shape borrows the mesh. The mesh must outlive every shape on it, and destroying the world
  does not free it. Defers run in reverse, so here the shape goes first.
- Wind triangles counter-clockwise, as `MeshDef` requires. `create_mesh` copies the arrays, so they
  need not outlive the call.
- Triangles with no area are skipped; a mesh with none left is refused with `DEGENERATE_GEOMETRY`.
  A refused build can leak part of its allocation, so validate geometry before building.
- A mesh shape contributes no mass. On a dynamic body, give the body other shapes for mass.

API: [shapes](api/shapes.md).

## Height field

A grid of heights for terrain, attached to a static body:

```c3
const int TERRAIN_SIZE = 4;

fn void? terrain(b3::WorldId world) {
    float[TERRAIN_SIZE * TERRAIN_SIZE] heights = {
        0.0f, 0.1f, 0.2f, 0.1f,
        0.1f, 0.3f, 0.4f, 0.2f,
        0.2f, 0.4f, 0.5f, 0.3f,
        0.1f, 0.2f, 0.3f, 0.2f,
    };
    b3::HeightFieldDef field_def = {
        .heights = &heights,
        .scale = { 2.0f, 1.0f, 2.0f },
        .count_x = TERRAIN_SIZE,
        .count_z = TERRAIN_SIZE,
        .global_minimum_height = 0.0f,
        .global_maximum_height = 1.0f,
    };
    b3::HeightFieldData* field = b3::create_height_field(&field_def)!;
    defer b3::destroy_height_field(field);

    b3::BodyDef body_def = b3::default_body_def();
    b3::BodyId ground = b3::create_body(world, &body_def)!;
    b3::ShapeDef shape_def = b3::default_shape_def();
    b3::ShapeId shape = b3::create_height_field_shape(ground, &shape_def, field)!;
    defer b3::destroy_shape(shape, false);

    for (int i = 0; i < 60; i++) b3::world_step(world, TIME_STEP, SUB_STEPS);
}
```

- Only a static body accepts a height field. A kinematic or dynamic body returns `INVALID_HANDLE`.
- `heights` holds `count_x * count_z` samples. Each is clamped to the global range and quantised to
  16 bits. `scale` spaces the grid in x and z and multiplies heights in y.
- Ownership works as for meshes: the field outlives its shape, and the world never frees it.
- `material_indices` is optional, one per cell; `0xFF` marks a hole.

API: [shapes](api/shapes.md).

## Character mover: basic move

A capsule moved by collide, solve and cast, the loop box3d's own mover sample uses. The character is
not a body; it lives in caller state:

```c3
const int MOVER_PLANE_CAPACITY = 16;
const int MOVER_ITERATIONS = 5;
const float MOVER_TOLERANCE = 0.01f;

struct MoverPlanes {
    b3::CollisionPlane[MOVER_PLANE_CAPACITY] planes;
    int count;
}

fn bool gather_planes(b3::ShapeId shape, b3::PlaneResult* results, int result_count, void* context) {
    MoverPlanes* gathered = context;
    for (int i = 0; i < result_count && gathered.count < MOVER_PLANE_CAPACITY; i++) {
        gathered.planes[gathered.count] = {
            .plane = results[i].plane,
            .push_limit = float::max,
            .push = 0.0f,
            .clip_velocity = true,
        };
        gathered.count++;
    }
    return true;
}

struct Character {
    b3::Pos position;
    b3::Vec3 velocity;
    b3::Capsule capsule;
}

fn void move_character(b3::WorldId world, Character* character, float time_step) {
    b3::QueryFilter filter = b3::default_query_filter();
    b3::Pos target = (b3::Pos)(character.position + character.velocity * time_step);
    MoverPlanes gathered;

    for (int iteration = 0; iteration < MOVER_ITERATIONS; iteration++) {
        gathered.count = 0;
        b3::world_collide_mover(
          world: world,
          origin: character.position,
          mover: &character.capsule,
          filter: filter,
          fcn: &gather_planes,
          context: &gathered,
        );

        b3::Vec3 target_delta = target - character.position;
        b3::PlaneSolverResult solved = b3::solve_planes(target_delta, gathered.planes[:gathered.count]);

        float fraction = b3::world_cast_mover(
          world: world,
          origin: character.position,
          mover: &character.capsule,
          translation: solved.delta,
          filter: filter,
          fcn: null,
          context: null,
        );
        b3::Vec3 delta = solved.delta * fraction;
        character.position = (b3::Pos)(character.position + delta);

        if (delta.length() < MOVER_TOLERANCE) break;
    }

    character.velocity = b3::clip_vector(character.velocity, gathered.planes[:gathered.count]);
}
```

- The capsule is relative to `origin`, and so are the planes. Copy the planes out inside the
  callback: the array dies with it.
- A mesh, height field or compound reports at most 64 planes per shape; the rest are dropped.
- `solve_planes` rewrites every plane's `push`. `clip_vector` then clips only against planes it
  pushed against whose `clip_velocity` is set.
- Movers do not push dynamic bodies. Apply impulses to them yourself, as box3d's sample does.

API: [character mover](api/character_mover.md).

## Explosion

A radial impulse on every dynamic body within reach:

```c3
fn void? blast(b3::WorldId world, b3::Pos where) {
    b3::ExplosionDef def = b3::default_explosion_def();
    def.position = where;
    def.radius = 2.0f;
    def.falloff = 1.0f;
    def.impulse_per_area = 20.0f;
    b3::world_explode(world, def)!;
}
```

- The default moves nothing: radius, falloff and impulse are zero. Fill all three.
- The impulse fades to zero across `falloff`, beyond `radius`, and scales with the area each shape
  presents. A negative `impulse_per_area` pulls inward.
- A non-finite position or impulse, or a negative radius or falloff, returns `INVALID_DEFINITION`.
- A mesh shape on a dynamic body inside the axis-aligned box of half-width `radius + falloff`
  around the position crashes box3d, even outside the reach. Exclude it with `mask_bits`, or set
  its shape's `explosion_scale` to 0.

API: [character mover](api/character_mover.md).

## Debug draw

Shapes are drawn through handles the renderer builds once per shape. Both shape callbacks are world
definition fields, so this recipe creates its own world:

```c3
struct DrawnSphere {
    b3::Sphere sphere;
}

fn void* make_debug_shape(draw::DebugShape* shape, void* context) {
    if (shape.type != b3::ShapeType.SPHERE) return null;
    return mem::new(DrawnSphere, { .sphere = *shape.sphere });
}

fn void free_debug_shape(void* user_shape, void* context) {
    free(user_shape);
}

fn void draw_shape(void* user_shape, b3::WorldTransform transform, draw::HexColor color, void* context) {
    DrawnSphere* drawn = user_shape;
    b3::Vec3 centre = transform.p + drawn.sphere.center.rotate_quat(transform.q);
    io::printfn("sphere r %.2f at y %.2f colour %06x", drawn.sphere.radius, centre.y, (uint)color & 0xFFFFFF);
}

fn void draw_segment(b3::Pos p1, b3::Pos p2, draw::HexColor color, void* context) {
    io::printfn("segment from y %.2f to y %.2f", p1.y, p2.y);
}
```

```c3
b3::WorldDef world_def = b3::default_world_def();
world_def.create_debug_shape = &make_debug_shape;
world_def.destroy_debug_shape = &free_debug_shape;
b3::WorldId world = b3::create_world(&world_def)!;
defer b3::destroy_world(world);

// ... bodies and shapes ...

draw::DebugDraw debug = draw::default_debug_draw();
debug.draw_shape_fcn = &draw_shape;
debug.draw_segment_fcn = &draw_segment;
debug.draw_shapes = true;
debug.draw_joints = true;
draw::world_draw(world, &debug, ulong::max);
```

- `draw_shapes` draws only shapes for which `create_debug_shape` returned non-null. A null return is
  asked again on the next draw.
- Copy geometry inside `create_debug_shape`. The sphere, capsule and mesh arms point into the
  world's shape array, which moves as shapes are added.
- Set `destroy_debug_shape` whenever `create_debug_shape` is set: box3d calls it without a null test
  when a shape is destroyed.
- `default_debug_draw()` fills every callback with an empty function and every flag false. Replace
  callbacks; never set one to null while its flag is raised. A `mask_bits` of 0 draws nothing.
- The high byte of a `HexColor` can carry a material preset; mask with `0xFFFFFF` for the RGB.

API: [debug draw](api/debug_draw.md).

## Record and replay

Record a run of steps, then replay it and check that it reproduces:

```c3
fn void? record_and_replay(b3::WorldId world) {
    record::Recording* recording = record::create_recording(0);
    defer record::destroy_recording(recording);

    record::world_start_recording(world, recording)!;
    for (int i = 0; i < 60; i++) b3::world_step(world, TIME_STEP, SUB_STEPS);
    record::world_stop_recording(world);

    record::Player* player = record::create_player(recording.data(), 1)!;
    defer record::destroy_player(player);

    while (record::player_step_frame(player)) {}

    if (try frame = player.diverge_frame()) {
        io::printfn("replay diverged at frame %d", frame);
    } else {
        io::printfn("replayed %d frames exactly", player.frame_count());
    }
}
```

To keep a recording, save it and check it later:

```c3
fn void? save_and_check(record::Recording* recording) {
    record::save_recording_to_file(recording, "session.b3rec")!;
    record::Recording* loaded = record::load_recording("session.b3rec")!;
    defer record::destroy_recording(loaded);
    if (!record::validate_replay(loaded.data(), 1)) io::printfn("not deterministic");
}
```

- A recording starts with a snapshot of the world, so it can begin mid-run.
  `create_recording(0)` starts at 64 KiB and grows as needed.
- `recording.data()` borrows the buffer, which moves as it grows. Take it after recording stops.
- A player runs a private world. It counts towards `MAX_WORLDS`, and sets the process-wide length
  unit to the recording's until destroyed. `player.world_id()` reaches it for drawing.
- `player_step_frame` returns false at the end. `diverge_frame()` returns `NO_HIT` when every
  recorded state hash matched.

API: [recording and replay](api/recording.md).

## Allocator, log and assert hooks

Route box3d's allocations and log messages through the application. Install at startup:

```c3
fn void* aligned_alloc_hook(int size, int alignment) {
    return mem::malloc_aligned((sz)size, (sz)alignment);
}

fn void aligned_free_hook(void* memory) {
    mem::free_aligned(memory);
}

fn void log_hook(ZString message) {
    io::eprintfn("box3d: %s", message);
}

fn void install_hooks() {
    b3::set_allocator(&aligned_alloc_hook, &aligned_free_hook);
    b3::set_log_fn(&log_hook);
}
```

Restore the built-in allocator only once box3d holds nothing, such as after the last world and
geometry are destroyed:

```c3
b3::set_allocator(null, null);
```

- The hooks are process-wide. Install them before the first box3d allocation and from one thread.
- A block is freed through whichever free function is installed at the time, so never swap
  allocators while box3d holds memory. `b3::byte_count()` reads 0 when it holds none.
- Never pass null to `set_log_fn`; box3d calls the handler without a test. The default handler
  cannot be reinstalled once replaced.
- `b3::set_assert_fn` is accepted, but the shipped Release build never calls an assert handler.

API: [library and math](api/library_and_math.md).
