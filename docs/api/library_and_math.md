# Library, math and identifiers

[Documentation](../index.md) › [Public API](index.md) › Library, math and identifiers

Process-wide hooks (version, float-ABI check, allocator, log, timer, length units, hash), the math
value types every other page uses, and the identifier structs with their null and validity
checks. Everything here is in module `b3`. Faults are listed in the
[API index](index.md#faults).

## Startup

```c3
fn void on_box3d_log(ZString message) {
    io::printfn("box3d: %s", message);
}

fn void? lib_startup() {
    b3::require_float_abi()!;
    b3::Version version = b3::version();
    io::printfn("box3d %d.%d.%d", version.major, version.minor, version.revision);

    b3::set_log_fn(&on_box3d_log);
    b3::set_length_units_per_meter(100.0f);   // centimetres
    b3::set_stall_threshold(0.002f);          // log continuous-collision passes over 2 ms
}
```

`require_float_abi` returns `DOUBLE_PRECISION_LIBRARY` when the linked library was built with
`BOX3D_DOUBLE_PRECISION`. The bindings declare `Vec3`, `Pos` and every struct embedding them as
float, so a double build corrupts every call. Call it once before anything else.
`is_double_precision` is the raw query behind it.

`version` returns the linked library's `Version` (`major`, `minor`, `revision`).

### Length units

`set_length_units_per_meter` scales box3d's internal tolerances (linear slop, AABB margin, the
`B3_HUGE` bound) and the defaults that `default_world_def`, `default_body_def` and
`default_shape_def` fill in. The value is a process-wide global read at the time each default is
built, so set it once at startup, before the first `default_*_def()` call and before any world
exists. `length_units_per_meter` reads it back. A replay player temporarily takes the value over;
see [recording](recording.md#contracts).

### Log and assert hooks

`set_log_fn` installs a `LogFn` that receives every message box3d logs: memory dumps from
[`world_dump_memory_stats`](worlds.md#diagnostics), continuous-collision stalls, and warnings.
Messages are formatted into a 512-byte buffer, so longer ones are truncated. The handler may be
called from a worker thread during a step.

`set_stall_threshold(seconds)` makes box3d log any single continuous-collision pass that takes
longer than `seconds`. The default is `float::max`, which disables the log.
`stall_threshold` reads it back.

`set_assert_fn` installs an `AssertFn`. The shipped library is a Release build with every box3d
assertion compiled out, so this handler is never called. Neither hook can be reset to box3d's
default handler once replaced.

## Allocator

```c3
fn void* box3d_alloc(int size, int alignment) {
    return mem::malloc_aligned(size, alignment);
}

fn void box3d_free(void* memory) {
    mem::free_aligned(memory);
}
```

```c3
b3::set_allocator(&box3d_alloc, &box3d_free);
```

`set_allocator` takes an `AllocFn` and a `FreeFn`. box3d asks for sizes rounded up to a multiple
of 16 with an alignment of 16. The returned pointer must honour that alignment and must not be
null: the check is compiled out. `set_allocator(null, null)` restores the built-in allocator.

box3d frees each block through whichever `FreeFn` is installed when it frees, not the one that
allocated it. Install the pair before box3d allocates anything (before the first world, mesh,
hull or recording) and keep it until all of that is destroyed. Install both functions or neither.

`byte_count` reports box3d's live allocated bytes, whichever allocator is installed. It returns to
its starting value only when every world, geometry object, recording and player is destroyed,
which makes it a leak check.

## Timer and threads

```c3
ulong stamp = b3::ticks();
b3::world_step(world, 1.0f / 60.0f, 4);
float since_stamp = b3::milliseconds(stamp);         // stamp unchanged
float lap = b3::milliseconds_and_reset(&stamp);      // stamp moved to now
```

`ticks` is box3d's monotonic tick counter. `milliseconds(stamp)` measures from a stamp and leaves
it alone. `milliseconds_and_reset(&stamp)` measures and then moves the stamp to now, so repeated
calls measure the gap between calls. `yield` yields the thread in a busy loop. `sleep(ms)` sleeps
it. Both are provided for task systems.

## Hash

```c3
char[] bytes = "frame 42";
uint digest = b3::hash(bytes);
uint chained = b3::hash(bytes, digest);
```

`hash` is the djb2 hash box3d uses for determinism checks. The seed defaults to `HASH_INIT`
(5381). Pass a previous result as the seed to hash several buffers as one stream.

## Math types

| C3 | C | Notes |
|---|---|---|
| `Vec3` | `b3Vec3` | `float[<3>]`: vector operators work directly |
| `Pos` | `b3Pos` | distinct `typedef` of `Vec3` for world positions; converts to `Vec3` implicitly, not back |
| `Quat` | `b3Quat` | `std::math` `Quaternionf`; `.rotate_vec3()`, `.mul()`, `.normalize()` and the rest apply |
| `Matrix3` | `b3Matrix3` | `std::math` `Matrix3f`; column-major, see below |
| `Transform` | `b3Transform` | `p: Vec3`, `q: Quat`; a local frame |
| `WorldTransform` | `b3WorldTransform` | `p: Pos`, `q: Quat`; a body pose |
| `Aabb` | `b3AABB` | `lower_bound`, `upper_bound` |
| `Plane` | `b3Plane` | `normal`, `offset` |
| `CosSin` | `b3CosSin` | `cosine`, `sine` |
| `SegmentDistanceResult` | `b3SegmentDistanceResult` | two closest points and their fractions |

```c3
b3::Vec3 x_axis = { 1.0f, 0.0f, 0.0f };
b3::Vec3 y_axis = { 0.0f, 1.0f, 0.0f };
b3::Quat quarter_turn = b3::compute_quat_between_unit_vectors(x_axis, y_axis)!;
b3::Vec3 turned = quarter_turn.rotate_vec3(x_axis);              // ~ y_axis

b3::Transform local = { .p = { 0.0f, 0.5f, 0.0f }, .q = quarter_turn };
b3::WorldTransform pose = { .p = { 10.0f, 0.0f, 0.0f }, .q = quarter_turn };
b3::Aabb box = { .lower_bound = { -1.0f, -1.0f, -1.0f }, .upper_bound = { 1.0f, 1.0f, 1.0f } };
bool usable = b3::is_valid_transform(local) && b3::is_sane_aabb(box);
```

### Matrix3 is column-major

`Matrix3` is the standard library's `Matrix3f`, which matches `b3Matrix3` byte for byte. Both
store three columns, and `m * v` computes what `b3MulMV` computes. Fill `.m` one column at a
time:

```c3
b3::Matrix3 quarter_turn_about_z = {
  .m = {
    0.0f, 1.0f, 0.0f,     // column 0: where x goes
    -1.0f, 0.0f, 0.0f,    // column 1: where y goes
    0.0f, 0.0f, 1.0f,     // column 2: where z goes
  },
};
b3::Vec3 onto_y = quarter_turn_about_z * x_axis;                  // (0, 1, 0)
b3::Quat same_turn = b3::make_quat_from_matrix(quarter_turn_about_z);
float row0_col1 = quarter_turn_about_z.m01;                       // -1
```

The named fields are `mRC`: `m01` is row 0 of column 1. A matrix filled row by row is the
transpose: as a rotation it turns the other way, and no layout check can detect that.

### Math functions

| Function | Result |
|---|---|
| `make_quat_from_matrix(m)` | the normalized rotation the matrix's columns describe |
| `compute_quat_between_unit_vectors(v1, v2)` | shortest rotation from `v1` to `v2`; `NOT_A_UNIT_VECTOR` unless both are unit length; antipodal vectors give a half turn |
| `compute_cos_sin(radians)` | cosine and sine together |
| `atan2(y, x)` | box3d's approximate, cross-platform deterministic arc tangent; `atan2(0, 0)` is 0 |
| `point_to_segment_distance(a, b, q)` | the point on segment `a`–`b` closest to `q` |
| `segment_distance(p1, q1, p2, q2)` | closest points between two segments given by endpoints |
| `line_distance(p1, d1, p2, d2)` | closest points between two infinite lines given by point and direction; `DEGENERATE_GEOMETRY` when `d1` has zero length |
| `steiner(mass, origin)` | the parallel-axis inertia term for a point mass at `origin`; add it to a tensor about the centre of mass to move it |

```c3
b3::SegmentDistanceResult closest = b3::line_distance(
  p1: { 0.0f, 0.0f, 0.0f },
  d1: { 1.0f, 0.0f, 0.0f },
  p2: { 0.0f, 1.0f, 0.0f },
  d2: { 0.0f, 0.0f, 1.0f },
)!;
```

`line_distance` fractions run along the directions as given and are unbounded, so a closest point
behind a line's origin has a negative fraction.

### Validity predicates

`is_valid_float`, `is_valid_vec3`, `is_valid_position`, `is_valid_quat`, `is_valid_transform`,
`is_valid_world_transform`, `is_valid_matrix3`, `is_valid_aabb`, `is_bounded_aabb`,
`is_sane_aabb` and `is_valid_plane` are box3d's own checks. Use them to validate input before
handing it to calls that do not check it themselves (most setters do not). Each is narrower than
its name:

- `is_valid_quat`: finite and unit length.
- `is_valid_world_transform`: finite position and unit rotation. Distance from the origin is not
  bounded.
- `is_valid_matrix3`: all nine entries finite. It does not test for a rotation: the zero matrix
  passes.
- `is_valid_aabb`: finite and not inverted.
- `is_bounded_aabb`: `lower_bound` not below `-B3_HUGE` and `upper_bound` not above it (`1e5`
  length units). It does not test ordering, and a NaN passes.
- `is_sane_aabb`: `is_valid_aabb` and `is_bounded_aabb` together.
- `is_valid_plane`: finite, with a unit normal.

## Identifiers

`WorldId`, `BodyId`, `ShapeId`, `JointId` and `ContactId` are small value structs, passed and
copied by value. They are handles into a world's tables, not pointers, and never own anything.

```c3
b3::BodyId unset;                                      // zero-initialised: unset.is_null()
if (!body.is_valid()) return b3::INVALID_HANDLE~;      // destroyed
b3::WorldId owner_world = body.world().checked()!;     // null id becomes INVALID_HANDLE
```

| Check | What it tests | Cost |
|---|---|---|
| `.is_null()` | the zero identifier (`index1 == 0`) | field read |
| `.is_valid()` | box3d's own test: the world slot is occupied, the object slot is in range and live, the generation matches | one call |
| `.checked()` | `.is_null()` as an optional: the id, or `INVALID_HANDLE` | field read |

`.is_null()` and `.checked()` are defined for all five types here. `.is_valid()` is
`WorldId.is_valid` ([worlds](worlds.md)), `BodyId.is_valid` ([bodies](bodies.md)),
`ShapeId.is_valid` ([shapes](shapes.md)), `JointId.is_valid` and `ContactId.is_valid`
([worlds](worlds.md#events)).

A non-null id is not proof of a live object. Ids from end-touch events, sensor overlaps and
joint events can name objects destroyed in the same step, and an id held across frames can
outlive its object. Test `.is_valid()` before using such an id. box3d's per-call validity
assertions are compiled out, so a call on a stale id reads or writes whatever now occupies the
slot.

`.is_valid()` checks an id against its own world. It does not tell you the id belongs to the world
you are about to pass it to.

### Checked access

`.checked()` is always available. The `BOX3D_CHECKED` feature does not enable or disable any
check; it only changes what `b3::checked_access()` returns, so code can branch on the build's
intent:

```json
{ "features": [ "BOX3D_CHECKED" ] }
```

`checked_geometry(pointer, FAULT)` is the macro the geometry builders use to turn a null pointer
into a named fault. It returns the pointer, or the fault you name.

## Contracts

- Call `require_float_abi()` before any other call.
- Set the length unit before building any `default_*_def()` and before creating any world. It is
  process-wide.
- Install a custom allocator before box3d allocates and keep it until everything is destroyed.
  Its allocations must be 16-byte aligned and never null.
- `set_log_fn(null)` crashes on the next log message. The log handler may run on a worker thread.
- The assert hook never fires in the shipped build. Both default handlers are unrecoverable once
  replaced.
- `Matrix3` is column-major; filling it from rows silently transposes it.
- `.is_null()` and `.checked()` test for the zero id only. `.is_valid()` is the liveness test, and
  only for the id's own world.
- Calls on a destroyed object's id are undefined behaviour, not a fault.
