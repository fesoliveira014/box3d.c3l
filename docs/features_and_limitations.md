# Features and limitations

[Documentation](index.md) › Features and limitations

What `b3` binds, what it leaves out and why, the targets and toolchain it ships for, the fixed
limits an application designs around, and the sharp edges that cross domains.

## Features

`b3` binds box3d's exported C API, apart from the five functions under [Not bound](#not-bound):

- Library hooks: version, allocator, log and assert handlers, timers, length units, the byte count.
- Math types mapped onto the C3 standard library, with box3d's validity predicates and helpers.
- Worlds: creation, stepping, tuning, worker threads and task callbacks, profile and counters,
  friction and restitution mixing callbacks, pre-solve and custom-filter callbacks.
- Events: contact begin, end and hit; sensor begin and end; body moves; joint thresholds.
- Bodies: type, pose, velocity, forces and impulses, mass data, damping, sleep, motion locks,
  bullets, contact and joint listing.
- Shapes: sphere, capsule, convex hull, triangle mesh, height field and baked compound, with
  materials, filters, sensor flags and geometry replacement.
- Geometry builders: hulls from points, boxes, cylinders, cones and rocks; grid, wave, torus, box and
  platform meshes; height fields from grids, samples and files; compounds and their byte round
  trip.
- Joints: all nine types (distance, filter, motor, parallel, prismatic, revolute, spherical, weld,
  wheel), with typed identifiers for the eight that have accessors of their own.
- World queries: closest and callback ray casts, shape casts, box and shape overlaps, collecting
  variants that fill a caller buffer, body-level casts and overlaps, contact data with manifolds.
- Standalone geometry with no world: local ray and shape casts, overlap predicates, the nine
  collide pairs, distance, time of impact, mesh and compound queries.
- The dynamic tree: proxies, queries, ray and box traversals, closest search, save and load.
- The character mover: world and body collide, cast, the plane solver and velocity clip; explosions.
- Debug draw (`b3::draw`): the nine draw callbacks and the per-shape renderer handles.
- Recording and replay (`b3::record`): recordings, file save and load, determinism validation, a
  frame-stepping player with seeking, keyframes, recorded queries and query drawing.

## Not bound

Five exported functions are deliberately absent:

| Function | Why |
|---|---|
| `b3World_DumpShapeBounds` | Declared in `box3d.h` and defined nowhere in box3d, so a call could not link. |
| `b3InternalAssert` | box3d's own assertion hook. It exists only when assertions are compiled in, and the shipped Release build compiles them out. |
| `b3DynamicTree_Validate` | Its body compiles to nothing unless box3d is built with validation, which the shipped build is not. A call that cannot validate would mislead. |
| `b3DynamicTree_ValidateNoEnlarged` | The same as `b3DynamicTree_Validate`. |
| `b3DynamicTree_GetProxyCount` | A read of the `proxy_count` field the binding already exposes on `DynamicTree`, and a C3 method may not share a name with a field. Read `tree.proxy_count`. |

The wrapper layer also leaves out one diagnostic it could not represent: `create_mesh` does not
report which triangles it skipped, because the count that goes with that list lives inside the
opaque mesh.

### Verify coverage

A header declaration does not prove a symbol was built, so coverage is the intersection of three
sets: what the binding names (`@cname` in `src/*.c3i`), what box3d declares (`B3_API` in its
headers), and what the archive defines. From a clone with the native library built:

```sh
grep -ho '@cname("[^"]*")' src/*.c3i | sed 's/@cname("\(.*\)")/\1/' | sort -u > bound.txt
grep -ho 'B3_API[^(]*(' vendor/box3d/include/box3d/*.h \
  | sed -E 's/[[:space:]]*\($//; s/.*[ *]//' | sort -u > declared.txt
nm -g --defined-only linked-libs/linux-x64/libbox3d.a | awk '$2 == "T" { print $3 }' | sort -u > defined.txt

comm -13 bound.txt declared.txt     # declared but not bound
comm -23 declared.txt defined.txt   # declared but not built
comm -23 bound.txt defined.txt      # bound but not built: must print nothing
```

At the pinned box3d commit: 580 functions are declared, 578 of them are defined in the archive
(all but `b3World_DumpShapeBounds` and `b3InternalAssert`), and 575 are bound. The first `comm`
prints the five functions above, the second prints the two undefined ones, and the third prints
nothing.

## Targets and toolchain

| Target | Native library | Compiler | C runtime |
|---|---|---|---|
| `linux-x64` | `linked-libs/linux-x64/libbox3d.a` | GCC | glibc; links `m` and `pthread` |
| `windows-x64` | `linked-libs/windows-x64/box3d.lib` | MSVC | static (`"wincrt": "static"`) |

- C3 0.8.5.
- The release `b3.c3l` carries both archives. A clone builds only the host's, with
  `scripts/build-box3d.sh`; there is no cross-compilation.
- Windows is the MSVC ABI only. c3c's `mingw-x64` target is a different ABI and is not shipped.
- box3d links the static MSVC runtime. A project on the dynamic runtime fails to link with
  duplicate-symbol errors; set `"wincrt": "static"` in `project.json`.
- Struct layouts are pinned at compile time against box3d's headers. The two targets were probed
  and agree on every pinned size, alignment and offset, so one set of pins serves both.

## Float ABI only

The bindings use box3d's single-precision layout. box3d's large-world mode,
`BOX3D_DOUBLE_PRECISION`, widens `b3Pos` and every struct embedding it, and is not supported. The
shipped archives are built with it off. `b3::require_float_abi()` returns
`DOUBLE_PRECISION_LIBRARY` for a library built with it on.

Single precision bounds the useful world size. box3d's own guidance is that positions beyond about
16 km from the origin lose precision, and its sanity limit is 100 km (`1e5` length units).

## Fixed limits

Constants exposed by `b3`:

| Constant | Value | Limits |
|---|---|---|
| `MAX_WORLDS` | 128 | Worlds alive at once, replay players' private worlds included |
| `MAX_WORKERS` | 32 | Workers per world; larger `worker_count` values are clamped |
| `MAX_SHAPE_CAST_POINTS` | 128 | Points in a `ShapeProxy` |
| `MAX_MANIFOLD_POINTS` | 4 | Points in one contact manifold |
| `MAX_HULL_VERTICES` | 128 | Vertices `create_hull` keeps; larger requests are clamped |
| `MAX_HULL_RING_STEPS` | 64 | Sides of `create_cylinder`, slices of `create_cone` |
| `MAX_CHILD_SHAPES` | 1048576 | Children in one compound, all kinds together |
| `MAX_COMPOUND_MESH_MATERIALS` | 4 | Materials on one compound mesh child |
| `draw::GRAPH_COLOR_COUNT` | 24 | Constraint graph colours |

Limits inside box3d with no C3 constant:

- A hull has at most 128 faces and 128 edges; `create_hull` refuses point sets that need more.
- A step enqueues at most 256 tasks, so a task system can size a fixed array for them.
- A mover collide reports at most 64 planes per mesh, height field or compound shape.

## Version

`manifest.json` pins the vendored box3d in its `vendor` block, as a commit (`box3d-commit`) and a
`git describe` string (`box3d-describe`). The release README carries the same description.
`b3::version()` returns the version box3d reports for itself.

## Release-build implications

The shipped box3d is a Release build: every internal assertion is compiled out, and an assert
handler set with `b3::set_assert_fn` is never called.

The wrappers restore the checks whose absence would corrupt memory, hang, or return undefined data,
and report them as faults. Values that are only physically wrong, such as a NaN position, a
negative density or a zero quaternion, pass to box3d unchecked, as they would from C. So does an
identifier passed to an ordinary accessor. [Architecture](architecture.md#release-build) lists the
categories.

## Sharp edges

These cross several domains. Each domain page lists its own.

- **A panic skips `defer`.** C3's panic path does not run deferred calls, so a panic after
  `create_world` leaks that world, and with it its bytes, for the rest of the process. In a test
  run that continues after a failed test, one leaked world shifts every later `world_count()` and
  `byte_count()` reading. A single real failure followed by many count failures is that pattern,
  not many regressions. Caller-owned geometry (`MeshData`, `HeightFieldData`, compounds, hulls,
  recordings) leaks the same way.
- **Identifiers are trusted.** Only `is_valid()` detects a stale identifier. Any other call on one
  reads or writes whatever now holds its slot. See [Identifiers](architecture.md#identifiers).
- **Borrowed geometry.** Mesh, height field and compound shapes read caller memory on every step.
  Freeing it first is a silent use-after-free.
- **Refused builds can leak.** Some box3d builders free only part of what they allocated before
  refusing, notably mesh builders refusing degenerate geometry. Validate input rather than relying
  on the refusal.
- **A locked world ignores many calls.** During `world_step`, and inside its callbacks, many
  setters do nothing and many counters answer zero, without a fault. Constructors return
  `INVALID_HANDLE`.
- **Events are one step deep.** Each event slice holds the last step. An event caused between steps
  appears after the next one.
- **Flags differ in who must set them.** Contact, hit, pre-solve and custom-filter events need one
  of the two shapes to ask. Sensor events need both the sensor and the visitor.
- **World queries report sensors.** Exclude them with filter bits or in a callback.
- **Process-wide state.** The allocator, log handler and length unit are global to the process,
  and a live `record::Player` changes the length unit until destroyed.
- **Geometry changes keep the old mass.** Replacing a shape's geometry does not recompute the
  body's mass until `b3::body_apply_mass_from_shapes(body)`.
