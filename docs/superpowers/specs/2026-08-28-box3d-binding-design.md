# box3d C3 binding — design

Date: 2026-08-28
Status: approved, not yet implemented

## Purpose

Bind [box3d](https://github.com/erincatto/box3d) — Erin Catto's C17 3D physics library — into a C3
library package (`box3d.c3l`) that a C3 project consumes as a normal dependency. The binding must
present a C3-native API: C3 types, C3 error handling, C3 naming. A caller should never see a raw
status value or a null handle standing in for an error.

The module is `b3`, taking the C library's own symbol prefix as its namespace, so `b3CreateWorld`
reads as `b3::create_world`. The prefix is therefore never part of an identifier — the namespace
carries it. The package directory keeps the library's full name, `box3d.c3l`; C3 resolves a
dependency by the manifest's `provides` name, not by directory.

## Constraints

- C3 0.8.3, the compiler installed at `/home/fesol/opt/c3/c3c`.
- `docs/style.md` is the style baseline. Its GPU and GLSL section does not apply here.
- The package ships a Release build of box3d, so `NDEBUG` is defined and box3d's own `B3_ASSERT`
  compiles to nothing. Misuse that box3d would catch in a debug build is silent here.
- Only `linux-x64` is built today.
- Planning vocabulary — milestone numbers, step numbers, plan references — must not appear in any
  identifier, filename, comment, docstring, test name, or string literal in the repository. The
  milestone names in this document exist only in this document.

## The surface

580 `B3_API` functions and 123 struct typedefs across eight headers:

| Header | Functions | Contents |
|---|---:|---|
| `box3d.h` | 419 | World 55, Body 80, Shape 50, nine joint types 186, recording 27 |
| `collision.h` | 107 | Geometry construction, distance and manifold queries, the dynamic tree |
| `math_functions.h` | 19 | Only the non-inline remainder; 105 more are `static inline` in the header |
| `types.h` | 18 | Definition constructors, 18 callback typedefs |
| `base.h` | 13 | Allocator, assert, and log hooks; version; timing |
| `constants.h` | 4 | |
| `id.h`, `config.h` | 0 | Types and macros only |

## Type mapping

`b3Vec3`, `b3Quat`, `b3Pos`, `b3Matrix3` and `b3Transform` were verified by compiling a C shim
against the real header layouts and calling it from C3 — as arguments and returns by value, and
nested inside structs. The remaining rows are derived from the header definitions and are pinned by
`$assert` when they are bound, not yet probed.

| box3d | C3 | Size |
|---|---|---:|
| `b3Vec3` | `float[<3>]` | 12 |
| `b3Quat` | `std::math::Quaternionf` | 16 |
| `b3Pos` | `double[<3>]` | 24 |
| `b3Matrix3` | `float[<3>][3]` | 36 |
| `b3Transform` | struct of the above | 28 |
| `b3WorldTransform` | struct of the above | 40 |
| `b3AABB` | struct of the above | 24 |
| `b3Plane` | struct of the above | 16 |
| `b3WorldId` | value struct, exact fields | 4 |
| `b3BodyId`, `b3ShapeId`, `b3JointId` | value structs, exact fields | 8 |
| `b3ContactId` | value struct, exact fields | 12 |

Two consequences follow from reusing the standard library types. Callers get `+`, `-`, `*`,
`.dot()`, `.cross()`, `.normalize()` and the quaternion operations for free, so the 105 header-only
inline math functions do not need porting — only the gaps do, discovered as milestones need them.
And `.x`, `.y`, `.z` work on C3 vectors, so field access reads the same as the C it replaces.

`b3Quat` maps onto `Quaternionf` because the layouts coincide: box3d stores `{b3Vec3 v; float s;}`
and C3 stores `{Real i, j, k, l;}` over a `Real[<4>]`, which is the same four floats in the same
order.

Positions are double and directions are float. This is box3d's large-world design, not an
oversight, and the binding preserves it rather than unifying on one precision.

### Layout discipline

Every fully declared struct carries `$assert T::size == N;`. `N` comes from
`scripts/build-box3d.sh`, which compiles a probe against the real headers and prints each type's
size and alignment. Type names go in `scripts/abi-types.txt`; the recorded values live in
`scripts/abi-sizes.txt`; `scripts/build-box3d.sh --check` fails when they diverge. No size is ever
written from reading a header by eye.

The ABI verification covers linux-x64 with the System V ABI. A three-float vector passed by value is
precisely the case where calling conventions differ between platforms, so adding a target means
re-running the probe on that target before trusting any of it.

The build must also stay on the float ABI. `BOX3D_DOUBLE_PRECISION` is a public compile definition
that widens `b3Vec3` and everything containing it; the build script forces it off.

## Error model

box3d reports no errors. Construction that fails returns an identifier whose `index1` is zero, and
misuse is undefined behavior. The binding converts both into C3's error handling, in three tiers.

**Construction always returns an optional.** `create_world`, `create_body`, `create_*_shape`, the
nine joint constructors, and the recording loaders return `T?` and fault on a null identifier:

```c3
WorldId? create_world(WorldDef* def)
```

Faults are named for the cause — `WORLD_LIMIT_REACHED`, `INVALID_DEFINITION` — one per line in a
`faultdef` in `box3d_check.c3`.

**Accessors are unchecked by default and checked on request.** Under `$feature(BOX3D_CHECKED)`,
every wrapper taking an identifier calls the corresponding `b3X_IsValid` first and returns a fault
instead of proceeding. Without the feature, the check is not compiled at all and the call is a
direct pass-through. This was verified: a consuming project that sets `"features":
["BOX3D_CHECKED"]` in its `project.json` does change code compiled from inside the `.c3l`.

The default is unchecked because a physics step touches thousands of bodies and an extra call per
access is real cost. The checked build exists because our Release build of box3d has its asserts
compiled out, so without it a stale identifier produces plausible garbage rather than a crash.

**No wrapper returns a null, a sentinel, or a raw status code.** Out-parameters become return
values. Pointer-and-count pairs become slices. Cleanup uses `defer`, and failure-only cleanup uses
`defer catch destroy`.

## Structure

One flat `module b3`, split by role and then by area:

```
box3d.c3i          types, identifiers, extern fn declarations, $assert pins
box3d.c3           wrappers for the base hooks and the world
box3d_body.c3      body wrappers
box3d_shape.c3     shape wrappers
box3d_joint.c3     joint wrappers
box3d_check.c3     faultdefs, validity helpers, null-identifier helpers
```

Later files appear when the area they cover is bound, not before. `box3d.c3i` splits by area once it
passes roughly a thousand lines. The manifest needs no `sources` entry: every `.c3` and `.c3i` at the
package root compiles into the library, which was verified by calling a function defined in
`box3d.c3` from the consumer project. Filenames keep the library's name; only the module is `b3`,
and C3 does not require the two to match.

The extern layer stays faithful to C — raw return types, raw out-parameters, no interpretation. All
interpretation happens in the wrapper layer. `@cname` carries the verbatim C symbol; no identifier on the
C3 side repeats the `b3` prefix the module already supplies. A C name shaped `b3Type_Method` becomes a method on that type.

The `static inline` functions in `id.h` and `math_functions.h` have no symbols in `libbox3d.a` and
cannot be bound. Where they are needed they are reimplemented in C3 and tested against the C
original.

## Milestones

Each milestone is independently testable and depends only on its predecessors. Counts are
approximate because some functions in an area are deferred to a later milestone that supplies the
types they need.

**Foundation.** Identifiers, the math type mapping, the thirteen `base.h` hooks, the faults, the
`$feature` switch, and the layout pins. Closed by: the size assertions pass and a custom allocator
hook observes box3d's allocations.

**World lifecycle.** `b3WorldDef`, `b3Capacity`, the scheduler and material-mixing callbacks,
create, destroy, step, gravity, and the sleeping, continuous, and warm-starting toggles. Roughly 35
functions. Closed by: a world is created, stepped sixty times, and destroyed; exhausting the world
limit produces `WORLD_LIMIT_REACHED` rather than a null identifier.

**Bodies.** `b3BodyDef`, `b3BodyType`, `b3MotionLocks`, mass data, transforms, velocities, forces,
damping, and sleep control. Roughly 60 of the 80 body functions; those returning shape, joint, or
contact collections wait for the milestones that bind those types. Closed by: a body released under
gravity arrives where the closed-form result says it should.

**Primitive shapes.** `b3ShapeDef`, `b3SurfaceMaterial`, `b3Filter`, sphere, capsule, and hull
construction, the shape property accessors, and mass computation. Roughly 55 functions. Closed by: a
box dropped onto static ground comes to rest on it — the first genuine simulation.

**Events.** The sensor, contact, body, and joint event structures, exposed through `World_Get*Events`
as C3 slices. Roughly 10 functions. Closed by: two bodies collide and the begin-touch event is
observed.

**Queries and casts.** `b3QueryFilter`, the ray and shape cast inputs and outputs, the world cast and
overlap functions with their result callbacks, and contact data. Roughly 35 functions. Closed by: a
ray fired at a known body reports the expected hit fraction.

**Joints.** The joint definition base and all nine joint types, 186 functions. Closed by: a distance
joint holds two bodies at its length and a revolute joint rotates about its axis.

**Complex geometry and tools.** Meshes, height fields, compound shapes, the character mover,
explosions, debug draw, the 107 standalone functions in `collision.h`, and recording and replay.
Closed by: a ball rolls across a height field.

## Testing

`test/` is a standalone consumer project that the package manifest never references, so consumers
never inherit its dependencies. It grows one source file per area, each named for the physical
behavior it demonstrates — `body_free_fall.c3`, `box_rests_on_ground.c3`, `ray_hits_body.c3`.

Every test asserts a physical outcome, not merely that a call returned. A binding error — a
transposed parameter, a wrong `@cname`, a struct field in the wrong order — usually still runs, and
usually still produces numbers. Only a test that knows what the numbers should be will catch it.

Verification before any commit: compile each binding file in isolation with `c3c compile-only
--no-obj`, build and run the consumer project, and run `scripts/build-box3d.sh --check` to confirm no
struct layout has drifted from its pins.

## Open questions

- Windows and macOS targets need their own ABI verification before the type mapping can be claimed
  for them.
- The eighteen callback typedefs, particularly the task scheduler, need a C3 calling-convention
  review before the world definition is bound.
- Whether `b3ContactId`'s explicit padding field should be exposed or hidden behind a constructor is
  deferred until contacts are bound.
