# b3 — C3 bindings for box3d

Version @VERSION@, built against box3d @BOX3D_DESCRIBE@ (Erin Catto's 3D rigid body physics
library, C17). This package ships the sources and a prebuilt `linked-libs/linux-x64/libbox3d.a`,
so nothing here needs CMake, a submodule or a C compiler.

box3d's exported surface is bound — 575 of the 580 `B3_API` functions. Worlds and bodies, every
shape kind, the world's event arrays and callbacks, casts, overlaps and queries, all nine joint
types, the dynamic tree, the character mover and explosions, the standalone geometry, distance and
manifold functions, debug draw, and recording and replay.

## Using it

Drop `b3.c3l` into the directory your project searches for libraries, and name `b3` as a
dependency:

```json
{
  "dependency-search-paths": [ "libs" ],
  "dependencies": [ "b3" ]
}
```

The file is a packed `.c3l` — c3c reads it as it stands, so do not unzip or rename it.

```c3
import b3;

b3::WorldDef def = b3::default_world_def();
b3::WorldId world = b3::create_world(&def)!;
defer world.destroy();
world.step(1.0f / 60.0f, 4);
```

## Modules

Most of the API is `b3`. Two optional subsystems carry the namespace their C prefix named, and
`import b3;` reaches both — it imports submodules recursively:

| Module | What it holds |
|---|---|
| `b3` | worlds, bodies, shapes, joints, geometry, casts, queries, the dynamic tree, the mover, the faults |
| `b3::draw` | `draw::DebugDraw`, `draw::DebugShape`, `draw::default_debug_draw()`, `draw::graph_color()`, `world.draw()` |
| `b3::record` | `record::Recording`, `record::Player`, `record::create_player()`, `world.start_recording()` |

The faults live in `b3` whichever module returns them: a `b3::record` call refuses with
`b3::NOT_A_RECORDING`.

## Checked access

The per-type `.checked()` macros (`WorldId.checked()`, `BodyId.checked()`, …) turn a null
identifier into a named fault instead of undefined behaviour; a caller opts in by routing an
identifier through one explicitly. The `BOX3D_CHECKED` feature does not gate them — it only flips
what `checked_access()` reports, for callers that want to branch on whether the build asked for
checking:

```json
{ "features": [ "BOX3D_CHECKED" ] }
```

## Conventions

The `b3` prefix is the module name and never appears inside an identifier — `b3::create_world`, not
`b3CreateWorld`. Functions are `snake_case`, types `PascalCase`. A C name shaped `b3Type_Method`
becomes a method on that type. Getters drop the `Get` verb; setters keep `Set` — `b3World_GetGravity`
becomes `world.gravity()`, `b3World_SetGravity` becomes `world.set_gravity(v)`. Construction with a
receiver already at hand becomes a method rather than a free function: `world.create_body(def)`,
not `create_body(world, def)`; `create_world` stays free because there is no world yet to own it.

Operations that can fail return an optional with a named fault; no wrapper returns a null handle or
a sentinel. `create_world`, `world.create_body` and the seven shape constructors validate that their
definition came from the matching `default_*_def()` before calling into box3d — a behaviour they
have that calling C directly does not, since a release build of box3d compiles out its own check
and would otherwise construct silently from a garbage definition.

box3d's vectors and quaternions map onto the C3 standard library's own types, so the usual
operators and vector methods work directly on values that cross the ABI. Matrices are not a
standard-library matrix type: `Matrix3` is a struct of the three `Vec3` **columns** box3d
multiplies it as, named `cx`, `cy` and `cz`, so a caller filling it from rows cannot do so by
accident.

## Target support

`linux-x64` only. The package is built against the float ABI — a box3d built with
`BOX3D_DOUBLE_PRECISION` widens every position and would corrupt every call, so
`b3::require_float_abi()` refuses one.

## License

MIT — see `LICENSE`. box3d is also MIT; see `NOTICE` and `LICENSE.box3d.mit`.
