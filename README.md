# box3d.c3l

C3 bindings for [box3d](https://github.com/erincatto/box3d), Erin Catto's 3D rigid body physics
library written in C17.

Status: box3d's exported surface is bound — 575 of the 580 `B3_API` functions across its headers.
Worlds and bodies, every shape kind — sphere, capsule, convex hull, mesh, height field and baked
compound — the world's event arrays with the pre-solve and custom-filter callbacks, the ray casts,
shape casts, overlap queries and contact data, all nine joint types, the dynamic tree and its two
traversals, the character mover and explosions, the world's two contact callbacks, the math and
validity singles, the standalone geometry, distance and manifold functions that answer about shapes
with no world in play, debug draw, and the recording and replay surface with the file and buffer
round trips that ride with it.

The other five are left out on purpose, and the reason is not the same one twice:

- `b3World_DumpShapeBounds` is declared in `box3d.h` and defined nowhere, so a call could not link.
- `b3InternalAssert` is box3d's own assertion hook, declared and defined behind the same
  `NDEBUG` guard, so it is absent from the Release build this package ships.
- `b3DynamicTree_Validate` and `b3DynamicTree_ValidateNoEnlarged` have bodies that compile to
  nothing unless box3d is built with validation on, which this package does not do. A bound call
  that cannot validate is worse than no call at all.
- `b3DynamicTree_GetProxyCount` reads the `proxy_count` field the binding already exposes, and a C3
  method may not share a name with one of its struct's fields. Read the field.

Rather than trust the paragraph above, derive it — and intersect three sets, not two, because a
header declaration is not evidence the symbol was built. Every bound symbol is a `@cname` in
`src/*.c3i`, every declared one a `B3_API` in `vendor/box3d/include/box3d/*.h`, and every one that
exists a `T` in `nm -g --defined-only linked-libs/linux-x64/libbox3d.a`.

## Using it

Download `b3.c3l` from a release, drop it into the directory your project searches for libraries,
and name `b3` as a dependency:

```json
{
  "dependency-search-paths": [ "libs" ],
  "dependencies": [ "b3" ]
}
```

That file is a packed `.c3l` — a zip c3c reads as it stands — and it carries the sources and a
prebuilt `libbox3d.a`, so a consumer needs neither this repository nor CMake. Do not unzip or
rename it. Building from a clone works too; see "Building the native library" below.

The module is `b3`, after the manifest's `provides` name — not the directory name `box3d.c3l`. The
module takes the C library's own symbol prefix, so `b3CreateWorld` in C reads as `b3::create_world`
in C3.

Two optional subsystems carry a namespace of their own, and `import b3;` reaches both because it
imports submodules recursively: debug draw is `b3::draw` (`draw::DebugDraw`,
`draw::default_debug_draw()`, `world.draw()`) and recording and replay is `b3::record`
(`record::Recording`, `record::Player`, `world.start_recording()`). The faults stay in `b3`
whichever module returns them — a `b3::record` call refuses with `b3::NOT_A_RECORDING`.

The per-type `.checked()` macros (`WorldId.checked()`, `BodyId.checked()`, …) turn a null
identifier into a named fault instead of undefined behavior; a caller opts in by routing an
identifier through one explicitly. The `BOX3D_CHECKED` feature does not gate them — it only flips
what `checked_access()` reports, for callers that want to branch on whether the build asked for
checking:

```json
{ "features": [ "BOX3D_CHECKED" ] }
```

## Building the native library

box3d itself is a submodule and is not committed as a binary. Build it once:

```sh
git submodule update --init
./scripts/build-box3d.sh
```

That produces `linked-libs/linux-x64/libbox3d.a`. `--check` verifies that no bound struct layout has
drifted from `scripts/abi-sizes.txt`; `--update` re-records them.

Only `linux-x64` is built today.

## Cutting a release

```sh
./scripts/package-release.sh 0.1.0
```

writes `dist/b3.c3l` and its `.sha256`. Pushing a `v*` tag runs the same thing under
`.github/workflows/release.yml`, which builds box3d, checks the layout pins, compiles the package,
runs both test targets and the comment audit, builds a consumer against the artifact it just made,
and uploads it.

## Conventions

The `b3` prefix is the module name and never appears inside an identifier — `b3::create_world`, not
`b3CreateWorld`. Functions are `snake_case`, types `PascalCase`. A C name shaped `b3Type_Method`
becomes a method on that type. Getters drop the `Get` verb; setters keep `Set` — `b3World_GetGravity`
becomes `world.gravity()`, `b3World_SetGravity` becomes `world.set_gravity(v)`. Construction with a
receiver already at hand becomes a method rather than a free function: `world.create_body(def)` is a
method on the world, not `create_body(world, def)`, because the world already exists at the call
site; `create_world` stays free because there is no world yet to own that call.
Operations that can fail return an optional with a named fault; no wrapper returns a null handle or a
sentinel. `create_world`, `world.create_body`, and the seven shape constructors
(`body.create_sphere_shape` and friends) in particular validate that their definition came from the
matching `default_*_def()` before calling into box3d — a behaviour they have that calling C directly
does not, since a release build of box3d compiles out its own check for this and would otherwise
construct silently with a garbage definition.

box3d's vectors and quaternions map onto the C3 standard library's own types, so the usual
operators and vector methods work directly on values that cross the ABI. Matrices are not a
standard-library matrix type: `Matrix3` is a struct of the three `Vec3` **columns** box3d
multiplies it as, named `cx`, `cy` and `cz`, so a caller filling it from rows cannot do so by
accident.

## License

MIT — see [LICENSE](LICENSE). box3d is also MIT; see [NOTICE](NOTICE) and
[LICENSE.box3d.mit](LICENSE.box3d.mit).
