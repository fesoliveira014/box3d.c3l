# box3d.c3l

C3 bindings for [box3d](https://github.com/erincatto/box3d), Erin Catto's 3D rigid body physics
library written in C17.

Status: early. Worlds, bodies, the sphere, capsule and convex-hull shapes, the world's event
arrays, and the pre-solve and custom-filter callbacks are bound; the rest of the API surface
follows incrementally.

## Using it

Add the package to a C3 project's dependency search path and list `b3` as a dependency:

```json
{
  "dependency-search-paths": [ "libs" ],
  "dependencies": [ "b3" ]
}
```

The module is `b3`, after the manifest's `provides` name — not the directory name `box3d.c3l`. The
module takes the C library's own symbol prefix, so `b3CreateWorld` in C reads as `b3::create_world`
in C3.

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

## Conventions

The `b3` prefix is the module name and never appears inside an identifier — `b3::create_world`, not
`b3CreateWorld`. Functions are `snake_case`, types `PascalCase`. A C name shaped `b3Type_Method`
becomes a method on that type. Getters drop the `Get` verb; setters keep `Set` — `b3World_GetGravity`
becomes `world.gravity()`, `b3World_SetGravity` becomes `world.set_gravity(v)`. Construction with a
receiver already at hand becomes a method rather than a free function: `world.create_body(def)` is a
method on the world, not `create_body(world, def)`, because the world already exists at the call
site; `create_world` stays free because there is no world yet to own that call.
Operations that can fail return an optional with a named fault; no wrapper returns a null handle or a
sentinel. `create_world`, `world.create_body`, and the four shape constructors
(`body.create_sphere_shape` and friends) in particular validate that their definition came from the
matching `default_*_def()` before calling into box3d — a behaviour they have that calling C directly
does not, since a release build of box3d compiles out its own check for this and would otherwise
construct silently with a garbage definition.

box3d's vectors and quaternions map onto the C3 standard library's own types, so the usual
operators and vector methods work directly on values that cross the ABI. Matrices are a plain
array alias, `Vec3[3]`, not a standard-library matrix type.

## License

MIT — see [LICENSE](LICENSE). box3d is also MIT; see [NOTICE](NOTICE) and
[LICENSE.box3d.mit](LICENSE.box3d.mit).
