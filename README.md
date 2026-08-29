# box3d.c3l

C3 bindings for [box3d](https://github.com/erincatto/box3d), Erin Catto's 3D rigid body physics
library written in C17.

Status: early. The package structure, native build, and design are in place; the API surface is
being bound incrementally.

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

Set the `BOX3D_CHECKED` feature to compile validity checks into every wrapper that takes an
identifier, which turns a stale or destroyed handle into a fault instead of undefined behavior:

```json
{ "features": [ "BOX3D_CHECKED" ] }
```

Leave it off in a release build; the checks cost a call per operation.

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
becomes a method on that type.
Operations that can fail return an optional with a named fault; no wrapper returns a null handle or a
sentinel.

box3d's vectors, quaternions, and matrices map onto the C3 standard library's own types, so the
usual operators and vector methods work directly on values that cross the ABI.

## License

MIT — see [LICENSE](LICENSE). box3d is also MIT; see [NOTICE](NOTICE) and
[LICENSE.box3d.mit](LICENSE.box3d.mit).
