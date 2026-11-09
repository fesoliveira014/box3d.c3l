# box3d.c3l

C3 bindings for [box3d](https://github.com/erincatto/box3d), Erin Catto's 3D rigid-body physics
library written in C17.

The module is `b3`. It binds box3d's exported C API and wraps it in idiomatic C3: optionals with
named faults instead of null identifiers and status codes, slices instead of count-and-pointer
pairs, and box3d's vectors and matrices as the C3 standard library's own math types.

```c3
import b3;

fn void? simulate() {
    b3::require_float_abi()!;
    b3::WorldDef def = b3::default_world_def();
    b3::WorldId world = b3::create_world(&def)!;
    defer b3::destroy_world(world);
    for (int i = 0; i < 60; i++) b3::world_step(world, 1.0f / 60.0f, 4);
}
```

## Highlights

- Worlds, bodies, and every shape kind: sphere, capsule, convex hull, mesh, height field, and baked
  compound.
- All eleven joint types, including mover and pogo.
- Ray casts, shape casts, overlap queries, and contact data.
- Body, contact, sensor, and joint event arrays; pre-solve, custom-filter, friction, and
  restitution callbacks.
- The dynamic tree, the character mover, and explosions.
- Standalone geometry, distance, and manifold functions that need no world.
- Debug draw (`b3::draw`) and recording and replay (`b3::record`).
- Constructors that refuse a definition not obtained from its `default_*_def()`, which box3d's
  Release build would accept silently.

## Requirements and status

C3 **0.8.5_1**. Targets `linux-x64` (GCC, glibc 2.35 or newer) and `windows-x64` (MSVC, static
CRT). Each release archive carries a prebuilt box3d for its target, so consumers need neither CMake
nor a C compiler. box3d is pinned in `manifest.json` and built with the patches in
`patches/series`; see [NOTICE](NOTICE).

Pre-1.0. See [features and limitations](docs/features_and_limitations.md) for what is and is not
bound.

## Start here

- [Getting started](docs/getting_started.md): install, then a falling body and its contact events.
- [Documentation](docs/index.md): architecture, recipes, and API reference.
- [Public API](docs/api/index.md): conventions, faults, and every symbol by domain.

## Install

Download `b3-v<version>-linux-x64.c3l` or `b3-v<version>-windows-x64.c3l` from the
[latest release](https://github.com/fesoliveira014/box3d.c3l/releases/latest), put it in a directory
your project searches for libraries, and name `b3` as a dependency:

```json
{
  "dependency-search-paths": [ "libs" ],
  "dependencies": [ "b3" ]
}
```

The file is a packed `.c3l` that c3c reads as it stands and resolves by its `provides` name, `b3`.
Do not unzip it. On Windows, the project must also set `"wincrt": "static"`.

To build from a clone instead, see [getting started](docs/getting_started.md#install).

## Contributing

[AGENTS.md](AGENTS.md) holds the contributor rules. Details are in
[docs/contributing/](docs/contributing/bindings.md): binding conventions, docstrings, testing, and
releasing.

## License

MIT; see [LICENSE](LICENSE). box3d is also MIT; see [NOTICE](NOTICE) and
[LICENSE.box3d.mit](LICENSE.box3d.mit).
