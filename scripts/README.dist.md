# b3: C3 bindings for box3d

Version @VERSION@, built against box3d @BOX3D_DESCRIBE@ (Erin Catto's 3D rigid-body physics
library, C17). This package carries the sources, the consumer documentation, and a prebuilt static
box3d per target, so it needs no CMake, submodule, or C compiler.

## Use

Put `b3.c3l` in a directory your project searches for libraries and name `b3` as a dependency:

```json
{
  "dependency-search-paths": [ "libs" ],
  "dependencies": [ "b3" ]
}
```

c3c reads the packed `.c3l` as it stands. Do not unzip or rename it.

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

## Documentation

Start at `docs/index.md` inside this archive:

- `docs/getting_started.md`: a falling body and its contact events.
- `docs/architecture.md`: modules, naming, identifiers, ownership, faults, threading.
- `docs/cookbook.md`: recipes.
- `docs/api/index.md`: conventions, faults, and every symbol by domain.

Docstrings in `src/*.c3i` and `src/*.c3` are the authority for exact signatures.

## Targets

| Target | Archive | Notes |
|---|---|---|
| `linux-x64` | `linked-libs/linux-x64/libbox3d.a` | GCC |
| `windows-x64` | `linked-libs/windows-x64/box3d.lib` | MSVC, static CRT; set `"wincrt": "static"` |

c3c's `mingw-x64` is a different ABI and is not shipped. Both archives use box3d's float ABI;
`b3::require_float_abi()` refuses a double-precision build.

## License

MIT; see `LICENSE`. box3d is also MIT; see `NOTICE` and `LICENSE.box3d.mit`.
