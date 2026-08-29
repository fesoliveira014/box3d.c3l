# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

C3 bindings for [box3d](https://github.com/erincatto/box3d) (Erin Catto's C17 3D physics library, MIT), packaged as a C3 **library** (`.c3l`) — not a standalone program.

- `manifest.json` — `provides` the `b3` module; `linklib-dir` is `linked-libs`. Consumers depend on `b3` (the `provides` name), not on the directory name `box3d.c3l`.
- `box3d.c3i` — types + raw `extern fn ... @cname(...)` declarations. `box3d.c3` — idiomatic wrappers. `box3d_check.c3` — `faultdef`s + the `check()` that maps C failure signals to faults. All three declare the same `module b3;` — the files keep the library's name, the module takes the C symbol prefix.
- `vendor/box3d/` — the upstream C library as a git submodule. `scripts/build-box3d.sh` configures and builds it with CMake into `linked-libs/<target>/libbox3d.a`; `--check` fails the build when a probed struct layout drifts from `scripts/abi-sizes.txt`, and `--update` rewrites that file. Only `linux-x64` is built today.
- The build must stay on the **float ABI**: `BOX3D_DOUBLE_PRECISION` is a PUBLIC compile definition that switches `b3Vec3`/`b3Pos` and everything embedding them to double. The script forces it OFF; a build with it on silently corrupts every call.
- `test/` — a standalone consumer project that exercises the bindings (`c3c build smoke` from `test/`). It is **not** part of the shipped library: `manifest.json` never references it, so consumers never inherit its deps. `test/libs/box3d.c3l` is a symlink to the repository root.

There is no `project.json` and no standalone build here. To syntax-check a file in isolation:

```sh
c3c compile-only --no-obj box3d.c3i     # remove the obj/ dir it leaves behind
```

`manifest.json` sets no `sources`, and it does not need to: every `.c3` and `.c3i` at the package root is compiled into the library. Verified — a `fn` defined in `box3d.c3` is callable from `test/`, and the filename does not need to match the `provides` name.

Do not assume `docs/style.md`'s `c3c build`/`c3c test` commands run in *this* repo — they describe a consuming project.

## Authoring rules — read before writing any C3

@docs/style.md is the style baseline and overrides this file where they conflict. Its §5 (GPU, GLSL & compute) does not apply to this repository; everything else does.

- **Invoke the `c3-expert`, `c3-style`, and `c3-bindings` skills** before writing, editing, or reviewing any C3, or diagnosing a c3c error. C3 is pre-1.0 and syntax drifts between releases. A review without them is not valid.
- **Target is C3 0.8.3**, which is what `/home/fesol/opt/c3/c3c` runs. (The glibc here is 2.35, too old for the stock release build — the install is the `c3-linux-static` tarball. The previous 0.8.0 install is kept at `/home/fesol/opt/c3-0.8.0.bak`.)
- **Do not run `c3fmt`** — it breaks the ≥4-argument named-parameter format and one-fault-per-line `faultdef`s. Hand-format.

## Binding conventions

- **The `b3` prefix is the module name, never part of an identifier.** `b3::create_world`, not `b3CreateWorld` and not `b3::b3_create_world`. Functions `snake_case`, types `PascalCase`, constants and enum values `SCREAMING_SNAKE_CASE` — the prefix stripped from all of them, carried by the namespace instead. No `@builtin`; it defeats the namespace.
- **`@cname("...")` carries the real C symbol verbatim**, exact casing: `extern fn void World.step(...) @cname("b3World_Step");`. (`@extern` as a rename attribute was removed in 0.8.x; `@cname` replaces it. The `extern fn` keyword is unchanged.)
- **`b3Type_Method` C names become C3 method syntax** on the corresponding type: `b3Body_SetTransform` → `extern fn void Body.set_transform(self, ...) @cname("b3Body_SetTransform");`.
- **The IDs are value structs, not opaque pointer handles** — `b3WorldId`, `b3BodyId`, `b3ShapeId` and friends are small structs passed and returned by value (box2d-v3 style). Declare them in full from the real header and pin the layout with `$assert T::size == N;`, with `N` coming from `scripts/build-box3d.sh` — add the type to `scripts/abi-types.txt` so drift is caught by `--check`. Never guess `N`, and never model an ID as `inline void*`; either corrupts the ABI silently.
- **Never bind from memory.** Read the actual declaration in the submodule's `include/box3d/*.h` before writing C3. A wrong parameter or return type is a silent cross-ABI memory bug.
- **Bind incrementally** — only the surface actually needed. The binding grows over time.

## Wrappers are where errors become faults

The `extern fn` layer stays faithful to C: raw return types, raw out-parameters, no interpretation. The `box3d.c3` wrapper layer is what callers use, and it is allowed — expected — to be a real C3 abstraction:

- Fallible operations return optionals (`?`) with named faults from `box3d_check.c3`. **Never** return null-as-error, a sentinel, or a raw C status code from a wrapper.
- An invalid/zero ID returned by C becomes a fault, not a value the caller has to test.
- Out-parameters become return values; `_count`/pointer pairs become slices.
- `defer` and `defer catch destroy` own cleanup.

## Repo conventions

- LICENSE is MIT (matching upstream). Vendored upstream carries its own `NOTICE` + `LICENSE.box3d.mit`.
- No development terminology in code: no issue refs, step numbers, or plan vocabulary in identifiers, comments, docstrings, or string literals.
