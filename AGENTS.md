# Contributor instructions

`box3d.c3l` is a C3 binding for [box3d](https://github.com/erincatto/box3d), Erin Catto's C17 3D
physics library, packaged as a C3 library (`.c3l`), not a program. The module is `b3` (the
manifest's `provides`); `b3::draw` and `b3::record` are its two submodules. The project targets
**C3 0.8.5_1** (`/home/fesol/opt/c3/c3c`).

## Before changing code

- Invoke the `c3-expert`, `c3-style`, and `c3-bindings` skills before writing, editing, or
  reviewing C3, or diagnosing a c3c error. C3 is pre-1.0; check syntax against the compiler, not
  memory. A review without them is not valid.
- Read [docs/contributing/bindings.md](docs/contributing/bindings.md) before touching `src/`.
  [docs/contributing/style.md](docs/contributing/style.md) is the baseline it refines.
- Never bind from memory. Read the declaration in `build/box3d-src/include/box3d/*.h`, the patched
  copy `scripts/build-box3d.sh` builds from; `vendor/box3d/` is the unpatched submodule.
- Build the native library once: `git submodule update --init && ./scripts/build-box3d.sh`.
- Bind incrementally: only the surface actually needed.

## Layout

| Path | Contents |
|---|---|
| `src/<area>.c3i` / `src/<area>.c3` | externs and types / wrappers, one pair per C API area |
| `src/check.c3` | faults, `.is_null()`/`.checked()` macros, shared private guards |
| `src/layout.c3` | generated `$assert` layout pins; never edit by hand |
| `vendor/box3d/` | upstream submodule, never modified |
| `patches/` | patches applied to box3d at build time, in `patches/series` order |
| `scripts/` | native build, layout probe, packaging, audits, docs gate |
| `test/` | standalone consumer project with the unit tests; not shipped |
| `docs/` | consumer docs (shipped); `docs/contributing/` is not shipped |

## Rules that corrupt memory when broken

- IDs are value structs declared field for field from the header, never `inline void*`.
- Layout pins are generated: add the type to `scripts/abi-types.txt`, run
  `./scripts/build-box3d.sh --update`. Never `--update` to silence a drift.
- Keep box3d on the float ABI; `BOX3D_DOUBLE_PRECISION` must stay off.
- Never edit `vendor/box3d/`. Change box3d through `patches/`; see
  [releasing.md](docs/contributing/releasing.md#patches).
- A caller-supplied pointer gets `@param [&in]` and an explicit null guard; release builds drop
  `[&in]`.
- Definitions must come from `default_*_def()`; wrappers check the cookie because box3d's Release
  build does not.

## Code rules

- The `b3` prefix is the module, never part of an identifier. Getters drop `get_`, setters keep
  `set_`. The C symbol alone decides method or free function; see
  [bindings.md](docs/contributing/bindings.md#method-or-free-function).
- Wrappers return optionals with named faults, declared with one `@return?` per fault. Never null,
  sentinels, or raw status codes.
- Docstrings: at most 6 prose lines, 100 characters per line, 10 lines with directives. Run
  `./scripts/audit-comments.sh`; see [docstrings.md](docs/contributing/docstrings.md).
- Do not run `c3fmt`. Hand-format.
- No development terminology (issue refs, step numbers, plan vocabulary) in code, comments, test
  names, or docs.

## Documentation

`README.md` and `docs/` (except `docs/contributing/`) are for consumers: current-state,
task-oriented, concise. Development history and plans do not go there.

When the public API changes:

- update the owning page under `docs/api/` and the symbol map in `docs/api/index.md`;
- update getting-started and cookbook examples that use it;
- compile every new C3 snippet against the library;
- run `./scripts/check-docs.sh`, which fails on a public symbol missing from the symbol map or a
  broken relative link.

## Verification

Full matrix in [docs/contributing/testing.md](docs/contributing/testing.md). Before every commit:

```sh
./scripts/build-box3d.sh --check
c3c compile-only --no-obj src/*.c3i src/*.c3 && rm -rf obj
(cd test && c3c build smoke && c3c test unit && c3c test unit-checked)
./scripts/audit-comments.sh --gate
./scripts/check-docs.sh
```

One test failure plus a burst of unrelated `world_count()`/`byte_count()` failures is one leaked
world: C3's panic path skips `defer`. Fix the first failure.

## Working in this repository

- Work on a branch in this checkout, not a git worktree: the built `linked-libs/**` archives are
  gitignored, so a fresh worktree cannot link the tests.
- Stage explicit paths whenever a background agent is editing the tree; `git add -A` sweeps its
  work into your commit.
- Never add a `Claude-Session` trailer or any Claude attribution to a commit or PR.
- Releases: [docs/contributing/releasing.md](docs/contributing/releasing.md).
