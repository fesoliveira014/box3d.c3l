# Building and releasing

Contributing › Building and releasing

## Native library

`vendor/box3d/` is the upstream C library as a git submodule. `scripts/build-box3d.sh` configures
and builds it with CMake into `linked-libs/<target>/`, then probes the struct layouts the binding
pins.

```sh
git submodule update --init
./scripts/build-box3d.sh            # build, print the probed layouts
./scripts/build-box3d.sh --check    # build, fail if a layout drifted from scripts/abi-sizes.txt
./scripts/build-box3d.sh --update   # build, rewrite abi-sizes.txt and src/layout.c3 (Linux only)
```

| Target | Archive | Compiler |
|---|---|---|
| `linux-x64` | `linked-libs/linux-x64/libbox3d.a` | GCC |
| `windows-x64` | `linked-libs/windows-x64/box3d.lib` | MSVC, from the developer environment |

- The script builds the host's target. There is no cross-compilation; the release workflow has a
  `windows-latest` job whose only outputs are the archive and the layout check.
- `scripts/abi-sizes.txt` is shared by both targets. The MSVC and GCC layouts were probed against
  the real headers on both platforms and agree on every pinned type's size, alignment, and field
  offsets, so one `src/layout.c3` serves both and `--check` on either platform gates the other.
  `--update` is Linux-only so the shared record has one author.
- box3d pins `CMAKE_MSVC_RUNTIME_LIBRARY` to `MultiThreaded`, the static CRT, so the manifest's
  `windows-x64` target sets `"wincrt": "static"`. A consumer on the dynamic CRT fails with
  duplicate-symbol errors that do not name the cause.
- The build must stay on the float ABI. `BOX3D_DOUBLE_PRECISION` is a PUBLIC compile definition
  that widens `b3Vec3`, `b3Pos`, and everything embedding them to double. The script forces it
  OFF; a build with it on corrupts every call. `b3::require_float_abi()` refuses such a library at
  run time.
- box3d builds in Release, so its `B3_ASSERT`s are compiled out. Wrappers guard the ones whose
  violation would corrupt memory.

### Moving the box3d submodule

1. Check out the new commit in `vendor/box3d`.
2. Run `./scripts/build-box3d.sh --check`. A drift means a bound struct changed: update the C3
   declaration, then `--update`.
3. Diff the `B3_API` declarations in `vendor/box3d/include/box3d/*.h` against the `@cname`s in
   `src/*.c3i` and bind or remove what changed.
4. Re-run `./scripts/audit-assertions.sh` and check its predicate list against
   `math_functions.h`.
5. Update the `vendor` block in `manifest.json` (`box3d-commit`, `box3d-describe`).

## Package

```sh
./scripts/package-release.sh 0.1.0
```

Writes `dist/b3.c3l` and `dist/b3.c3l.sha256`. The archive is a zip with `manifest.json` at its
root, plus `src/`, both native archives, the licences, the consumer docs (`docs/` without
`docs/contributing/`), and `scripts/README.dist.md` as `README.md`. c3c reads a packed `.c3l` as it
stands, unpacking it into `<build>/unpacked_c3l/` keyed on a CRC32, so consumers never rename or
extract it. `manifest.json` under a directory prefix fails with `Missing manifest`.

The script refuses a dirty working tree (an untracked file under `src/` would ship), requires both
native archives, and runs `build-box3d.sh --check` first.

## Release workflow

Pushing a `v*` tag runs `.github/workflows/release.yml`:

1. `windows`: build `box3d.lib`, check the layout pins, upload the archive.
2. `release`: install c3c 0.8.5 (the static tarball), build `libbox3d.a` with `--check`, fetch the
   Windows archive, compile the package, run `smoke`, `unit`, `unit-checked`, the comment gate,
   the assertion survey, and the docs gate, package, build `scripts/consumer-smoke/` against the
   packed archive alone, and publish `b3.c3l` and its checksum.

`scripts/consumer-smoke/` is a real project rather than a heredoc in the workflow so that an API
sweep over the repository reaches it.
