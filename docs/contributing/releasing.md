# Building and releasing

Contributing › Building and releasing

## Native library

`vendor/box3d/` is the upstream C library as a git submodule, and it is never modified.
`patches/series` lists the patches applied to it, in order. `scripts/build-box3d.sh` copies the
pinned commit with `git archive` (line endings pinned to LF) into `build/box3d-src/`, applies the
series there, builds that copy with CMake into `linked-libs/<target>/`, then probes the struct
layouts the binding pins.

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
- The manifest's `windows-x64` target sets `"wincrt": "static"`, so the script configures box3d
  with `CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded`, the static CRT. box3d sets no runtime of its
  own, and CMake's default is the dynamic CRT. A consumer on the dynamic CRT fails with
  duplicate-symbol errors that do not name the cause.
- The Windows build fails unless `dumpbin` finds `box3d.lib` free of the dynamic CRT: no `__imp_`
  import of a CRT function (`__imp_fopen`, `__imp_remainderf`), which a static-CRT consumer cannot
  resolve, and no default library besides `LIBCMT` and `OLDNAMES`. Win32 imports such as
  `__imp_Sleep` are expected.
- The build must stay on the float ABI. `BOX3D_DOUBLE_PRECISION` is a PUBLIC compile definition
  that widens `b3Vec3`, `b3Pos`, and everything embedding them to double. The script forces it
  OFF; a build with it on corrupts every call. `b3::require_float_abi()` refuses such a library at
  run time.
- box3d builds in Release, so its `B3_ASSERT`s are compiled out. Wrappers guard the ones whose
  violation would corrupt memory.

### Patches

- A declaration a patch changes exists only in `build/box3d-src/include/`, never in the submodule:
  `b3HeightFieldDef.apronSides` is the current example. Bind from that copy.
- A patch that no longer applies fails the build with the patch's name (`git apply --check`).
- To change a patch: apply it to the submodule tree, edit, rewrite the patch file with
  `git diff`, and reset the submodule tree.
- `NOTICE` describes what the patches change; keep it in step with `patches/series`.

### Moving the box3d submodule

1. Check out the new commit in `vendor/box3d`.
2. Run `./scripts/build-box3d.sh --check`. A patch that fails to apply must be rebased first. A
   layout drift means a bound struct changed: update the C3 declaration, then `--update`.
3. Diff the `B3_API` declarations in `build/box3d-src/include/box3d/*.h` against the `@cname`s in
   `src/*.c3i` and bind, update, or remove what changed.
4. Re-run `./scripts/audit-assertions.sh` and check its predicate list against
   `math_functions.h`.
5. Update the `vendor` block in `manifest.json` (`box3d-commit`, `box3d-describe`).

## Package

```sh
./scripts/package-release.sh 0.1.0
```

Writes `dist/b3-v0.1.0-linux-x64.c3l`, `dist/b3-v0.1.0-windows-x64.c3l`, and `dist/SHA256SUMS`.
Each archive is a zip with `manifest.json` at its root, plus `src/`, that platform's native
archive, the licences, the consumer docs (`docs/` without `docs/contributing/`), and
`scripts/README.dist.md` as `README.md`, which names the patch series. Entries are sorted so an
archive is reproducible.

c3c reads a packed `.c3l` as it stands, unpacking it into `<build>/unpacked_c3l/` keyed on a CRC32,
and resolves it by the manifest's `provides`, not by file name. `manifest.json` under a directory
prefix fails with `Missing manifest`.

The script refuses a dirty working tree (an untracked file under `src/` would ship), requires both
native archives, and runs `build-box3d.sh --check` first.

## Release workflow

`.github/workflows/release.yml` runs on a `v*` tag, on every pull request, and on manual dispatch:

1. `windows`: build `box3d.lib`, check that it uses the static CRT and the layout pins, upload
   the archive.
2. `release` (Ubuntu 22.04): install c3c 0.8.5_1 (the static tarball), build `libbox3d.a` with
   `--check`, fail if the archive references a glibc symbol newer than 2.35, fetch the Windows
   archive, compile the package, run `smoke`, `unit`, `unit-checked`, the comment gate, the
   assertion survey, and the docs gate, package, build `scripts/consumer-smoke/` against the
   Linux archive alone, and upload the archives and `SHA256SUMS`.
3. On a tag only, publish a GitHub release with those files. A pull request or manual run packages
   as version `0.0.0-dev` and publishes nothing.

`scripts/consumer-smoke/` is a real project rather than a heredoc in the workflow so that an API
sweep over the repository reaches it.
