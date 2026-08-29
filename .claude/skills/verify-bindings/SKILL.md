---
name: verify-bindings
description: Compile-check the b3 C3 binding bundle and build the test/ consumer project against it. Use before every commit, after editing any .c3/.c3i in this repo, or when asked to verify, check, or test the bindings.
---

# Verify the box3d bindings

This package has no `project.json` and no standalone build, so verification is three steps: compile-check the binding package, compile a real consumer against them, then check the ABI pins.

## 1. Compile-check the binding package

`module b3` spans `box3d.c3i`, `box3d.c3`, and `box3d_check.c3`, so a single-file invocation cannot see the declarations its siblings provide — compile them together in one invocation.

```sh
cd /home/fesol/source/repos/box3d.c3l
c3c compile-only --no-obj box3d.c3i box3d.c3 box3d_check.c3
rm -rf obj
```

This catches syntax and type errors but not linkage or ABI problems.

## 2. Build the consumer project

```sh
cd /home/fesol/source/repos/box3d.c3l/test
c3c build smoke
./build/smoke
```

Step 1 alone is not proof the binding works — it does not link, so it cannot catch a wrong `@cname` or a mismatched ABI. Only the consumer build does.

This step needs `linked-libs/linux-x64/` populated. If it is empty, the native library has not been built: run `scripts/build-box3d.sh` from the repository root and report if the submodule is not checked out rather than skipping the step silently.

## 3. Check the ABI pins

```sh
cd /home/fesol/source/repos/box3d.c3l
./scripts/build-box3d.sh --check
```

This rebuilds box3d and compares every type in `scripts/abi-types.txt` against `scripts/abi-sizes.txt`. A drift here means the `$assert` pins in `box3d.c3i` are wrong and calls will corrupt memory. Do not paper over it with `--update` unless the binding is being updated to match in the same change.

## 4. Report honestly

State exactly which steps ran and which passed. Quote the shortest decisive compiler error line for any failure. If a step was skipped — no `test/`, empty `linked-libs/`, `c3c` not matching the 0.8.3 target this repo declares — say which step and why. Do not claim the bindings are verified when only step 1 ran.
