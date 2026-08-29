# box3d binding smoke test

A standalone consumer project that compiles and runs against the `box3d` binding in the
parent directory. It is deliberately **not** referenced by the package `manifest.json`, so
consumers of `box3d` never inherit anything from here.

`libs/box3d.c3l` is a symlink to the repository root — the package resolves by its
`provides` name (`box3d`), not by directory name.

Prerequisites: `linked-libs/linux-x64/libbox3d.a` must exist. Build it with
`scripts/build-box3d.sh` from the repository root.

```sh
c3c build smoke     # from this directory
./build/smoke
```
