#!/bin/sh
# Builds the distributable package: a packed .c3l, which c3c reads as a zip whose root holds
# manifest.json. The name is b3.c3l rather than box3d-<version>.zip because c3c's own
# `project fetch` looks for <search-path>/<dependency>.c3l, and the dependency name is the
# manifest's `provides`.
#
# What ships is what a consumer compiles and links against, and nothing else: the sources, the
# static library, the licences and a consumer README. Not the vendored submodule, not the build
# and audit scripts, not the test project.
set -eu

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="linux-x64"
VERSION="${1:-}"
DIST="$ROOT/dist"
STAGE="$DIST/.stage"
ARCHIVE="$DIST/b3.c3l"

[ -n "$VERSION" ] || { echo "usage: scripts/package-release.sh <version>" >&2; exit 1; }

# An untracked file counts: src/ is copied from the working tree, so a source never committed
# would ship. A dirty submodule working tree does not -- vendor/ is not in the artifact -- but a
# submodule at a commit other than the recorded one is, since it is what libbox3d.a was built from.
dirty="$(git -C "$ROOT" status --porcelain --ignore-submodules=dirty)"
if [ -n "$dirty" ]; then
    echo "ERROR: the working tree is dirty. A release is cut from a committed state:" >&2
    printf '%s\n' "$dirty" >&2
    exit 1
fi

if [ ! -f "$ROOT/vendor/box3d/CMakeLists.txt" ]; then
    echo "ERROR: vendor/box3d is empty. Run: git submodule update --init" >&2
    exit 1
fi

# A shipped archive whose layout no longer matches the $assert pins compiled beside it would
# corrupt every call it is used for, so this is a gate rather than a convenience.
"$ROOT/scripts/build-box3d.sh" --check

rm -rf "$STAGE" "$ARCHIVE" "$ARCHIVE.sha256"
mkdir -p "$STAGE/src" "$STAGE/linked-libs/$TARGET"

cp "$ROOT"/src/*.c3 "$ROOT"/src/*.c3i "$STAGE/src/"
cp "$ROOT/linked-libs/$TARGET/libbox3d.a" "$STAGE/linked-libs/$TARGET/"
cp "$ROOT/LICENSE" "$ROOT/LICENSE.box3d.mit" "$ROOT/NOTICE" "$STAGE/"

BOX3D_COMMIT="$(git -C "$ROOT/vendor/box3d" rev-parse --short HEAD)"
BOX3D_DESCRIBE="$(git -C "$ROOT/vendor/box3d" describe --tags 2>/dev/null || echo "$BOX3D_COMMIT")"
sed -e "s/\"version\": \"[^\"]*\"/\"version\": \"$VERSION\"/" \
    -e "s/\"box3d-commit\": \"[^\"]*\"/\"box3d-commit\": \"$BOX3D_COMMIT\"/" \
    -e "s/\"box3d-describe\": \"[^\"]*\"/\"box3d-describe\": \"$BOX3D_DESCRIBE\"/" \
    "$ROOT/manifest.json" > "$STAGE/manifest.json"

sed -e "s/@VERSION@/$VERSION/g" \
    -e "s/@BOX3D_DESCRIBE@/$BOX3D_DESCRIBE/g" \
    "$ROOT/scripts/README.dist.md" > "$STAGE/README.md"

# manifest.json must sit at the archive root: c3c reports "Missing manifest" for an archive that
# carries it under a directory prefix.
( cd "$STAGE" && zip -q -r -X "$ARCHIVE" manifest.json README.md LICENSE LICENSE.box3d.mit NOTICE src linked-libs )
rm -rf "$STAGE"

( cd "$DIST" && sha256sum b3.c3l > b3.c3l.sha256 )

echo "Wrote $ARCHIVE ($(wc -c < "$ARCHIVE") bytes), box3d $BOX3D_DESCRIBE, version $VERSION"
