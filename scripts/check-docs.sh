#!/bin/sh
# The documentation gate. Two checks:
#
#   1. every public symbol declared in src/ appears in docs/api/index.md as `name`
#   2. every relative link in README.md and docs/ resolves, and no consumer page links into
#      docs/contributing/, which the release archive does not carry
#
# --list prints "area<TAB>symbol" for every public symbol instead, one per line, where area is the
# source file's base name. Methods print as Type.method. @private declarations, enum values,
# struct fields and src/layout.c3 are not symbols.
set -eu
cd "$(dirname "$0")/.."

list_symbols() {
    for f in src/*.c3i src/*.c3; do
        [ "$f" = src/layout.c3 ] && continue
        area=$(basename "$f"); area=${area%.*}
        awk -v area="$area" '
            # A declaration may span lines; @private can sit on its closing line.
            function flush() {
                if (pending != "" && decl !~ /@private/) printf "%s\t%s\n", area, pending
                pending = ""; decl = ""
            }
            /^[[:space:]]*<\*/ { doc = 1 }
            doc { if ($0 ~ /\*>/) doc = 0; next }
            pending != "" {
                decl = decl " " $0
                if ($0 ~ /[;{]|=>/) flush()
                next
            }
            infault {
                line = $0; gsub(/[[:space:],;]/, "", line)
                if (line != "") printf "%s\t%s\n", area, line
                if ($0 ~ /;/) infault = 0
                next
            }
            /^faultdef/ { infault = 1; next }
            /^(extern[[:space:]]+)?(fn|macro)[[:space:]]/ {
                head = $0; sub(/\(.*/, "", head)
                n = split(head, parts, /[[:space:]]+/)
                pending = parts[n]; decl = $0
                if ($0 ~ /[;{]|=>/) flush()
                next
            }
            /^(struct|union|enum|bitstruct|alias|typedef|constdef)[[:space:]]/ {
                name = $2; sub(/[^A-Za-z0-9_].*/, "", name)
                if ($0 !~ /@private/) printf "%s\t%s\n", area, name
                next
            }
            /^const[[:space:]]/ {
                head = $0; sub(/[[:space:]]*=.*/, "", head)
                n = split(head, parts, /[[:space:]]+/)
                if ($0 !~ /@private/) printf "%s\t%s\n", area, parts[n]
                next
            }
        ' "$f"
    done
}

if [ "${1:-}" = "--list" ]; then
    list_symbols
    exit 0
fi

status=0

missing=$(list_symbols | cut -f2 | sort -u | while read -r sym; do
    grep -qF "\`$sym\`" docs/api/index.md || echo "$sym"
done)
echo "== public symbols missing from docs/api/index.md (must be empty) =="
if [ -n "$missing" ]; then printf '%s\n' "$missing"; status=1; fi

broken=$(for page in README.md $(find docs -name '*.md'); do
    dir=$(dirname "$page")
    grep -o '](\([^)#]*\)[^)]*)' "$page" | sed 's/^](\([^)#]*\).*/\1/' | while read -r target; do
        case "$target" in ''|http://*|https://*|mailto:*) continue ;; esac
        if [ ! -e "$dir/$target" ]; then
            echo "$page: $target does not exist"
        fi
        case "$page" in
            docs/contributing/*|README.md) ;;
            *) case "$target" in *contributing/*) echo "$page: $target is not shipped" ;; esac ;;
        esac
    done
done)
echo "== broken or unshipped relative links (must be empty) =="
if [ -n "$broken" ]; then printf '%s\n' "$broken"; status=1; fi

exit $status
