#!/bin/sh
# Prints every docstring whose body exceeds the cap in CLAUDE.md. Must print nothing.
# @param and @return tag lines are excluded from the count; the contract is capped, not the signature.
set -eu
CAP="${1:-6}"
cd "$(dirname "$0")/.."

for f in *.c3i *.c3 test/tests/*.c3; do
    [ -f "$f" ] || continue
    awk -v file="$f" -v cap="$CAP" '
        /^[[:space:]]*<\*/ { inside = 1; start = NR; body = 0; next }
        inside && /\*>/ {
            if (body > cap) printf "%s:%d: docstring body is %d lines, cap is %d\n", file, start, body, cap
            inside = 0
            next
        }
        inside {
            line = $0
            sub(/^[[:space:]]*/, "", line)
            if (line == "") next
            if (line ~ /^@(param|return)/) next
            body += 1
        }
    ' "$f"
done
