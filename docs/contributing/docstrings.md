# Docstrings and comments

Contributing › Docstrings and comments

[style.md](style.md) §7 is the baseline: docstrings state the contract, short `//` comments say
why, and nothing narrates the code. This package relaxes the docstring length rule to a hard cap
and audits it, because binding docstrings carry contracts box3d states and the signature cannot.

## Limits

| Reading | Limit | Status |
|---|---|---|
| Prose lines in one docstring | 6 | gate, must be empty |
| Characters on any docstring line | 100 | gate, must be empty |
| Prose plus directive lines in one docstring | 10 | backlog, must never grow |
| Docstrings with prose at exactly 6 lines | none | backlog, must never grow |
| Inline `//` runs over 3 lines | none | backlog, must never grow |

Prose ends at the first `@` directive, which is where C3's doc parser stops reading free text.
The width limit and the 10-line budget exist so a contract cannot move into a long `@return` to
dodge the cap. Wrap a long description in backticks, which span lines where `"…"` does not.

Three lines is the target. Six is a ceiling, not a budget: a pile-up of docstrings at exactly six
means the limit chose the length.

## What earns length

A docstring earns up to six prose lines only for a contract the signature cannot carry, in this
priority order:

1. an assertion box3d's Release build compiles out (the caller's bad value is accepted silently);
2. an argument box3d silently corrects instead of refusing;
3. a zero that belongs to the state rather than the type;
4. an ownership rule a caller could break in a way that aborts.

A compiled-out assertion is never the line a trim cuts. When a contract needs more than six lines,
it belongs in the consumer docs under `docs/api/`, with one line in the docstring pointing there.

Always forbidden, at any length: narration of the body, prose restating the signature, design
history, and a second declaration's contract restated instead of cross-referenced.

## Cross-references

Cross-reference a reason, not a claim. A docstring that points at a sibling inherits whatever the
sibling says today, including a sentence that is false for the referring call or one a later trim
deletes.

When a review overturns a claim, fix the docstring carrying it in the same change. A correction
recorded anywhere else does not reach the consumer.

## Audits

```sh
./scripts/audit-comments.sh          # five readings
./scripts/audit-comments.sh --gate   # exits non-zero if either gate is non-empty
./scripts/audit-assertions.sh        # compiled-out assertions the docstring omits
```

`audit-comments.sh` is a required review step: a reviewer reports all five readings and treats a
violation as an Important defect. The script cannot judge whether a comment that fits is earning
its lines; that remains the reviewer's job.

`audit-assertions.sh` lists declarations whose C body asserts an argument's validity while the
docstring says nothing about it. It is a standing backlog, not a gate, and must never grow. Its
predicate list is all it can see; check it against `math_functions.h` whenever box3d moves.
