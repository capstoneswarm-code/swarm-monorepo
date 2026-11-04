# Project guidance

Read [CONTRIBUTING.md](CONTRIBUTING.md) before changing code or preparing commits.
Use idiomatic Python 3.12 with useful function annotations, a main entry point for
scripts, and the project's formatting and lint checks. Keep Bash scripts consistent
with ShellCheck and shfmt.

Limit edits to the requested scope. The current cleanup covers the simulation
setup; it does not include the existing LiDAR viewer, the older `18500-swarm` repo,
upstream submodules, or installed libraries.

When commits are requested, make each commit a coherent, validated change with a
plain, descriptive imperative subject without prefixes such as `feat:` or `chore:`.
Use `capstoneswarm-code` with `capstoneswarm@gmail.com` as both author and committer,
and authenticate pushes as `capstoneswarm-code`. Leave commits unsigned as requested.
Do not commit automatically, stage unrelated work, or push without authorization.
Follow the commit examples and workflow in CONTRIBUTING.md.
