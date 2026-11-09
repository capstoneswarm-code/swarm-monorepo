# Development and commit conventions

## Scope and Python version

The simulation runs Python 3.12 from Ubuntu 24.04 / ROS 2 Jazzy. Use features
supported by that interpreter even if the host has a newer Python version.

The current style checks cover the simulation setup and offline anomaly detector.
Ruff excludes the hardware demos in `src/fusion_test`, `src/motor_test`, and
`src/wheel_ekf_demo`, the existing
hardware viewer, and the upstream LiDAR driver. Review and update that scope
explicitly before extending checks to those files. Apply hardware style changes
as a separate task so merging the simulation does not reformat its control code.

## Python

- Annotate function parameters and return values, including `-> None` for functions
  that do not return a value. Annotate stored state when inference is insufficient.
- Prefer `list[str]`, `dict[str, ...]`, and `T | None` over older typing aliases.
- Keep scripts importable: do file writes, process startup, and other work inside
  `main()`, guarded by `if __name__ == "__main__":`.
- Extract functions around useful behavior, such as parsing or transforming
  configuration. Avoid classes or abstractions that do not simplify the code.
- Use concrete types when the data shape is known. Keep `Any` limited to genuinely
  dynamic boundaries, such as upstream RViz plugin configuration.
- Use `pathlib.Path`, explicit text encodings, descriptive names, and small,
  specific exception handlers. Let unexpected failures retain useful diagnostics.
- Use docstrings to explain contracts and intent; comments should explain reasons
  or constraints rather than repeat the statement below them.
- Format and lint with the checked-in Ruff configuration. It includes annotation,
  import-order, modern-syntax, and common correctness checks.

## Bash, Docker, and configuration

- Use a Bash shebang, consistent four-space indentation, quoted expansions, and
  arrays for lists of arguments or process IDs. Use `exec` for final commands.
- Enable `errexit`, `pipefail`, and `nounset` where compatible. The ROS helpers
  source the upstream ROS setup before enabling `nounset` because those setup
  scripts may reference unset variables.
- Keep ShellCheck exclusions specific and explained. The ROS source directive
  refers to a file available in the container rather than on the host.
- Keep signal cleanup for background processes and meaningful exit statuses.
- Keep Docker's dependency installation and cache cleanup together. Use JSON-form
  startup commands, declare runtime configuration in Compose, and preserve the
  common amd64/arm64 build path.
- Use two-space YAML indentation. Keep generated maps, bags, caches, and other
  runtime artifacts out of source commits.
- Update the run guide when commands or operational behavior change.

## Install development tools

These tools run on the host and do not require installing ROS there. From the repo:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
```

For the offline anomaly detector, also run `python -m pip install -e .` and
`python -m unittest discover -s tests/anomaly -v`. Its package and tests use the same
Python style checks. See [the anomaly-detection guide](docs/ANOMALY_DETECTION.md).

The pinned packages supply Ruff, ShellCheck, and shfmt. See the
[Ruff configuration documentation](https://docs.astral.sh/ruff/configuration/),
[ShellCheck package](https://pypi.org/project/shellcheck-py/), and
[shfmt package](https://pypi.org/project/shfmt-py/) for their configuration and packaging.

## Check changes

Run these from `swarm-monorepo`, with the development environment active:

```bash
ruff check .
ruff format --check .
shellcheck docker/sim/swarm-desktop docker/sim/swarm-sim \
  docker/sim/swarm-teleop docker/sim/swarm-save-map
shfmt -d -i 4 -ci docker/sim/swarm-desktop docker/sim/swarm-sim \
  docker/sim/swarm-teleop docker/sim/swarm-save-map
docker compose config --quiet
git diff --check
```

To apply formatting, use `ruff format .` and replace shfmt's `-d` with `-w`.
Review formatter changes before staging them. Ruff is not a static type checker;
annotations and lint success do not replace runtime verification.

For a change to simulation behavior, also check the affected behavior in the ROS
container. Relevant checks include generated RViz configuration, startup, sensor
topics, transforms, simulated motion, map saving, and shutdown. Use the
[simulation guide](docs/SIMULATION.md). Do not add tests that merely restate the
implementation or rerun unrelated checks without a reason.

## Files to keep in Git

Commit source code and the files teammates need to reproduce the project:

- Simulation scripts and `Dockerfile` under `docker/sim/`, plus `compose.yaml`.
- `README.md`, `docs/`, `CONTRIBUTING.md`, and shared agent guidance in `AGENTS.md`.
- `pyproject.toml`, `requirements-dev.txt`, `.editorconfig`, `.gitignore`, and
  `.dockerignore`.

Keep generated maps in `maps/`, ROS recordings in `bags/`, and screenshots or
other run output in `artifacts/`. These directories, build output, Python package
metadata, virtual environments, and caches are ignored. Put intentional reference
maps and small test fixtures in a separate source directory such as
`tests/fixtures/`; map images and YAML files are not ignored globally. ROS bag
file formats are ignored globally because recordings can be large; manage shared
recordings separately rather than committing them by default.

Keep actual local settings and credentials in ignored `.env` files or local
Compose override files. Commit sanitized examples such as `.env.example` when
needed. Review files before staging: ignore rules do not detect every secret or
remove files already tracked by Git. `.dockerignore` also excludes local settings
and generated output from the Docker build context.

## Commit size, frequency, and messages

Commit at useful milestones: one complete, understandable change that can be
reviewed or reverted independently. Examples include adding the simulation
environment, correcting a mapping bug, or improving its documentation. Commit
once the relevant checks pass; frequency follows completed work, not a timer or
one commit per file. Keep a necessary configuration change and its matching docs
together. Separate unrelated fixes or refactors.

Use plain, descriptive subjects in the imperative, usually within 72 characters.
Describe the resulting change without prefixes such as `feat:` or `chore:`:

```text
Add the single-robot mapping environment
Extract typed RViz configuration helpers
Stop child processes when the desktop exits
Explain how to build, drive, and save maps
Add Python and shell style checks
```

Use the body when needed to explain the reason, tradeoffs, relevant validation,
and remaining limitations. Describe only work and tests actually completed.

For this repository, use `capstoneswarm-code` as both author and committer, and
authenticate pushes with that GitHub account. Git commit identity and GitHub
authentication are separate: pushing with the team account does not change an
existing commit's author or committer. Configure this checkout locally:

```bash
git config --local user.name capstoneswarm-code
git config --local user.email capstoneswarm@gmail.com
git config --local commit.gpgsign false
```

Commits are unsigned as requested. These settings do not affect other repositories.

Before committing:

1. Inspect `git status --short` and the changes, including new files.
2. Run checks appropriate to that change.
3. Stage explicit paths or selected hunks so unrelated work stays unstaged.
4. Inspect `git diff --cached` and run `git diff --cached --check`.
5. Create the commit when requested. Preserve existing history unless rewriting
   it was explicitly authorized; push only when authorized.

Do not use messages such as "update", "stuff", or "misc changes" as the description
of an otherwise unexplained change. Attribute external code and AI assistance
according to the course requirements.
