# Contributing to Lumice

Thank you for your interest in contributing to Lumice! This guide covers the development workflow and conventions used in this project.

## Development Setup

### Prerequisites

- C++17 compiler (GCC 13+, Clang 15+, Apple Clang 15+)
- CMake 3.14+
- Ninja build system
- Python 3.9+ (for E2E tests and version management)

### Building

```bash
./scripts/build.sh -j release             # Parallel release build
./scripts/build.sh -tj release            # Build + run unit tests
./scripts/build.sh -gtj release           # Build with GUI + all tests
./scripts/build.sh -k release             # Clean rebuild (preserves dependency cache)
```

Build output: `build/cmake_install/<flavor>/` (install tree), `build/cmake_build/<flavor>/` (CMake
build tree), where `<flavor>` is `shared` with `-s` and `static` without it

### Git hooks (recommended, one-time)

```bash
./scripts/install-hooks.sh    # non-interactive pre-commit: policy checks + clang-format on staged files
```

This mirrors the CI `policy` + `format-check` jobs locally so failures surface
before you push. It only adds a `pre-commit` hook and leaves the git-lfs hooks
intact. Bypass a single commit with `git commit --no-verify`. The CI jobs are
authoritative regardless of whether the hook is installed.

### Running Tests

```bash
./scripts/build.sh -tj release            # Unit tests (GoogleTest via CTest)
./scripts/build.sh -gtj release           # GUI tests (requires display server)
pytest -v                                 # E2E tests, fast subset — pinned by pyproject.toml
                                           # addopts, matches CI (requires Pillow)
```

See `AGENTS.md`'s "Testing and Platform Notes" for the full test-scope picture
(`scripts/test.sh`, the slow/full pytest marker, and which one to run when).

## Branch Naming

| Prefix | Purpose | Example |
|--------|---------|---------|
| `feature/` | New features | `feature/multi-crystal` |
| `bugfix/` | Bug fixes | `bugfix/asin-nan` |
| `release/` | Release preparation | `release/v4.1.0` |
| `refactor/` | Code refactoring | `refactor/config-parser` |

Use lowercase with hyphens. Keep branch names short and descriptive.

## Development Workflow

1. **Create a branch** from `main` using the naming conventions above
2. **Make changes** — keep commits focused and atomic
3. **Run tests locally** — at minimum `./scripts/build.sh -tj release`
4. **Format code** — run `./scripts/format.sh` before committing
5. **Pass policy checks** — `python3 scripts/check_policies.py` (env-knob centralization, GUI API boundary, using-namespace); installing the hook in step above runs this automatically
6. **Open a pull request** — fill in the PR template (summary, test plan, checklist)
7. **CI checks** — ensure all checks pass (policy, format-check, build, tests)
8. **Merge** — squash or rebase merge to keep history clean

### Commit Messages

Follow the conventional commit format:

```
<type>(<scope>): <subject>
```

- **type**: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`
- **scope**: module name (e.g., `core`, `cli`, `gui`, `config`) or task name
- **subject**: concise description, under 50 characters

## Code Style

- **C++**: Google C++ Style, enforced by `.clang-format` and `.clang-tidy`
- **Editor settings**: `.editorconfig` — 2-space indent, UTF-8, LF line endings
- **Formatting**: Run `./scripts/format.sh` to format all C++ source files

See `CLAUDE.md` for detailed naming conventions and coding guidelines.

## Release Process

A release is a **chore** with its own worktree and branch, not a commit made directly on `main`.
The version's `CHANGELOG.md` section is written in that chore, in one batch, from a mechanical
enumeration of every PR merged since the previous tag — never from memory, and not accumulated
PR by PR beforehand (`CHANGELOG.md`'s own "Maintaining this file" rule says why). The tag is
the owner's to create, on the merge commit.

1. **Version number**: the owner decides `X.Y.Z`. Semver is read against what a *user* can
   perceive — CLI, GUI, config files, produced images and files. The C API is part of the
   product only insofar as it ships as a library; while it does not, a C API break alone does
   not force a major bump, though it still earns a `⚠️ Breaking Changes` entry.
2. **Bootstrap the chore**: `/chore-bootstrap release-X.Y.Z`, then work it in a linked worktree
   on its own branch (`AGENTS.md`, "Where a change lives, and from where it is made").
3. **Enumerate, then decide per PR**: run the three commands under "Sourcing" in `CHANGELOG.md`
   (merge commits, first-parent non-merges, and the `LUMICE_API_VERSION` diff — with `main` in
   place of the tag that does not exist yet). For every PR in the output, record in the chore's
   progress notes a **per-PR disposition**: entry written (and under which heading — `Added` /
   `Changed` / `Fixed` / `Breaking Changes`), or no entry plus one sentence of why. The table is
   the audit trail that the section is complete; it stays in the chore, not in the changelog.
4. **Write the section and set the version**: add `## [X.Y.Z] - <UTC date>` at the top of the
   version list in `CHANGELOG.md`, with its entries under the file's headings, then
   `python scripts/version.py set X.Y.Z` — it bumps `CMakeLists.txt`, checks that the dated
   section exists and holds at least one bullet (`--allow-empty-changelog` is the escape hatch
   for a release with genuinely no user-perceptible change), and adds the version's link
   definition. Both files are computed and validated before either is written.
   Then `python scripts/version.py check --tag vX.Y.Z` must exit 0 — pass `--tag` explicitly:
   without it the check compares against the *previous* tag, which no longer matches — and
   read `python scripts/version.py extract-notes X.Y.Z` end to end: that output is the release
   page's body verbatim, so how it reads is how the release page reads.
5. **PR → CI → merge**: open the PR from the chore branch; the last commit is the cut
   (`chore(release): cut X.Y.Z`), on top of the changelog backfill. Merge as usual.
6. **Tag** (owner): `git tag vX.Y.Z` on the merge commit, then `git push origin vX.Y.Z`.
   The release workflow (`.github/workflows/release.yml`) triggers on the `v*` tag push, runs
   the same `check --tag` before building — a `CMakeLists.txt` / `CHANGELOG.md` mismatch blocks
   the release — and uses `extract-notes` for the release page body.

There is one shape of release, not two: every version is cut together with its backfilled
section, so a "cut only" commit on `main` with the notes written some other time does not
occur.

The release produces platform-specific packages:
- **Linux x64**: `.tar.gz` with CLI and GUI executables, each shipped twice (`<name>.baseline`,
  `<name>.x86-64-v4`) behind a CPUID launcher installed as `Lumice` / `LumiceGUI` — see
  `doc/performance-testing.md`, "A local build is not the shipped binary"
- **Linux ARM64**: `.tar.gz` with CLI executable (excludes GUI due to runner GPU limitations)
- **macOS ARM64**: `.tar.gz` with CLI executable and `LumiceGUI.app` bundle
- **Windows x64**: `.zip` with CLI and GUI executables, each shipped twice (`<name>.baseline.exe`
  built by MSVC cl.exe, `<name>.x86-64-v3.exe` built by clang-cl) behind a CPUID launcher installed
  as `Lumice.exe` / `LumiceGUI.exe` — the same shape as Linux x64, see the same doc section

The release page's body is that version's `CHANGELOG.md` section; GitHub's auto-generated pull-request list follows it as an appendix.

### Before tagging

Always run `python scripts/version.py check --tag vX.Y.Z` locally to verify version and changelog consistency before creating a tag. The CI check is a safety net, not a substitute for local verification.
