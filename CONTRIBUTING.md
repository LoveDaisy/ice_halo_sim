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

Build output: `build/cmake_install/` (Release), `build/cmake_build/` (Debug)

### Running Tests

```bash
./scripts/build.sh -tj release            # Unit tests (GoogleTest via CTest)
./scripts/build.sh -gtj release           # GUI tests (requires display server)
pytest test/e2e/ -v               # E2E tests (requires Pillow)
```

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
5. **Open a pull request** — fill in the PR template (summary, test plan, checklist)
6. **CI checks** — ensure all checks pass (build, tests, format)
7. **Merge** — squash or rebase merge to keep history clean

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

1. **Update version**: `python scripts/version.py set X.Y.Z`
2. **Verify**: `python scripts/version.py check` (should exit 0)
3. **Commit** the version change
4. **Tag**: `git tag vX.Y.Z`
5. **Push**: `git push origin main --tags`

The release workflow (`.github/workflows/release.yml`) triggers automatically on `v*` tag pushes. It runs a version consistency check before building — if `CMakeLists.txt` version doesn't match the tag, the release is blocked.

### Before tagging

Always run `python scripts/version.py check` locally to verify version consistency before creating a tag. The CI check is a safety net, not a substitute for local verification.
