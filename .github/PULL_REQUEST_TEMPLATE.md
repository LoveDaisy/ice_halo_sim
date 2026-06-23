## Summary

<!-- Briefly describe the changes and their purpose. -->

## Test Plan

<!-- How were these changes tested? -->

## Checklist

- [ ] Tests pass (`./scripts/build.sh -tj release`)
- [ ] Code formatted (`./scripts/format.sh`)
- [ ] Policy checks pass (`python3 scripts/check_policies.py`) — also enforced by the CI `lint` job
- [ ] Documentation updated (if applicable)

### Judgment checks (delete a line only if clearly N/A)

These need a human decision the automated gate can't make — see `doc/env-var-policy.md` and `AGENTS.md`:

- [ ] **No new user-facing behavior switch hidden in an env var.** If this change adds a knob that alters the output a user sees, it goes through CLI / config / API — not `getenv`.
- [ ] **Any new `std::getenv` for a `LUMICE_*` knob** is routed through `src/util/env_knobs` and registered in `doc/env-var-policy.md`.
- [ ] **GUI/core API boundary respected** — `src/gui/` reaches core only via the C API (`src/include/lumice.h`), not direct `core/`/`config/` includes.
