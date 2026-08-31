"""Every backend route must charge the SAME emitted energy for the same config.

Emitted energy is the renderer's normalization denominator, and it is written at
four sites in the simulator and carried to the consumer by three different
transports: the legacy route hands over a whole SimData, the exit-seam route
splits one into commit-sized chunks that are copied field by field, and the
device-fused route drains a window that aggregates many batches. Emitted energy
is not a property of any of that — it is the light source times the ray budget —
so all three must land on the same number, and a disagreement is a per-backend
brightness divergence rather than a rounding detail.

The transports are what make this worth its own test rather than an inference
from the field being set once. Two of them silently drop a field they were not
explicitly taught about:

  * The chunk split copies fields by hand and leaves anything unlisted at its
    default zero. A zero denominator makes ExposureScale() return 0, i.e. a
    BLACK image, and only on the exit-seam route.
  * A batch whose rays are all rejected produces no pixels, and the server used
    to drop it whole. It still emitted its rays, so under an absolute scale it
    still owes the denominator; dropped, the denominator would count only the
    batches that survived their filter -- re-brightening a filtered scene by
    exactly the amount the filter removed, which is the content-dependence the
    absolute scale exists to remove.

Both were verified to produce emitted=0 on the cpu_backend route when the code
that handles them is removed, while legacy and Metal stayed correct -- which is
also why neither shows up on a machine that only runs the default route.
"""

from __future__ import annotations

import pytest

from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

_SEED = 42

# Float32 accumulation over tens of thousands of batches, summed in a different
# order per route (per batch on the CPU routes, per drained window on the device
# one). Measured spread is ~1e-6; anything structural is a factor, not a ulp.
_REL_TOL = 1e-4

# "cpu_backend" drives the exit-seam transport with its chunk split; "metal" the
# device-fused window drain; the default is the legacy whole-SimData handover.
_ROUTES = ("legacy", "cpu_backend", "metal")

_CASES = (
    # A plain scene: exercises the chunk split on a route that produces exit rays.
    "cpu_backend_route",
    # An impossible raypath filter, so EVERY batch is rejected in full and lands
    # in the "nothing to render" branch. The image is legitimately black here --
    # the denominator must not be.
    "ms_filter_leak_impossible",
)


def _emitted(config: str, route: str):
    return run_scene_capi_buffered(
        str(CONFIGS_DIR / f"{config}.json"),
        sim_seed=_SEED,
        backend=route,
        timeout_sec=900,
    )


@pytest.mark.slow
@pytest.mark.parametrize("config", _CASES)
def test_every_route_charges_the_same_emitted_energy(config):
    results = {}
    for route in _ROUTES:
        r = _emitted(config, route)
        if r.fell_back:
            pytest.skip(f"{route}: backend fell back to legacy on this host — routes not distinct")
        results[route] = r

    reference = results["legacy"].emitted_energy
    assert reference > 0.0, (
        f"{config}: the legacy route charged no emitted energy at all — "
        "the denominator is never being written"
    )

    for route, r in results.items():
        assert r.emitted_energy == pytest.approx(reference, rel=_REL_TOL), (
            f"{config}: route {route!r} charged {r.emitted_energy!r} against the legacy "
            f"route's {reference!r}. Emitted energy does not depend on how a batch is "
            "transported to the consumer, so this is a per-backend normalization "
            "divergence — a route rendering at its own brightness, or (at 0) black."
        )
