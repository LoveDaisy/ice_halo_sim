"""Every backend route must publish the SAME exposure anchor for the same scene.

The anchor is the P99 radiance of the sky, measured on a fixed full-sky buffer
that no renderer configuration touches. Its value is a property of the scene, so
two things must hold and neither is implied by the other:

  1. All three routes must agree, even though they accumulate the plane in
     structurally different places. The two CPU routes project on the HOST, out
     of the outgoing rays the seam hands over; the device-fused route cannot —
     by the time the host sees anything the directions are already pixels — so
     it accumulates a second target inside the trace kernel and drains it. That
     is three separate accumulation sites for one number, and only a comparison
     across routes can tell whether one of them was taught a different geometry,
     missed an exit tail, or reset its buffer on the wrong clock.

     "Agree" is not one bar for all three, because the routes do not all trace
     the same rays. The two host routes share an RNG stream and must match to the
     BIT; the device route draws its own, so it traces a different realization of
     the same sky and is held to the statistic's own measured seed-to-seed spread
     instead. Those are two separate tests below, deliberately — collapsing them
     onto one tolerance would mean either lying about the host pair or excusing
     the device one.

  2. It must not move when the renderer's LENS does. That is the property the
     anchor exists for, and it is what separates this number from the P99 the
     renderer already takes over its own output buffer — which moves by roughly
     a stop across the same pair of configs. Checking only (1) would pass for a
     backend trio that all agreed on the wrong, lens-dependent quantity.

Both halves run on the SAME scene so a difference can only come from the thing
being varied.
"""

from __future__ import annotations

import json
import math
import tempfile
from pathlib import Path

import pytest

from test.e2e.capi_runner import run_scene_capi_buffered
from test.e2e.runner import get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"

_SEED = 42

# Seeds used to measure the statistic's OWN Monte-Carlo spread — see
# test_device_fused_route_agrees_within_monte_carlo_noise for why the device route
# cannot be compared against a fixed tolerance.
_SPREAD_SEEDS = (42, 43, 44)

# The two HOST routes share one RNG stream and differ only in how a batch is
# transported to the consumer, so they trace the same sky ray for ray and must
# agree to the bit. Measured: identical float32 at all three seeds above. This is
# the sharp assertion — it is what catches a chunk split that forgot a field or an
# accumulation that depends on the commit grain.
_HOST_REL_TOL = 1e-6

# If the anchor's own seed-to-seed spread ever exceeds this, the scene has stopped
# being able to discriminate anything and the device-route comparison below would
# be vacuous rather than passing. Measured spread on this scene: ~1.1%.
_MAX_USEFUL_SPREAD = 0.10

_ROUTES = ("legacy", "cpu_backend", "metal")

_CONFIG = "cpu_backend_route"


def _run(config_path: str, route: str, seed: int = _SEED):
    return run_scene_capi_buffered(
        config_path,
        sim_seed=seed,
        backend=route,
        timeout_sec=900,
    )


def _with_lens(config_path: Path, lens: dict, resolution: list) -> Path:
    """Copy a config, replacing every renderer's lens and resolution.

    Only the RENDERER is touched — the scene, the ray budget and the seed are
    left alone — so the sky being measured is identical between the two variants
    and any movement in the anchor is attributable to the lens alone.
    """
    doc = json.loads(config_path.read_text())
    for r in doc.get("render", []) if isinstance(doc.get("render"), list) else [doc["render"]]:
        r["lens"] = lens
        r["resolution"] = resolution
    out = Path(tempfile.mkdtemp()) / f"{config_path.stem}_{lens['type']}_{resolution[0]}.json"
    out.write_text(json.dumps(doc))
    return out


@pytest.mark.slow
def test_host_routes_publish_the_bit_identical_anchor():
    """legacy vs cpu_backend: same rays, different transport, so no slack at all.

    These two routes draw from one RNG stream and trace the same sky; all that
    differs is how the batch reaches the consumer — legacy hands over a whole
    SimData, the exit seam splits it into commit-sized chunks copied field by
    field. The anchor is not a property of that, so anything but bit-equality here
    is a transport defect (a chunk that dropped the rays, an accumulation that
    scales with the commit grain), not noise.
    """
    results = {}
    for route in ("legacy", "cpu_backend"):
        r = _run(str(CONFIGS_DIR / f"{_CONFIG}.json"), route)
        if r.fell_back:
            pytest.skip(f"{route}: backend fell back to legacy on this host — routes not distinct")
        results[route] = r

    reference = results["legacy"].anchor_l99_sky
    assert reference > 0.0, (
        f"{_CONFIG}: the legacy route published no anchor at all — the anchor buffer is "
        "never being accumulated on the path every other route is compared against"
    )
    assert results["cpu_backend"].anchor_l99_sky == pytest.approx(reference, rel=_HOST_REL_TOL), (
        f"{_CONFIG}: the exit-seam route published {results['cpu_backend'].anchor_l99_sky!r} "
        f"against the legacy route's {reference!r}. These two trace the same rays, so this is "
        "a transport defect in the chunk split, not a sampling difference."
    )


@pytest.mark.slow
def test_device_fused_route_agrees_within_monte_carlo_noise():
    """metal vs legacy: judged against the statistic's own spread, not a fixed ulp.

    The device-fused route generates its rays ON DEVICE from its own RNG stream, so
    it traces a DIFFERENT realization of the same sky. The anchor is a P99 over
    box-summed bins of that realization — an order statistic, which converges far
    more slowly than a mean — so the two routes cannot agree to the bit however
    correct both are, and a fixed tolerance would either be vacuous or a lie.

    The honest bar is the statistic's own seed-to-seed spread, measured here rather
    than assumed. And it is not a weak bar for the defects that matter: a missed
    exit tail, a disagreement about the anchor's geometry, or a plane reset on the
    wrong clock all produce FACTORS (2x, N_batches, or 0), not percents. The
    lens-invariance case below is what pins the device route's exit-tail placement
    exactly, since there the seed is held fixed and the comparison IS exact.
    """
    legacy = []
    for seed in _SPREAD_SEEDS:
        r = _run(str(CONFIGS_DIR / f"{_CONFIG}.json"), "legacy", seed=seed)
        legacy.append(r.anchor_l99_sky)
    assert min(legacy) > 0.0, f"{_CONFIG}: legacy published no anchor at some seed: {legacy!r}"

    spread = max(legacy) / min(legacy) - 1.0
    assert spread < _MAX_USEFUL_SPREAD, (
        f"{_CONFIG}: the anchor's own seed-to-seed spread is {spread:.3%}, past the "
        f"{_MAX_USEFUL_SPREAD:.0%} at which this scene stops being able to tell a correct "
        "device route from a wrong one. Raise ray_num or pick a scene with more structure — "
        "do NOT widen the bound below to accommodate it."
    )

    m = _run(str(CONFIGS_DIR / f"{_CONFIG}.json"), "metal")
    if m.fell_back:
        pytest.skip("metal: backend fell back to legacy on this host — routes not distinct")
    assert m.anchor_l99_sky > 0.0, (
        f"{_CONFIG}: the device-fused route published no anchor at all — its kernel is not "
        "accumulating the anchor plane, or the drain never reaches the consumer"
    )

    centre = sorted(legacy)[len(legacy) // 2]
    # Twice the observed spread: both sides carry their own draw, and `spread` is a
    # 3-sample range rather than a distribution.
    bound = 2.0 * spread
    delta = abs(m.anchor_l99_sky / centre - 1.0)
    assert delta <= bound, (
        f"{_CONFIG}: the device-fused route's anchor is {delta:.3%} off the legacy median "
        f"({m.anchor_l99_sky!r} vs {centre!r}), past the {bound:.3%} that this statistic's own "
        f"seed-to-seed spread ({spread:.3%}) accounts for. Too large to be a different draw of "
        "the same sky — look for a missed exit tail, a divergent anchor geometry, or a plane "
        "reset on the wrong clock."
    )

    # The scene really is the same scene on both routes: a MEAN over the same 2M
    # rays converges fast enough to compare tightly, unlike the P99 above. Without
    # this, a device route simulating something else entirely could sit inside the
    # bound by coincidence.
    assert m.snapshot_intensity == pytest.approx(
        _run(str(CONFIGS_DIR / f"{_CONFIG}.json"), "legacy").snapshot_intensity, rel=1e-3
    ), "the two routes did not even simulate the same scene — the anchor comparison is moot"


@pytest.mark.slow
@pytest.mark.parametrize("route", _ROUTES)
def test_anchor_does_not_move_with_the_lens(route):
    src = CONFIGS_DIR / f"{_CONFIG}.json"
    narrow = _with_lens(src, {"type": "linear", "fov": 20.0}, [512, 512])
    wide = _with_lens(src, {"type": "fisheye_equal_area", "fov": 180.0}, [1024, 1024])

    a = _run(str(narrow), route)
    b = _run(str(wide), route)
    if a.fell_back or b.fell_back:
        pytest.skip(f"{route}: backend fell back to legacy on this host — routes not distinct")

    assert a.anchor_l99_sky > 0.0 and b.anchor_l99_sky > 0.0, (
        f"{route}: one of the two lens variants published no anchor "
        f"({a.anchor_l99_sky!r} / {b.anchor_l99_sky!r})"
    )
    assert a.anchor_l99_sky == pytest.approx(b.anchor_l99_sky, rel=_HOST_REL_TOL), (
        f"{route}: the anchor moved from {a.anchor_l99_sky!r} to {b.anchor_l99_sky!r} when "
        "only the renderer's lens and resolution changed. The anchor is measured on a fixed "
        "full-sky buffer precisely so that it cannot — a renderer field has leaked into it."
    )

    # The control: the quantity the anchor REPLACES really does move across this
    # same pair, so the assertion above is not passing on a scene where every
    # anchor would agree anyway. snapshot_intensity is the per-renderer landed
    # energy per pixel — lens-dependent by construction.
    shift = abs(math.log2(a.snapshot_intensity / b.snapshot_intensity))
    assert shift > 0.2, (
        f"{route}: control failed — the lens-dependent quantity (snapshot_intensity) moved "
        f"only {shift:.3f} stop across these two configs, so this scene cannot distinguish a "
        "lens-independent anchor from a lens-dependent one. Pick a scene with more structure."
    )
