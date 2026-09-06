"""Base test case for Lumice E2E tests."""

import shutil
import tempfile
import unittest
from typing import NamedTuple, Sequence, Tuple

from test.e2e.runner import find_lumice_binary, run_lumice, DEFAULT_TIMEOUT


class SharedRender(NamedTuple):
    """One CLI run, executed once for a whole TestCase class.

    Immutable by construction, which is the point: several test methods read the
    same instance, so a field one of them could rewrite in place would be shared
    mutable state between tests -- the classic cost of hoisting setup up a scope.
    A NamedTuple makes that impossible to write by accident rather than leaving it
    to each reader's care.

    The images the run produced are NOT carried here; `output_dir` names the
    directory and each test opens what it needs. Decoding a PNG costs a few
    milliseconds against a render's ~1 s, and an image object is mutable, so
    sharing the cheap half would buy nothing and give up the guarantee above.
    """

    output_dir: str
    returncode: int
    stdout: str
    stderr: str
    args: Tuple[str, ...]


class LumiceTestCase(unittest.TestCase):
    """Base class for Lumice E2E tests.

    Provides:
      - self.lumice_bin: Path to the Lumice executable (skips test if not found).
      - self.output_dir: A fresh temporary directory for test output.
      - self.run_lumice(args, timeout): Convenience method.
      - cls.render_once(config, ...): One render shared by every test in the class.
    """

    def setUp(self):
        try:
            self.lumice_bin = find_lumice_binary()
        except FileNotFoundError as e:
            self.skipTest(str(e))

        self.output_dir = tempfile.mkdtemp(prefix="lumice_e2e_")

    def tearDown(self):
        if hasattr(self, "output_dir") and self.output_dir:
            shutil.rmtree(self.output_dir, ignore_errors=True)

    def run_lumice(self, args, timeout=DEFAULT_TIMEOUT):
        """Run Lumice with the given arguments.

        Returns subprocess.CompletedProcess with stdout/stderr as strings.
        """
        return run_lumice(args, timeout=timeout)

    # --- class-level shared render ------------------------------------------
    #
    # Several classes here ask two or three questions of one rendered frame --
    # "did it exit 0 and produce an image", "do the corner pixels stay black",
    # "does stdout carry the class-signal line" -- and each test method used to
    # buy its own render to ask one of them. The frame is the same either way,
    # so the extra runs bought nothing; and a CLI render costs about a second
    # even for a fixture too small to spend it (main.cpp polls for completion on
    # a one-second tick), so the waste does not shrink with the fixture.
    #
    # This is opt-in, not a hook: a class that wants the sharing says so in its
    # own setUpClass. A base class that silently rendered for every subclass
    # would change what a test does without the test saying it.

    @classmethod
    def _shared_render_cache(cls):
        """This class's own cache -- never a base class's or a sibling's.

        Read through `cls.__dict__` rather than plain attribute lookup: the
        latter would find an ancestor's dict and let two unrelated TestCase
        classes hand each other renders.
        """
        cache = cls.__dict__.get("_shared_renders")
        if cache is None:
            cache = {}
            setattr(cls, "_shared_renders", cache)
        return cache

    @classmethod
    def render_once(
        cls,
        config,
        extra_args: Sequence[str] = (),
        timeout: int = DEFAULT_TIMEOUT,
    ) -> SharedRender:
        """Render `config` once for this class and return the same result to every caller.

        Call it from `setUpClass`. The run happens before any test method, so it
        does not matter which of them runs first, whether the order is shuffled,
        or whether only one of them was selected -- each still sees a render that
        already happened, and none can be made to depend on another having run.

        Repeated calls with the same `(config, extra_args)` return the cached
        result; a different config or a different argument list is a different
        render and gets its own directory. Output goes to a directory owned by
        the class, removed by `tearDownClass`.

        The returncode is returned, not asserted: whether a non-zero exit is a
        failure belongs to the test that asked for the render, and asserting it
        here would report it as a class-level error rather than as that test's
        own failure.
        """
        key = (str(config), tuple(extra_args))
        cache = cls._shared_render_cache()
        if key in cache:
            return cache[key]

        try:
            find_lumice_binary()
        except FileNotFoundError as e:
            # Class-level skip: the same outcome the per-method self.skipTest gave,
            # reported once instead of once per test.
            raise unittest.SkipTest(str(e))

        out_dir = tempfile.mkdtemp(prefix="lumice_e2e_cls_")
        args = ("-f", str(config), "-o", out_dir) + tuple(extra_args)
        result = run_lumice(list(args), timeout=timeout)
        shared = SharedRender(
            output_dir=out_dir,
            returncode=result.returncode,
            stdout=result.stdout,
            stderr=result.stderr,
            args=args,
        )
        cache[key] = shared
        return shared

    @classmethod
    def tearDownClass(cls):
        """Drop whatever `render_once` produced for this class.

        Automatic, unlike the render itself: this only undoes what the class
        already opted into, and the mirror image of `tearDown`'s rmtree. A class
        that never called `render_once` has an empty cache and nothing happens.
        """
        for shared in cls._shared_render_cache().values():
            shutil.rmtree(shared.output_dir, ignore_errors=True)
        cls._shared_render_cache().clear()
        super().tearDownClass()
