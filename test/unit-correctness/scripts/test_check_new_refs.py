"""Regression net for `scripts/check_new_refs.py`.

Its sibling gate `scripts/check_policies.py` ships without tests, and that is
fine: every one of its checks asks "is this symbol present in this file?", which
is legible by reading. This one is a regex battery driving a unified-diff
parser, and it is neither — five distinct defects were found on the same small
surface in one day, one of them living *inside* the fix for another. Careful
reading demonstrably does not converge here.

What makes that worth a test file rather than more care: every defect failed
*silently*. A broken parser does not crash, it attributes lines to a path that
does not exist and reports success. A regex that lost its teeth reports success.
The gate stays green while enforcing nothing — exactly the failure the gate
itself exists to prevent, so it is the one failure it cannot be trusted to
reveal.

Each test below therefore states the mechanism it pins and, where the mechanism
is a trade rather than an obvious property, why the opposite choice is wrong.
Fixture payloads use invented identifiers (nine-hundred-series numbers,
`example`-style slugs); the checker only reads prose, so these string literals
are out of its scope by construction, but they are kept fictional anyway so that
nothing here reads as a citation of real work.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_new_refs  # noqa: E402
import check_policies  # noqa: E402


class Repo:
    """A throwaway git repo the checker is pointed at via its REPO_ROOT global."""

    def __init__(self, root: Path):
        self.root = root

    def _git(self, *args: str) -> str:
        return subprocess.run(
            ["git", *args], cwd=self.root, capture_output=True, check=True, text=True
        ).stdout

    def write(self, rel: str, text: str) -> None:
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")

    def mv(self, src: str, dst: str) -> None:
        (self.root / dst).parent.mkdir(parents=True, exist_ok=True)
        self._git("mv", src, dst)

    def cp(self, src: str, dst: str) -> None:
        (self.root / dst).parent.mkdir(parents=True, exist_ok=True)
        (self.root / dst).write_bytes((self.root / src).read_bytes())

    def commit(self, msg: str = "wip") -> str:
        self._git("add", "-A")
        self._git("commit", "-q", "-m", msg)
        return self._git("rev-parse", "HEAD").strip()

    def stage(self) -> None:
        self._git("add", "-A")

    def added(self) -> dict[str, set[int]]:
        self.stage()
        return check_new_refs.added_lines(["--cached"])

    def violations(self) -> list[check_new_refs.Violation]:
        """Run the gate over the staged diff, as the pre-commit hook does."""
        self.stage()
        return check_new_refs.check(["--cached"], "")[0]

    def hits(self) -> list[tuple[str, int]]:
        return sorted(
            (str(v.path.relative_to(self.root)), v.line) for v in self.violations()
        )


@pytest.fixture
def repo(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Repo:
    r = Repo(tmp_path)
    r._git("init", "-q", "-b", "main")
    r._git("config", "user.email", "t@example.com")
    r._git("config", "user.name", "T")
    # REPO_ROOT has to be rebound in BOTH modules, and the second one is easy to
    # miss: the git helpers and the blob reader close over the name imported into
    # check_new_refs, but Violation.render() resolves it in the module where
    # Violation is defined. Patch only the first and everything passes until a
    # test reaches the renderer, which is the one path the gate takes when it has
    # something to say.
    monkeypatch.setattr(check_new_refs, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_policies, "REPO_ROOT", tmp_path)
    r.write("seed.txt", "seed\n")
    r.commit("seed")
    return r


# --- the diff is scoped to added lines, and renames do not forge additions ----


def test_pure_rename_adds_nothing(repo: Repo) -> None:
    """Moving a file must not resurface its history as "new" prose.

    Without rename detection a move is reported as a whole delete plus a whole
    add, so every pre-existing line of the moved file arrives as added and the
    move is blocked until unrelated old prose is cleaned up. That is precisely
    the burden this gate is scoped to avoid: the rule has never held
    historically, and only newly written prose is in scope.
    """
    body = "# Doc\n\nSee the working notes under scratchpad/example-name/ for detail.\n"
    repo.write("doc/old.md", body)
    repo.commit()

    repo.mv("doc/old.md", "doc/new.md")

    assert repo.added() == {}
    assert repo.hits() == []


def test_rename_with_edit_reports_only_the_rewritten_line(repo: Repo) -> None:
    """A move-plus-edit is in scope for what it rewrites, and nothing else.

    Rename detection is a *similarity* judgement, not a record of the `git mv`,
    so this guarantee holds only while the moved file still resembles its old
    self. The body here is padded to forty-odd lines on purpose: that is the
    shape of the files the guarantee has to cover — a real doc carrying enough
    pre-existing prose to matter — and at that length rewriting one line leaves
    it ~95% similar, far above the threshold. The companion test below pins what
    happens on the other side of it.
    """
    body = "\n".join(f"paragraph {i} of the guide body." for i in range(40))
    repo.write(
        "doc/old.md",
        f"# Doc\n\nold line citing scratchpad/example-name/ here.\n{body}\n",
    )
    repo.commit()

    repo.mv("doc/old.md", "doc/new.md")
    repo.write(
        "doc/new.md",
        f"# Doc\n\nrewritten line citing scratchpad/other-example/ here.\n{body}\n",
    )

    assert repo.added() == {"doc/new.md": {3}}
    assert repo.hits() == [("doc/new.md", 3)]


def test_small_file_rewritten_while_moved_is_scanned_whole(repo: Repo) -> None:
    """Below the similarity threshold a move reads as a plain add — by design.

    This is the exact price of the test above, and it is deliberately pinned
    rather than left to be discovered: a short file whose content is largely
    replaced in the same commit no longer resembles its predecessor, so the move
    is reported as a delete plus a whole-file add and every line — including the
    untouched fourth one — is scanned as new.

    That is the right outcome, not a leak in the scoping. The guarantee this gate
    owes is "do not bill anyone for prose they did not write", and someone who
    rewrote most of a small file in the act of moving it *is* writing this prose;
    asking them to stand behind the few lines they carried along costs them
    almost nothing. The guarantee's real subject is the opposite case — the
    long-lived file with a large body of pre-existing citations — and no
    realistic edit to one of those drops it below the threshold.
    """
    repo.write(
        "doc/old.md",
        "# Note\n\nold line citing scratchpad/example-name/ here.\n"
        "tail cites scratchpad/third-example/ too.\n",
    )
    repo.commit()

    repo.mv("doc/old.md", "doc/new.md")
    repo.write(
        "doc/new.md",
        "# Note\n\nan entirely rewritten paragraph with substantially different "
        "wording and considerably greater length than whatever stood in this "
        "place before it was replaced.\n"
        "tail cites scratchpad/third-example/ too.\n",
    )

    assert repo.added() == {"doc/new.md": {1, 2, 3, 4}}
    # The replacement paragraph cites nothing, so the *only* violation reported
    # is the fourth line — which this commit never touched. Nothing but a
    # whole-file scan can produce that hit.
    assert repo.hits() == [("doc/new.md", 4)]


def test_copy_is_scanned_whole_unlike_a_rename(repo: Repo) -> None:
    """A copied file is in scope for all of it, deliberately — not like a move.

    The two operations look adjacent and are opposites here. A rename relocates
    content its author never wrote, so billing them for it is the exact burden
    this gate is scoped away from. A copy leaves the source in place and brings a
    second path into existence: whoever did that chose to put this prose at this
    path, and the fourth line below is theirs to stand behind even though they
    typed none of it.

    The mechanism is that only rename detection is enabled, so git reports the
    copy as a plain whole-file add. Pinning the behaviour rather than the flags
    keeps this honest in both directions: turning copy detection on would make
    the new path either filtered away or hunk-less, and either way this goes red
    rather than quietly conceding a protection nobody decided to grant.
    """
    repo.write(
        "doc/source.md",
        "# Doc\n\nintro paragraph.\ncites scratchpad/example-name/ for detail.\n",
    )
    repo.commit()

    repo.cp("doc/source.md", "doc/copy.md")

    assert repo.added() == {"doc/copy.md": {1, 2, 3, 4}}
    assert repo.hits() == [("doc/copy.md", 4)]


# --- header parsing: the prefixes are ambiguous, position is not --------------


def test_added_line_shaped_like_a_new_file_header(repo: Repo) -> None:
    """Payload beginning `++ ` renders as `+++ ` and must not reroute the hunk.

    Read as a header it would repoint the rest of the hunk at a path that does
    not exist, whose blob reads back empty — so the real violation two lines
    later vanishes and the gate reports success.
    """
    repo.write("doc/patch.md", "# Patch\n")
    repo.commit()

    repo.write(
        "doc/patch.md",
        "# Patch\n"
        "++ b/phantom.md\n"
        "a quoted diff, then a real citation of scratchpad/example-name/ here.\n",
    )

    assert repo.added() == {"doc/patch.md": {2, 3}}
    assert repo.hits() == [("doc/patch.md", 3)]


def test_deleted_line_shaped_like_an_old_file_header(repo: Repo) -> None:
    """Payload beginning `-- ` renders as `--- ` and must not arm the guard.

    The mirror image of the case above, and the reason the guard is positional:
    requiring a `--- ` partner in front of `+++ ` is not enough, because the
    partner itself can be forged by a deleted line. Here the deletion arms the
    guard and the very next added line — which carries the violation — is eaten
    as the header it appears to be.
    """
    repo.write("doc/patch.md", "# Patch\n-- old bullet\ntail\n")
    repo.commit()

    repo.write(
        "doc/patch.md",
        "# Patch\n++ new bullet, see scratchpad/example-name/ for detail\ntail\n",
    )

    assert repo.added() == {"doc/patch.md": {2}}
    assert repo.hits() == [("doc/patch.md", 2)]


def test_multiple_files_keep_their_own_hunks(repo: Repo) -> None:
    """Path state must reset per file section, not leak across the boundary."""
    repo.write("doc/a.md", "# A\n")
    repo.write("doc/b.md", "# B\n")
    repo.commit()

    repo.write("doc/a.md", "# A\ncites scratchpad/example-name/ here.\n")
    repo.write("doc/b.md", "# B\ncites explore-example-name here.\n")

    assert repo.hits() == [("doc/a.md", 2), ("doc/b.md", 2)]


# --- the trailing-boundary asymmetry -----------------------------------------
#
# The two identifier patterns look alike and are deliberately not alike: one
# ends in a word boundary and the other must not. Aligning them "for tidiness"
# in either direction silently costs real detections in one direction or fires
# on the rule's own wording in the other. The three tests below pin both signs,
# because a comment alone has already proven too weak to hold this.


def test_review_round_filename_needs_its_trailing_boundary(repo: Repo) -> None:
    """The placeholder wording for the round-note filename must stay writable.

    Documenting this rule means naming the filename pattern with a placeholder
    digit. An unguarded trailing `\\d+` matches that placeholder through its
    leading zero, so the gate would fire on any file that explains it — this
    very repo's policy prose included.
    """
    repo.write("doc/rule.md", "# Rule\n")
    repo.commit()

    repo.write(
        "doc/rule.md",
        "# Rule\nNotes are named with a placeholder digit, e.g. code-review-0N.\n",
    )

    assert repo.hits() == []


def test_letter_suffixed_identifier_must_still_be_caught(repo: Repo) -> None:
    """The identifier pattern's number branch must NOT end in a word boundary.

    Phase labels suffixed with a letter are a real citation form, and a
    trailing boundary drops them entirely: the number branch cannot end before
    the letter, and the slug branch needs a hyphen that is not there. This is
    the direction that "aligning the patterns" breaks, and it breaks it
    silently — the gate keeps passing, having stopped looking.
    """
    repo.write("doc/note.md", "# Note\n")
    repo.commit()

    repo.write("doc/note.md", "# Note\nHost-only until scrum-9d lands.\n")

    assert repo.hits() == [("doc/note.md", 2)]


def test_dotted_identifier_is_caught(repo: Repo) -> None:
    repo.write("doc/note.md", "# Note\n")
    repo.commit()

    repo.write("doc/note.md", "# Note\nMeasured under task-901.2 conditions.\n")

    assert repo.hits() == [("doc/note.md", 2)]


# --- what must be caught ------------------------------------------------------


@pytest.mark.parametrize(
    "prose",
    [
        "Decided in task-901 after discussion.",
        "See explore-example-name for the numbers.",
        "Superseded by scrum-example-topic-name.",
        "The chore-example-cleanup pass removed it.",
        "Rationale lives in plan.md next door.",
        "Tracked in progress.md as it lands.",
        "Written up in SUMMARY.md at close.",
        "Deferred; see backlog.md.",
        "Full detail in scratchpad/example-name/insights.md.",
        "A relative hop like ../scratchpad/example-name/ counts too.",
        "Answers round-97 feedback.",
    ],
)
def test_citations_are_caught(repo: Repo, prose: str) -> None:
    repo.write("doc/note.md", "# Note\n")
    repo.commit()
    repo.write("doc/note.md", f"# Note\n{prose}\n")

    assert repo.hits() == [("doc/note.md", 2)]


# --- what must not be caught --------------------------------------------------


@pytest.mark.parametrize(
    "prose",
    [
        # Permanent anchors: these survive in git forever and are the whole
        # point of the rule's escape route, so they must never trip it.
        "Fixed in PR#199, see the discussion there.",
        "Introduced by commit a961fb91 on main.",
        "Handled at simulator.cpp:412.",
        "The ExposureScale() helper owns this.",
        # A hex byte contains a letter-plus-digits run. An earlier draft of the
        # identifier pattern matched that shape and fired here; the shape is
        # legitimate program prose and must stay silent.
        "Masks the high nibble of 0xE6 before the compare.",
        "Bit pattern 0xDEADBEEF marks an unwritten slot.",
        # A bare single word after a known prefix is an English compound or a
        # command name far more often than a citation, so it is deliberately
        # out of the pattern.
        "This is task-specific behaviour.",
        "Values are explore-only knobs.",
        # A longer word merely ending in the directory name is not a path.
        "The oldscratchpad/ directory predates this.",
    ],
)
def test_permanent_anchors_and_lookalikes_are_not_caught(repo: Repo, prose: str) -> None:
    repo.write("doc/note.md", "# Note\n")
    repo.commit()
    repo.write("doc/note.md", f"# Note\n{prose}\n")

    assert repo.hits() == []


# --- scope: prose only --------------------------------------------------------


def test_python_string_literals_are_out_of_scope_but_prose_is_not(repo: Repo) -> None:
    """Identifiers are program data; only what a human reads as prose is scanned.

    Test-case names and fixture payloads legitimately carry citation-shaped
    tokens — this very file is made of them — so scanning string literals would
    make the gate unusable for the code that tests it.
    """
    repo.write("t.py", "x = 1\n")
    repo.commit()

    repo.write(
        "t.py",
        '"""Module prose mentioning task-901.2 is in scope."""\n'
        'NAME = "task-902.3"  # a literal is not\n'
        "# but a comment citing task-903.4 is\n",
    )

    assert repo.hits() == [("t.py", 1), ("t.py", 3)]


def test_unknown_file_types_are_not_scanned(repo: Repo) -> None:
    """Reading an unknown type whole-file would fire on identifiers and payloads."""
    repo.write("data.json", "{}\n")
    repo.commit()
    repo.write("data.json", '{"note": "scratchpad/example-name/plan.md"}\n')

    assert repo.hits() == []


# --- scope: the exemption criterion ------------------------------------------


def test_agent_instruction_entry_points_are_exempt(repo: Repo) -> None:
    """Exempt only where the file's SUBJECT is the working-notes system itself."""
    repo.write("AGENTS.md", "# Agents\n")
    repo.commit()
    repo.write("AGENTS.md", "# Agents\nNotes live in scratchpad/<name>/plan.md.\n")

    assert repo.hits() == []


def test_citing_the_system_does_not_earn_an_exemption(repo: Repo) -> None:
    """The exemption is about subject, not about mentioning it.

    A file that cites the notes tree is the thing being checked; if citing it
    were grounds to be skipped, the gate would exempt exactly its own targets.
    """
    repo.write("doc/guide.md", "# Guide\n")
    repo.commit()
    repo.write("doc/guide.md", "# Guide\nBackground sits in scratchpad/example-name/.\n")

    assert repo.hits() == [("doc/guide.md", 2)]


def test_nested_ignore_files_stay_in_scope(repo: Repo) -> None:
    """Only the root ignore file declares the rule; a nested one can only cite."""
    repo.write("sub/.gitignore", "*.tmp\n")
    repo.commit()
    repo.write("sub/.gitignore", "*.tmp\n# mirrors scratchpad/example-name/ layout\n")

    assert repo.hits() == [("sub/.gitignore", 2)]


# --- the command-line contract ------------------------------------------------


@pytest.mark.parametrize("argv", [[], ["--staged"]])
def test_staged_is_the_default_and_reads_the_index(
    repo: Repo, monkeypatch: pytest.MonkeyPatch, argv: list[str]
) -> None:
    """No arguments and `--staged` are one path, and the path reads the index.

    Worth pinning because the coupling is invisible in the source: `--staged` is
    declared and then never inspected, the no-range branch simply falls through
    to the index, and the two agree only by construction. A tidying pass that
    "wires up the unused flag", or drops the default the flag is documenting,
    breaks one spelling while the other keeps working — and on a clean tree both
    still exit 0, so nothing complains.

    The comparison is the index against HEAD — what is about to be committed —
    and not the worktree against the index. Those two coincide in most fixtures,
    which is what makes the distinction easy to lose; here the file is staged and
    left alone, so the worktree-vs-index diff is empty and only the intended
    comparison can still find the violation.
    """
    repo.write("doc/note.md", "# Note\n")
    repo.commit()
    repo.write("doc/note.md", "# Note\nBackground sits in scratchpad/example-name/.\n")
    repo.stage()

    monkeypatch.setattr(sys, "argv", ["check_new_refs.py", *argv])

    assert check_new_refs.main() == 1


@pytest.mark.parametrize("argv", [[], ["--staged"]])
def test_unstaged_edits_are_not_the_gates_business(
    repo: Repo, monkeypatch: pytest.MonkeyPatch, argv: list[str]
) -> None:
    """Prose that exists only in the worktree is not being committed yet.

    The other half of the index contract: a draft on disk must not block the
    commit of something else, or the hook becomes a reason to stop running it.
    """
    repo.write("doc/note.md", "# Note\n")
    repo.commit()
    repo.write("doc/note.md", "# Note\nBackground sits in scratchpad/example-name/.\n")
    # Deliberately left unstaged.

    monkeypatch.setattr(sys, "argv", ["check_new_refs.py", *argv])

    assert check_new_refs.main() == 0
