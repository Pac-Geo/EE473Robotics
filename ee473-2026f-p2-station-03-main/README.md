# Project 2: Six-Axis Kinematics of the WidowX AI

**Full instructions:** [profshrestha/ee473-573 · Project 2](https://github.com/profshrestha/ee473-573/blob/main/projects/project-02-widowx-6dof/README.md)

Consult Canvas for deliverables, deadlines, and the grading rubric.

This repository belongs to one team and holds one submission: a single package and a single
report, with all members working in it directly.

---

## Contents

```
robot/widowx.py         the quantities particular to the WidowX AI: the
                        Denavit-Hartenberg table, joint limits, tool offset,
                        and the Part C hardware interface. The only
                        hardware-aware module.
kinematics/
  transforms.py         frame algebra, independent of any manipulator
  fk.py                 fk(), fk_all_frames(), fk_poe()
  ik.py                 wrist_centre(), ik(), and the Solution class
tests/test_roundtrip.py supplied; run it with pytest
scripts/verify_fk.py    harness for the verifications in §3.3
REPORT.md               the team report, and the document assessed
data/                   the Part C measurement session
figures/                photographs and plots referenced by the report
```

Every function is a stub raising `NotImplementedError`. Derive and implement the mathematics for
this manipulator.

The layout follows the one used in Project 1 and should already be familiar.

**Observe the `robot/` boundary.** Every quantity particular to this manipulator belongs in
`robot/widowx.py`, and the modules within `kinematics/` accept numerical arguments and return
numerical results. Project 3 extends this library to velocity control, and that extension is
substantially easier where the boundary has held.

---

## Getting started

Each station provides the complete environment in any new terminal.

```bash
git clone https://github.com/SSU-ISL/ee473-2026f-p2-station-NN.git
cd ee473-2026f-p2-station-NN

pytest -v
```

Every test fails initially, which is the expected starting condition.

Invoke `pytest` from the repository root, where the package resides.

Should `pytest` not be found, open a new terminal or activate the environment:

```bash
source /opt/isl-venv/bin/activate
```

The recommended order of work, each step verifiable before the next depends upon it:

1. **Part A.** Assign frames, derive the six-row Denavit-Hartenberg table, and complete
   `robot/widowx.py`.
2. `transforms.py`, after which `test_fk_at_home_is_finite` and `test_rotation_block_is_a_rotation`
   should pass. A failing determinant test indicates a sign error, which is considerably cheaper
   to locate at this stage than after the inverse kinematics has been built upon it.
3. `fk()`, followed by the verifications in `scripts/verify_fk.py`.
4. **Part A, comparison.** Compare the derived table against Trossen's published parameters and
   account for each disagreement.
5. **Part B.** `ik()` by wrist decoupling, followed by the round-trip tests.
6. **Part C.** The hardware session.

Compose `REPORT.md` as the work proceeds. Each section corresponds to a result already obtained,
and the figures are most easily recorded while they remain on screen.

---

## The manipulator

The WidowX AI is addressed directly over Ethernet by the `trossen_arm` Python driver. The
controller resides on the laboratory network and is commanded from Python, in contrast to the ROS
stack used for the RX150 in Project 1.

> **The SDK version must match the controller firmware.** The controllers in this laboratory run
> **v1.10.0**. A plain `pip install trossen-arm` retrieves a later release, which fails at
> `configure()` with a `LogicError`. The required version is pinned in `requirements.txt`; confirm
> it before investigating any other cause of that error.

This manipulator is heavier, faster, and of substantially greater value than the RX150. Review the
safety note in Part C of the instructions before connecting to it, and remain within reach of the
power disconnect when commanding a pose for the first time.

---

## Team practice

Each member commits under their own account. For a team submission the commit history is the only
record of individual contribution, and the closing section of the report is read against it.

---

## Before committing

Review `git status` before each commit. Credentials, tokens, and `.env` files are never committed;
`.gitignore` covers the common cases.

Log out of Codex and the GitHub CLI upon leaving a station, which is shared.

This repository and its contents remain private for the duration of the course, as subsequent
cohorts undertake the same project.

---

## To be completed before submission

Replace this section with the team's own notes.

**Running the code:**

**Work left incomplete, and the reason:**
