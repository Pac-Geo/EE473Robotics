# Project 1: RX150 Kinematics

**Full instructions:** [profshrestha/ee473-573 · Project 1](https://github.com/profshrestha/ee473-573/blob/main/projects/project-01-rx150-kinematics/README.md)

Check Canvas for deliverables, deadlines, and grading rubric.

This is **your** directory inside your station's repository. Your two teammates have their own
beside it. The code in here is yours: you write your own derivation, your own kinematics, and
your own report.

---

## What is here

```
robot/rx150.py          every number that knows this is an RX150: DH table,
                        joint limits, tool offset, and the Part D hardware
                        interface. The ONLY hardware-aware file.
kinematics/
  transforms.py         frame algebra. Knows nothing about any particular robot.
  fk.py                 fk(), fk_all_frames(), fk_poe()
  ik.py                 ik() and the Solution class
tests/test_roundtrip.py written for you. Run it. It fails until your code works.
scripts/verify_fk.py    harness for the three Part B verifications
REPORT.md               your report. This is the deliverable people read.
figures/                photographs and plots the report points at
```

Every function is a stub that raises `NotImplementedError`. The structure is given so that work
across the class is comparable and so that Project 3 is a swap rather than a rewrite; the
mathematics is yours.

**Keep the `robot/` boundary.** If you put a link length in `kinematics/`, the code works and you
lose the point of the exercise. In Project 3 the arm changes, and that boundary is what decides
whether you replace one file or all of them.

---

## Getting started

Work at your station. Open a terminal and everything is ready: ROS 2 Jazzy, the Interbotix
packages, and the Python environment with NumPy, `modern_robotics`, PyTorch and the rest.

```bash
git clone https://github.com/SSU-ISL/ee473-2026f-p1-station-NN.git
cd ee473-2026f-p1-station-NN/students/palomecnic

pytest -v          # everything fails. That is the starting line.
```

**Run `pytest` from your own directory**, not from the repository root. Three copies of the same
package live side by side, and collecting them all at once confuses the import machinery.

If `pytest` is not found, open a new terminal, or run:

```bash
source /opt/isl-venv/bin/activate
```

Work in this order. Each step is checkable before the next one depends on it.

1. **Part A.** Assign frames by hand, build the DH table, fill in `robot/rx150.py`.
2. `transforms.py`, then `test_fk_at_home_is_finite` and `test_rotation_block_is_a_rotation`
   should pass. If the determinant test fails you have a sign error, and it is far cheaper to
   find now than after IK is written on top of it.
3. **Part B.** `fk()`, then `scripts/verify_fk.py` and all three comparisons.
4. **Part C.** `ik()`, then the round-trip tests.
5. **Part D.** The arm. Fill in the `Arm` class in `robot/rx150.py`.

Write `REPORT.md` as you go, not at the end. Every section of it corresponds to something you
have just done, and the numbers are easiest to record while they are still on your screen.

---

## What you share with your team

The Part D measurement session is done together, and the measurements go in `data/` at the
repository root, not in here. All three of you analyse that one dataset against your own forward
kinematics, and your predictions will differ if your DH tables differ. That comparison is worth
writing about.

Commit as yourself, not from a shared login. Commit history is how the instructor sees who did
what, and it is the only record that exists.

---

## Before you commit

Run `git status` and look at it. Never commit credentials, tokens, or `.env` files. The
`.gitignore` covers the common cases but it does not cover carelessness.

Do not publish this repository or its contents publicly while the course is running. Future
students work the same project. After grades are posted, put it on your own account; it is worth
showing.

---

## Fill this in before you submit

Replace this section with your own notes.

**How to run my code:**

**What I did not finish, and why:**
