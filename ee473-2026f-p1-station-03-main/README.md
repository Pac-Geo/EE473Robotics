# EE 473/573 Project 1, Station 03

**Full instructions:** [profshrestha/ee473-573 · Project 1](https://github.com/profshrestha/ee473-573/blob/main/projects/project-01-rx150-kinematics/README.md)

Check Canvas for deliverables, deadlines, and grading rubric.

Private to this station team and the instructor.

---

## Layout

```
students/amandolil/      individual work. Code, report, figures
students/gonzcarlos/      individual work. Code, report, figures
students/palomecnic/      individual work. Code, report, figures
data/
  partD-measurements.csv   shared: one measurement session, three analyses
requirements.txt             a manifest of what the station provides
```

**The code is individual work.** Each of you fills in your own copy: your own frame
assignment, your own DH table, your own forward and inverse kinematics, your own report.
Comparing three independent derivations against one shared measurement is part of the
exercise, so do not merge your directories or copy between them.

**Part D is done together.** One arm, one measurement session, the three roles rotated.
The numbers go in `data/partD-measurements.csv`, and all three of you analyse that same
file against your own forward kinematics. Your predictions will differ if your tables
differ, and that difference is worth writing about.

---

## Getting started

```bash
cd students/<your-username>
pytest -v          # everything fails. That is the starting line.
```

Run `pytest` from your own directory, not from here. Three copies of the same package
live side by side, and collecting them all at once confuses the import machinery.

The rest of the instructions are in your own directory's README.

---

## Working in one repository

Pull before you push. Commit as yourself, not from a shared login: the commit history is
how the instructor sees who did what, and it is the only record that exists.

Stay in your own directory. A commit that edits a teammate's files is the one thing that
makes this layout go wrong.
