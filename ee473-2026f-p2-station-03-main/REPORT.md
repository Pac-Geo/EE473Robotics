# Project 2 Report: Six-Axis Kinematics of the WidowX AI

**Station:**
**Team:**
**Date:**

---

## About this document

This is the team report, one per team. It is read on GitHub, and should be composed as a
technical document that a reader can follow rather than as a form with its blanks completed.

For an indication of the form to aim at, see
[Forward Kinematics of the UR5e: DH and PoE](https://www.roboticsunveiled.com/forward-kinematics-ur5e-dh-poe/).
That treatment states the frame assignment, presents the table, gives the equations, shows the
implementing code, and then presents the **actual output** establishing that the methods agree.
Every claim is supported by evidence the reader can verify.

Match its rigour rather than its length.

Delete this section before submission.

---

## 1. Frame assignment

Photograph or scan your hand-drawn frame assignment and put it here.

```
![Frame assignment](figures/frames.jpg)
```

The base frame, the tool frame, and the direction of every z axis are prescribed in §3.1 of the
instructions. This section concerns what remained open: the position of each origin along its
axis, and the direction of x wherever the common normal did not determine it.

State the Denavit-Hartenberg convention employed. The instructions specify the classical
convention; state it regardless, as a reader cannot determine it from the table alone.

## 2. DH table

| i | θᵢ | dᵢ | aᵢ | αᵢ |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |
| 4 |  |  |  |  |
| 5 |  |  |  |  |
| 6 |  |  |  |  |

Lengths in metres, angles in radians. Identify the joint variable in each row.

Then account for the tool. Trossen's chain proceeds from the base through the six joints to
`Flange` and thence to `Tool`, so two fixed transformations follow joint 6, and the translation to
the tool point is 156.06 mm. State the transformation carrying frame 6 to `Tool`.

## 3. Forward kinematics

Give the link transformation employed, then its composition to the tool frame.

```math
H = \begin{bmatrix} R & d \\ 0 & 1 \end{bmatrix}
```

Then present both verifications from §3.3: the URDF chain and the product of exponentials.
Include the actual output of `scripts/verify_fk.py` rather than a description of it.

Where either disagrees, that disagreement belongs here, together with its resolution.

## 4. Comparison against the manufacturer's table

Put Trossen's published table beside your own.

| i | Ours: θ, d, a, α | Trossen: θ, d, a, α | Agree? | Cause of the difference |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |
| 4 |  |  |  |  |
| 5 |  |  |  |  |
| 6 |  |  |  |  |

For every row that disagrees, give the cause. Constant offsets in the θ column are expected and
arise from a specific mechanism; identify it in each case.

Report the numerical comparison as well: the tool pose computed from Trossen's parameters
against the pose computed from your own. Tables differing row by row may nonetheless agree at
every pose.

A derived table exhibiting no θ offsets is itself a result, and warrants explanation.

## 5. Inverse kinematics by decoupling

State the decoupling argument: why the wrist centre depends only upon the first three joints,
and how the remaining three are recovered once those are known.

Give the equations, the number of solution branches expected, and the origin of each.

## 6. Round-trip test

Include the `pytest -v` output.

A passing round trip establishes that every solution returned is valid. It does not establish that
the complete set was found, nor that the manipulator can physically adopt them. Address those two
requirements separately.

## 7. Part C: hardware measurements

| Pose | Target (x, y, z, roll, pitch, yaw) | Branch chosen | Predicted | Measured | Error (mm) |
|---|---|---|---|---|---|
| 1 |  |  |  |  |  |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |
| 4 |  |  |  |  |  |
| 5 |  |  |  |  |  |

State the orientation convention under which these targets were constructed, and the measurement
procedure, before presenting the figures. The errors are interpretable only once the reader knows
what was measured, between which points, and under which convention.

Orientation is evaluated against `driver_ee_pose()` rather than measured. Report that comparison
separately from the position errors above.

Record the configuration the servos actually attained, not only the configuration commanded.

## 8. Comparison against Project 1

The method is the same and the manipulator differs. Determine whether the error is larger or
smaller than that observed on the RX150, and account for the difference.

Argue from the collected data. An appeal to build quality is not an argument; a mass, a lever arm,
or a measured servo offset is.

## 9. Discussion

What would be done differently, what remains uncertain, and what did not work, with the reason.

Candidly stated uncertainty is of greater value here than unsupported confidence.

## 10. Who did what

| Member | Contribution |
|---|---|
|  |  |
|  |  |
|  |  |

This section is read against the commit history.

---

<details>
<summary><strong>Getting equations to render on GitHub</strong></summary>

GitHub's math support has several pitfalls. The rules below address the common ones.

**Use fenced math blocks.** This form always works:

````
```math
H = \begin{bmatrix} R & d \\ 0 & 1 \end{bmatrix}
```
````

**Inline math is where this goes wrong.** GitHub applies Markdown emphasis *before* it resolves
math, so two underscores in the same paragraph pair into italics and carry the subscripts away
with them. Writing `$z_{i-1}$` and `$o_{i-1}$` in one paragraph renders as neither: the underscores
become an `<em>` and the dollar signs are left sitting in the text.

When an expression contains an underscore, delimit it with backticks inside the dollars:

````
$`z_{i-1}`$        renders correctly
$z_{i-1}$          breaks if another underscore appears nearby
````

Plain symbols with no underscore, like `$\theta$` or `$\alpha$`, are safe either way.

**A line break `\\` inside inline math collapses**, so never put a matrix there.

**Avoid `\,` and `\;`.** They are thin spaces and some renderers show them as a literal comma or
semicolon. Use a plain space.

**`\operatorname` is not supported.** Use `\mathrm{tr}` rather than `\operatorname{tr}`.

**Verify in the browser.** Push the file, open it on github.com, and inspect the result. The
preview in an editor uses a different renderer, as does the GitHub mobile application, which does
not render mathematics at all.

</details>
