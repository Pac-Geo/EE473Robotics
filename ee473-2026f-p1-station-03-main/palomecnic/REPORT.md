
# Project 1 Report: RX150 Kinematics

**Station:3**
**Team: Geovani Palomec, Lee Lamandoli,Carlos Gonzales**
**Date:10/12/26**

---

## Abstract

This project investigates the forward and inverse kinematics of the ReactorX-150 (RX150), a five-degree-of-freedom robotic manipulator. Coordinate frames are assigned to each joint using the Denavit–Hartenberg (DH) convention to develop a mathematical model describing the position and orientation of the end effector. Forward kinematics is implemented in Python using homogeneous transformation matrices and evaluated against independent calculations using the robot's URDF model, ROS 2/RViz, and the Product of Exponentials (PoE) method. Inverse kinematics is developed to determine joint configurations for specified end-effector positions and orientations while accounting for multiple solutions and joint limits. Finally, experimental measurements from the physical RX150 are used to evaluate the accuracy of the kinematic model and identify differences between theoretical predictions and actual robot behavior.

---

## 1. Frame assignment

Photograph or scan your hand-drawn frame assignment and put it here.
<img width="921" height="661" alt="Screenshot 2026-10-15 at 8 10 39 PM" src="https://github.com/user-attachments/assets/f531f1a7-e281-4bed-b7c0-3b24b84138b3" />

To model the RX150, I needed to establish a coordinate frame for each of its five joints. I used the Denavit-Hartenberg (DH) convention method, starting at the base_link, which is located at the mounting surface of the robot. This frame serves as the reference for calculating the position and orientation of the gripper, with the x-axis pointing forward, the y-axis pointing left, and the z-axis pointing upward.

One important details that had to be accounted for was the positioning of the elbow relative to the shoulder of the arm. According to the URDF, the elbow joint has an offset of 0.05 m along the positive x-axis and an offset of 0.15 m along the positive z-axis of the shoulder frame when the joints are at the zero position. This gives a diagonal distance of approximately 0.158114 m, with an angle of about 1.249 rad ~ 71.57°. Because of this geometry, the DH table requires constant angular offsets for these joints.
<img width="1006" height="812" alt="Captura de pantalla 2026-10-16 a la(s) 12 53 49 a m" src="https://github.com/user-attachments/assets/29732c2c-dd71-48bf-946d-c61264d9e20b" />



## 2. DH table
| i | Joint | θᵢ (rad) | dᵢ (m) | aᵢ (m) | αᵢ (rad) |
|---|---|---|---|---|---|
| 1 | Waist | **θ₁** | 0.10457 | 0 | −π/2 |
| 2 | Shoulder | **θ₂** −  β  | 0 | 0.158114 | 0 |
| 3 | Elbow | **θ₃** +  β  | 0 | 0.150 | 0 |
| 4 | Wrist angle | **θ₄** − π/2 | 0 | 0 | −π/2 |
| 5 | Wrist rotate | **θ₅** | 0.065 | 0 | 0 |

Lengths in metres, angles in radians. 

The link connecting the shoulder to the elbow has a horizontal distance of 0.05 m and a vertical distance of 0.15 m. Using the Pythagorean theorem:

```math
a_2 = \sqrt{0.05^2+0.15^2}=0.158114\text{ m}
```
Thus the diagonal link's angle offset:

```math
\beta = \mathrm{atan2}(0.15,0.05)\approx1.249046\text{ rad}
```
This offset is subtracted from the shoulder angle and added to the elbow angle in the DH table. 

The URDF puts four fixed joints after the last revolute one, adding 0.093575 m in total. Since the four fixed joints beyond the rotating wrist extend the tool, the last DH, end effector, frame does not directly coincide with ee_gripper_link. This distance needs to be included through a separate transformation to calculate the position of the actual gripper frame. The fourth joint also includes a constant offset of −π/2 to account for its assigned coordinate-frame orientation.
```math
0.065+0.093575=0.158575\text{ m}
```

The transformation from the fifth DH frame to the tool frame is:

```math
{}^5T_{\mathrm{tool}}=
\begin{bmatrix}
0&0&1&0\\
0&-1&0&0\\
1&0&0&0.093575\\
0&0&0&1
\end{bmatrix}
```
This accounts for the additional distance and orientation difference between the final DH frame and the tool frame. Using the DH parameters from Section 2, a homogeneous transformation matrix for each joint was calculated then multiplied together to find the final pose relative to base_link.

## 3. Forward kinematics
Using forward kinematics we can determine the position and orientation of the RX150's gripper from its five joint angles.

The following is a general case matrix for calculating each joint transformation:
```math
A_i =
\begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
```

Multiplying the five matrices gives the transformation of the last DH frame, but due to the extended distance this is not the actual gripper frame, for that the Tool Frame Offset is used.

The complete forward kinematics transformation is therefore:
```math
{}^0T_{\mathrm{tool}} = A_1 A_2 A_3 A_4 A_5 {}^5T_{\mathrm{tool}}
```
The code that implements this is shown below. (A short excerpt)
The main calculation is:
```python
H = np.eye(4)
for theta, params in zip(thetas, rx150.DH_PARAMS):
    theta_offset, d, a, alpha = params
    H = H @ dh_transform(theta + theta_offset, d, a, alpha)
H = H @ rx150.TOOL_OFFSET
return H
```

Home Configuration
At the home configuration, all five joint angles are zero. Using the URDF dimensions, the resulting transformation is:

```math
{}^0T_{\mathrm{tool}} =
\begin{bmatrix}
1 & 0 & 0 & 0.358575 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.254570 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

The DH table describes the five revolute joints, but the actual end-effector frame, `ee_gripper_link`, is located farther along the arm. According to the URDF information, four fixed joints after the wrist contribute an additional distance: 0.358575 m forward and 0.254570 m above the base, with no displacement along the y-axis.

General Configuration
A general configuration: 
For:
```math
\theta = [0.3,\ -0.2,\ 0.5,\ 0.1,\ 1.2]
```

```math
{}^0T_{\mathrm{tool}} \approx
\begin{bmatrix}
0.87992 & 0.23966 & 0.41024 & 0.29478 \\
0.27219 & 0.45343 & -0.84871 & 0.09119 \\
-0.38942 & 0.85846 & 0.33375 & 0.15543 \\
0 & 0 & 0 & 1
\end{bmatrix}
```
Here, the gripper is about 0.29478 m forward, 0.09119 m to the left, and 0.15543 m above the base.
These calculations give the predicted position and orientation of the gripper for any set of joint angles.

## 4. Verification 1: RViz

For the first verification, we used ROS 2 and RViz to compare the RX150's position results with the calculated forward kinematics. RViz displays the robot using its URDF model, while the joint angles determine how each link is positioned relative to base_link.

Our group tested four configurations: the home position, a waist rotation, a folded-arm position, and a general position.

FK vs. RVi
| Configuration | Joint Angles (rad) | FK Position (mm) | RViz Position (mm) |
|---|---|---|---|
| Home | (0, 0, 0, 0, 0) | (358.575, 0, 254.570) | (359, 0, 255) |
| Waist Rotation | (0.493, 0, 0, 0, 0) | (315.875, 169.703, 254.570) | (316, 170, 255) |
| Folded Arm | (0, -0.4, 0.8, -0.4, 0) | (284.374, 0, 203.787) | (284, 0, 204) |
| General Position | (0.3, -0.2, 0.5, 0.1, 1.2) | (294.779, 91.186, 155.433) | (295, 91, 155) |

The results were pretty consistent across our configurations. The small differences shown come from rounding the RViz values to the nearest mm. Overall the tests showed that our forward kinematics predictions were consistent with the positions reported by the RX150 model in RViz.

## 5. Verification 2: URDF chain

For the second verification, I compared the forward kinematics with the RX150's URDF model. Rather than using DH parameters, this method uses the robot's physical joint structure and multiplies the transformations defined in the URDF to calculate the position and orientation of ee_gripper_link.
The RX150 has five revolution joints and four fixed joints that connect the rotating wrist to the gripper.

The complete transformation is calculated as:
```math
{}^0T_{\mathrm{tool}} =
T_1R_1T_2R_2T_3R_3T_4R_4T_5R_5T_6T_7T_8T_9
```
Here, the first five transformations are the joint rotations, and the remaining four represent the fixed connections to the gripper.
I used the from_urdf_chain(thetas) function in scripts/verify_fk.py to perform this calculation and compare it with the DH-based fk(thetas) function.

FK vs. URDF
| Configuration | FK Position (mm) | URDF Position (mm) | Difference |
|---|---|---|---|
| Home | (358.575, 0, 254.570) | (358.575, 0, 254.570) | 0.0000 mm |
| Waist Rotation | (314.679, 171.910, 254.570) | (314.679, 171.910, 254.570) | 0.0000 mm |
| Folded Arm | (284.374, 0, 203.787) | (284.374, 0, 203.787) | 0.0000 mm |
| General Position | (294.779, 91.186, 155.433) | (294.779, 91.186, 155.433) | 0.0000 mm |

Note: The waist rotation used in this verification is 0.5 rad, rather than the 0.493 rad used during the RViz comparison

The position differences were calculated using the Euclidean distance between the two:
```math
e_p = \sqrt{(x_{DH}-x_{URDF})^2+(y_{DH}-y_{URDF})^2+(z_{DH}-z_{URDF})^2}
```
Both methods produced matching results for all four configurations; Based on these results, the team's DH model is consistent with the RX150 geometry described by the URDF.

## 6. Verification 3: Product of exponentials
For the third verification, the Product of Exponentials (PoE) method was used to calculate the RX150's forward kinematics. Unlike the DH method, PoE describes each joint using its axis of rotation and position relative to the base frame. This gives us another way to check whether the forward kinematics calculations are correct.

Each joint is represented by a screw axis consisting of an angular component, ω , and a linear component, v wher the angular component describes the direction of rotation, and the linear component accounts for the location of that axis relative to base_link and is calculated using a point q on that axis.

For each revolute joint, the linear component is calculated using:
```math
v_i=-\omega_i\times q_i
```
and the screw axis:

```math
S_i=
\begin{bmatrix}
\omega_i\\
v_i
\end{bmatrix}
```
List of screw axes in the base frame at the home configuration:
| Joint | ωᵢ | vᵢ (m) |
|---|---|---|
| 1 - Waist | (0, 0, 1) | (0, 0, 0) |
| 2 - Shoulder | (0, 1, 0) | (-0.10457, 0, 0) |
| 3 - Elbow | (0, 1, 0) | (-0.25457, 0, 0.05) |
| 4 - Wrist Angle | (0, 1, 0) | (-0.25457, 0, 0.20) |
| 5 - Wrist Rotate | (1, 0, 0) | (0, 0.25457, 0) |

This shows that the shoulder, elbow, and wrist-angle joints all rotate about parallel y-axes, but their linear components differ because each joint is located at a different position. 

For the RX150, the home transformation is:
```math
M=
\begin{bmatrix}
1&0&0&0.358575\\
0&1&0&0\\
0&0&1&0.254570\\
0&0&0&1
\end{bmatrix}
```
Using this matrix and the five screw axes, the end-effector transformation is calculated as:
```math
{}^0T_{\mathrm{tool}}=
e^{[S_1]\theta_1}
e^{[S_2]\theta_2}
e^{[S_3]\theta_3}
e^{[S_4]\theta_4}
e^{[S_5]\theta_5}M
```
The fk_poe(thetas) function in kinematics/fk.py is used to implement this calculation giving the following.

| Configuration | DH(mm) | PoE(mm) | Difference |
|---|---|---|---|
| Home | (358.575, 0, 254.570) | (358.575, 0, 254.570) | 0.0000 mm |
| Waist Rotation | (314.679, 171.910, 254.570) | (314.679, 171.910, 254.570) | 0.0000 mm |
| Folded Arm | (284.374, 0, 203.787) | (284.374, 0, 203.787) | 0.0000 mm |
| General Position | (294.779, 91.186, 155.433) | (294.779, 91.186, 155.433) | 0.0000 mm |

This verification is useful because PoE calculates the gripper pose without using the coordinate frames required by DH. 

## 7. Paste the output
To verify the forward kinematics calculations, the professor provided a verification script.

Runing `python -m scripts.verify_fk`  yeilds.

```
$ python -m scripts.verify_fk

=== home  [0.0, 0.0, 0.0, 0.0, 0.0] ===
DH table:
 [[ 1.      -0.       0.       0.35858]
 [ 0.       1.      -0.       0.     ]
 [-0.       0.       1.       0.25457]
 [ 0.       0.       0.       1.     ]]
URDF chain: AGREE   position 0.0000 mm   rotation 1.11e-16
RViz / tf2: not implemented yet
PoE: AGREE   position 0.0000 mm   rotation 1.11e-16

=== waist only  [0.5, 0.0, 0.0, 0.0, 0.0] ===
DH table:
 [[ 0.87758 -0.47943  0.       0.31468]
 [ 0.47943  0.87758 -0.       0.17191]
 [-0.       0.       1.       0.25457]
 [ 0.       0.       0.       1.     ]]
URDF chain: AGREE   position 0.0000 mm   rotation 1.11e-16
RViz / tf2: not implemented yet
PoE: AGREE   position 0.0000 mm   rotation 1.11e-16

=== arm folded, wrist level  [0.0, -0.4, 0.8, -0.4, 0.0] ===
DH table:
 [[ 1.      -0.       0.       0.28437]
 [ 0.       1.      -0.       0.     ]
 [-0.       0.       1.       0.20379]
 [ 0.       0.       0.       1.     ]]
URDF chain: AGREE   position 0.0000 mm   rotation 1.11e-16
RViz / tf2: not implemented yet
PoE: AGREE   position 0.0000 mm   rotation 1.11e-16

=== general pose  [0.3, -0.2, 0.5, 0.1, 1.2] ===
DH table:
 [[ 0.87992  0.23966  0.41024  0.29478]
 [ 0.27219  0.45343 -0.84871  0.09119]
 [-0.38942  0.85846  0.33375  0.15543]
 [ 0.       0.       0.       1.     ]]
URDF chain: AGREE   position 0.0000 mm   rotation 2.22e-16
RViz / tf2: not implemented yet
PoE: AGREE   position 0.0000 mm   rotation 2.22e-16
```
Results
The results shows that all four configurations agree with DH, URDF and PoE calculations. The ROS transformations for the RViz/tf2 were not automatically retreices causing 'not implemented yet'. Overall, the results show that the three methods produce consistent forward kinematics predictions. 

## 8. Inverse kinematics

Instead of using joint angles to calculate the gripper's position and orientation, inverse kinematic's purpose is to determine which joint angles will place the gripper at a requested target.

**Contraints
**
IK focuses on four constraints: the gripper's x, y, and z positions, along with its pitch angle. 
The solver is defined as:
```python
ik(x, y, z, pitch, roll=0.0)
```
The RX150 has five revolute joints, so it cannot independently control all six components of a general pose: three for position and three for orientation.

To calculate the Joint Angles
1. calculate the waist angle using the requested x and y coordinates:
```math
\theta_1=\mathrm{atan2}(y,x)
```
2. calculate the wrist center. (The specified position refers to the gripper, so first account for the distance)
```math
L_{\mathrm{tool}}=0.065+0.093575\text{ m}
```
3. Once this extension is subtracted, the shoulder and elbow can be treated as a two-link planar mechanism with the two arm lengths:
```math
L_1=0.158\text{ m}
```
```math
L_2=0.150\text{ m}
```

4. Use the wrist center coordinates relative to the shoulder to apply the law of cosines to calculate the elbow angle:
```math
\cos(q_3)=\frac{r_w^2+z_w^2-L_1^2-L_2^2}{2L_1L_2}
```
Here, $r_w$ and $z_w$ are the horizontal and vertical wrist-center coordinates relative to the shoulder.

5. The shoulder angle can then be calculated from the wrist-center position and the selected elbow configuration.

The solver considers two waist configurations: one facing the target directly and another rotated by 180°. For each waist configuration, it also checks both elbow-up and elbow-down positions. This gives up to four possible solutions. 

The IK calculations are implemented in kinematics/ik.py in which before calculating the elbow angle, the solver checks whether the wrist center is within reach of the two main arm links.
```math
|L_1-L_2|\leq\sqrt{r_w^2+z_w^2}\leq L_1+L_2
````

If the wrist center is out of reach, the law-of-cosines calculation gives a value of outside the interval $[-1,1]$. And that that branch is therefore flaged unreachable. The complete function handles the waist configurations, elbow branches, joint-limit checks, and returned solutions.
This approach allows the solver to identify multiple ways of reaching a target while distinguishing between positions that are geometrically unreachable and joint configurations that exceed the physical robot's limits.

The following limits were reported in our group's RX150 model:

| Joint | Lower Limit | Upper Limit |
|---|---|---|
| Waist | ~ -180° | ~ +180° |
| Shoulder | -106° | +100° |
| Elbow | -102° | +95° |
| Wrist Angle | -100° | +123° |
| Wrist Rotate | ~ -180° | ~ +180° |


## 9. Round-trip test
After completing the forward and inverse kinematics functions, The automated tests provided with the project to check whether both calculations worked together correctly were run.

The main idea behind a round-trip test:
    A. Start with a target position and pitch 
    B. Calculate possible joint angles using inverse kinematics
    C. Pass those angles through forward kinematics. 
    
If the calculations are correct, the resulting gripper position matches the original target.

Running 'python -m pytest -v tests' yeilds:

```
=============================================== test session starts ================================================
platform darwin -- Python 3.13.5, pytest-8.3.4, pluggy-1.5.0 -- /opt/anaconda3/bin/python
cachedir: .pytest_cache
rootdir: /Users/carlosgonzalez/ee473-2026f-p1-station-03
plugins: anyio-4.7.0
collected 14 items                                                                                                 
tests/test_roundtrip.py::test_roundtrip[0.25-0.0-0.2-0.0-straight ahead, level] PASSED                       [  7%]
tests/test_roundtrip.py::test_roundtrip[0.2-0.1-0.15--30.0-off to one side, angled down] PASSED              [ 14%]
tests/test_roundtrip.py::test_roundtrip[0.18--0.12-0.25-15.0-other side, angled up] PASSED                   [ 21%]
tests/test_roundtrip.py::test_roundtrip[0.3-0.0-0.1--45.0-reaching low and far] PASSED                       [ 28%]
tests/test_roundtrip.py::test_roundtrip[0.15-0.0-0.3-60.0-high and close] PASSED                             [ 35%]
tests/test_roundtrip.py::test_multiple_branches[0.25-0.0-0.2-0.0-straight ahead, level] PASSED               [ 42%]
tests/test_roundtrip.py::test_multiple_branches[0.2-0.1-0.15--30.0-off to one side, angled down] PASSED      [ 50%]
tests/test_roundtrip.py::test_multiple_branches[0.18--0.12-0.25-15.0-other side, angled up] PASSED           [ 57%]
tests/test_roundtrip.py::test_multiple_branches[0.3-0.0-0.1--45.0-reaching low and far] PASSED               [ 64%]
tests/test_roundtrip.py::test_multiple_branches[0.15-0.0-0.3-60.0-high and close] PASSED                     [ 71%]
tests/test_roundtrip.py::test_limits_are_flagged_not_dropped PASSED                                          [ 78%]
tests/test_roundtrip.py::test_unreachable_returns_empty PASSED                                               [ 85%]
tests/test_roundtrip.py::test_fk_at_home_is_finite PASSED                                                    [ 92%]
tests/test_roundtrip.py::test_rotation_block_is_a_rotation PASSED                                            [100%]
================================================ 14 passed in 0.59s ================================================

```
The round-trip tests confirmed that the IK solutions could reproduce the requested target positions when evaluated using FK. The multiple-branch tests also passed, showing that the solver could identify different joint configurations for the tested targets.

These results show that the FK and IK implementations are consistent for the cases covered by the tests.

## 10. Hardware measurements

Our group selected five joint configurations to test the arm in different positions. These where the home position, positive and negative waist rotations, a folded-arm position, and a general configuration with several joints. We took turns operating the robot and observing its movement.

Predicted Gripper Positions
The following shows the gripper positions using the forward kinematics model.

| Test |  Angles (rad) | Predicted X,Y,Z (mm) |
|---|---|---|
| 1 - Home | (0, 0, 0, 0, 0) | 358.575 , 0.000 , 254.570 |
| 2 - Positive Waist | (0.493, 0, 0, 0, 0) | 315.875 , 169.703 , 254.570 |
| 3 - Folded Arm | (0, -0.4, 0.8, -0.4, 0) | 284.374 , 0.000 , 203.787 |
| 4 - General Position | (0.3, -0.2, 0.5, 0.1, 1.2) | 294.779 , 91.186 , 155.433 |
| 5 - Negative Waist | (-0.493, 0, 0, 0, 0) | 315.875 , -169.703 , 254.570 |

The arm perfeormed the following: Arm returned to the home configuration, its waist rotated about Z in the positive direction, the arm moved into the folded configuration, multiple joints moved to the requested position, and finaly the waist rotated in the negative direction.

The following values were reported during our team's testing: 

| Test | Reported X,Y,Z (mm) | 
|---|---|
| 1 | 358.6 , 0 , 254.6 | 
| 2 | 316 , 170 , 254.6 | 
| 3 | 284 , 0 , 204 | 
| 4 | 294.8 , 91.2 , 155.4 | 
| 5 | NA , NA , NA |

Thus calculating the positioning errors of the robot, the actual measured coordinates can be compared with the forward kinematics predictions. Using the values recorded by the group, the arithmetic differences are:

| Test | ERROR X (mm) | ERROR Y (mm) | ERROR Z (mm) |
|---|---:|---:|---:|
| 1 | +0.025 | 0.000 | +0.030 |
| 2 | +0.125 | +0.297 | +0.030 |
| 3 | −0.374 | 0.000 | +0.213 |
| 4 | +0.021 | +0.014 | −0.033 |
| 5 | NA | NA | NA |

**Error analysis** 
There are also several possible sources of error that would need to be considered in a more detail; as that could introduce error in the physical robot casuing it to move to a slightly different position than the one calculated using forward kinematics.

One possible source of error is the reference point used for measurement, since measuring the gripper fingers instead of the ee_gripper_link frame introduces an additional offset that can is hard to measure with a tape measurer consistently.

One issue encountered in this was valiidating the precision and accuracy of other measurement tools. Not having more detail on the tool makes it difficult to trust its measurements.

Power is a huge consideration that our team had to overcome, a reliable power source is essentiall. 

Another consideration is the robot's link dimensions. The DH model uses dimensions from the URDF, but the actual physical dimensions could have other differences. Overall, our group was able to command the RX150 through all five configurations and observe movements consistent with the forward kinematics model. The tests helped connect the mathematical calculations to the actual movement of the robot.

## 11. Discussion

Our team encounterd various difficulties, but nothing severe. 
One of the biggest things I would do differently, given more time and resources, would be to familiarize myself with the tools and software before actually needing to use them. This would allow let to spend less time figuring out the controls and how everything works and give me more time to focus on the project itself.

During the physical testing, our group had difficulty communicating with the RX150 motors and computer. After checking the setup, we discovered that there was a power supply issue. Once resolved we were able to operate the robot and complete the five selected configurations. 

One limitation of our testing was that we did not have accurate independent measurements of the gripper's position. Though the robot moved as expected, we did not determine its actual positioning error. With better measuring equipment and a consistent reference point, we could compare the predicted and measured positions more accurately.

During physical testing, we ran into issues getting the RX150 to respond to our commands. We had to troubleshoot the ROS 2 setup, check that the necessary programs were running, and verify the robot's connections before we could continue testing.

Overall this project helped me better understand how the Kinematics model of a robotic arm relates to its actual movement. The forward kinematics verification was an important part of the project for me. Comparing the DH calculations with the URDF chain and Product of Exponentials (PoE) method allowed me to check the model using different approaches. The results agreed for all four configurations tested, which gave me more confidence in the calculations. 
Despite this confidence I learned that even when the calculations agree and the robot moves as expected, additional measurements are needed to determine the accuracy of the model.


## 12. Who did what

| Team Member | Contribution |
|---|---|
| Lee Amandoli | Individual DH model, forward and inverse kinematics code, software verification, and report. Also participated in setting up and testing the physical RX150. |
| Carlos Gonzalez | individual DH model, forward and inverse kinematics implementation, verification scripts, automated testing, and report. I also helped with the robot setup, operation, and observations during hardware testing. |
| Geovani Palomec | Individual DH model. Participated kinematics implementation & script verifications, operating and monitoring the robot, and observing the results during hardware testing. |

Responsibilities were rotated among team members to give all the opportunity to operate, monitor, and record the RX150 results. We devided the work to test the five selected configurations and ensure robot operation.
