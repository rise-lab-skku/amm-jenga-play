# amm-jenga-play
Jenga play with Franka Reasearch 3 [SKKU 2023 URP Team2's Project]

## How to use
<details><summary>create an instance of Franka Research 3</summary>

```python
import fr3
robot=fr3.fr3()
```
## modifying module
in \_\_init\_\_ method, 
```python
URDF_read(
    your_xacro_file,tld=your_root_path,xacro_tld=xacro_root_path
)
```
for gripper changes, in fr3.py
```python
self.grippers[0].tool=SE3(x,y,z)
```
and in franka_description/robots/common/inertial.yaml, change values
```yaml
leftfinger:
    inertia:
        xx: I_xx
        yy: I_yy
        zz: I_zz
```

</details>
<details><summary>transform and rotation</summary>

# Make the joint at 45 degrees
```python
# Rotation
# This is a constant rotation around the x-axis by 90 degrees
rx_cons = rtb.ET.Rx(np.pi / 2)
transform = rx_cons.A()
sm_transform = sm.SE3(transform)
# Make the joint at 45 degrees
rx_var = rtb.ET.Rx()
q = np.pi / 4
transform = rx_var.A(q)
sm_transform = sm.SE3(transform)

# Translation
# This is a constant translation along the y-axis by 25 cm
ty_cons = rtb.ET.ty(0.25)\
transform = ty_cons.A()
sm_transform = sm.SE3(transform)

# Make the joint at 15 cm
ty_var = rtb.ET.ty()
q = 0.15
transform = ty_var.A(q)
sm_transform = sm.SE3(transform)
```


</details>

<details><summary>no attribure 'qplot' error(trajectory.qplot)</summary>

```python
rtb.tools.trajectory.qplot(qt.q, block=False)
```

``` Error code: module 'roboticstoolbox.tools.trajectory' has no attribute 'qplot' ```
- there are class name "Tracjectory" in roboticstoolbox.tools.trajectory
- we can change the the code as follow
- xplot in qplot module in plot module in RTB

```
rtb.tools.plot.xplot(qt.q, block=False)
```

</details>