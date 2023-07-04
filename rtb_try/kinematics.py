# %%
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

# %matplotlib notebook

# %% [markdown]
# The Toolbox supports models defined using a number of different conventions.  We will load a very classical model, a Puma560 robot defined in terms of standard Denavit-Hartenberg parameters

# %%
p560 = rtb.models.DH.Puma560()

# %% [markdown]
# Now we can display the simple Denavit-Hartenberg parameter model

# %%
print(p560)

# %% [markdown]
# The first table shows the kinematic parameters, and from the column titles we can see clearly that this is expressed in terms of standard Denavit-Hartenberg parameters.  The first column shows that the joint variables qi are rotations since they are in the θ column.  Joint limits are also shown.  Joint flip (motion in the opposite sense) would be indicated by the joint variable being shown as for example like `-q3`, and joint offsets by being shown as for example like `q2 + 45°`.
# 
# The second table shows some named joint configurations.  For example `p560.qr` is 

# %%
p560.qr

# %% [markdown]
# If the robot had a base or tool transform they would be listed in this table also.
# 
# This object is a subclass of `DHRobot`, equivalent to the `SerialLink` class in the MATLAB version of the Toolbox.
# This class has many methods and attributes, and we will explore some of them in this notebook.

# %% [markdown]
# We can easily display the robot graphically

# %%
p560.plot(p560.qn);

# %% [markdown]
# where `qn` is one of the named configurations shown above, and has the robot positioned to work above a table top.  You can use the mouse to rotate the plot and view the robot from different directions.  The grey line is the _shadow_ which is a projection of the robot onto the xy-plane.
# 
# In this particular case the end-effector pose is given by the forward kinematics

# %%
p560.fkine(p560.qn)

# %% [markdown]
# which is a 4x4 SE(3) matrix displayed in a color coded way with rotation matrix in red, translation vector in blue, and constant elements in grey.  This is an instance of an `SE3` object safely encapsulates the SE(3) matrix.  This class, and related ones, are implemented by the [Spatial Math Toolbox for Python](https://github.com/petercorke/spatialmath-python).
# 
# You can verify the end-effector position, the blue numbers are from top to bottom the x-, y- and z-coordinates of the end-effector position, match the plot shown above.
# 
# We can manually adjust the joint angles of this robot (click and drag the sliders) to see how the shape of the robot changes and how the end-effector pose changes

# %%
# p560.teach(); # works from console, hangs in Jupyter

# %% [markdown]
# An important problem in robotics is _inverse kinematics_, determining the joint angles to put the robot's end effector at a particular pose.
# 
# Suppose we want the end-effector to be at position (0.5, 0.2, 0.1) and to have its gripper pointing (its _approach vector_) in the x-direction, and its fingers one above the other so that its _orientation vector_ is parallel to the z-axis.
# 
# We can specify that pose by composing two SE(3) matrices:
# 
# 1. a pure translation
# 2. a pure rotation defined in terms of the orientation and approach vectors

# %%
T = SE3(0.5, 0.2, 0.5) * SE3.OA([0,0,1], [1,0,0])
T

# %% [markdown]
# Now we can compute the joint angles that results in this pose

# %%
sol = p560.ikine_LM(T)

# %% [markdown]
# which returns the joint coordinates as well as solution status

# %%
sol

# %% [markdown]
# indicating, in this case, that there is no failure. The joint coordinates are

# %%
sol.q

# %% [markdown]
# and we can confirm that this is indeed an inverse kinematic solution by computing the forward kinematics

# %%
p560.fkine(sol.q)

# %% [markdown]
# which matches the original transform.

# %% [markdown]
# A simple trajectory between two joint configuration is

# %%
qt = rtb.tools.trajectory.jtraj(p560.qz, sol.q, 50)

# %% [markdown]
# The result is a _namedtuple_ with attributes `q` containing the joint angles, as well as `qd`, `qdd` and `t` which hold the joint velocity, joint accelerations and time respectively.  
# 
# The joint angles are a matrix with one column per joint and one row per timestep, and time increasing with row number.

# %%
qt.q

# %% [markdown]
# We can plot this trajectory as a function of time using the convenience function `qplot`

# %%
# rtb.tools.trajectory.qplot(qt.q, block=False)
rtb.tools.plot.xplot(qt.q, block=False)

# %% [markdown]
# and then we can animate this

# %%
p560.plot(qt.q, dt=0.1)

# %% [markdown]
# _Note: animation not working in Jupyter..._

# %% [markdown]
# The inverse kinematic solution was found using an iterative numerical procedure.  It is quite general but it has several drawbacks:
# - it can be slow
# - it may not find a solution, if the initial choice of joint coordinates is far from the solution (in the case above the default initial choice of all zeros was used)
# - it may not find the solution you want, in general there are multiple solutions for inverse kinematics.  For the same end-effector pose, the robot might:
#     - have it's arm on the left or right of its waist axis, 
#     - the elbow could be up or down, and
#     - the wrist can flipped or not flipped.  For a two-finger gripper a rotation of 
#       180° about the gripper axis leaves the fingers in the same configuration.
# 
# Most industrial robots have a _spherical wrist_ which means that the last three joint axes intersect at a single point in the middle of the wrist mechanism.  We can test for this condition

# %%
p560.isspherical()

# %% [markdown]
# This greatly simplifies things because the last three joints only control orientation and have no effect on the end-effector position.  This means that only the first three joints define position $(x_e, y_e, z_e)$.  Three joints that control three unknowns is relatively easy to solve for, and analytical solutions (complex trigonmetric equations) can be found, and in fact have been published for most industrial robot manipulators.
# 
# The Puma560 has an analytical solution.  We can request the solution with the arm to the left and the elbow up, and the wrist not flipped by using the configuration string `"lun"`
# 

# %%
sol = p560.ikine_a(T, "lun")
sol

# %% [markdown]
# which is different to the values found earlier, but we can verify it is a valid solution

# %%
p560.fkine(sol.q)

# %% [markdown]
# In fact the solution we found earlier, but didn't explicitly specify, is the right-handed elbow-up configuration

# %%
sol = p560.ikine_a(T, "run")
sol.q

# %% [markdown]
# Other useful functions include the manipulator Jacobian which maps joint velocity to end-effector velocity expressed in the world frame

# %%
p560.jacob0(p560.qn)

# %%



