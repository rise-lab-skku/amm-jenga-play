# %%
# try:
#   # We must install required packages if we are in Google Colab
#   import google.colab
#   %pip install roboticstoolbox-python>=1.0.2
# except:
#   # We are not in Google Colab
#   # Apply custon style to notebook
#   from IPython.core.display import HTML
#   import pathlib
#   styles_path = pathlib.Path(pathlib.Path().absolute(), "style", "style.css")
#   styles = open(styles_path, "r").read()
#   HTML(f"<style>{styles}</style>")

# %% [markdown]
# _Note: This notebook must be run locally. Due to how the Swift Simulator operates, this notebook will not run on Google Colab_

# %% [markdown]
# # 3.0 Resolved-Rate Motion Control
# 
# $\large{\text{Manipulator Differential Kinematics}} \\ \large{\text{Part I: Kinematics, Velocity, and Applications}}$
# 
# $\text{By Jesse Haviland and Peter Corke}$
# 
# <br>
# 
# The sections of the Tutorial paper related to this notebook are listed next to each contents entry.
# It is beneficial to read these sections of the paper before attempting the notebook Section.
# 
# ### Contents
# 
# [3.1 Robotics Toolbox for Python - Robot Class Introduction ](#utility)
# 
# [3.2 Swift Introduction](#swift)
# 
# [3.3 Resolved-Rate Motion Control](#rrmc)
# * Resolved-Rate Motion Control
# 
# [3.4 Position based Servoing](#pbs)
# * Resolved-Rate Motion Control
# 
# [3.5 Velocity Profile](#err)
# * Resolved-Rate Motion Control
# 
# [3.6 Using the Robotics Toolbox](#rtb)
# 
# [3.7 Dynamic Target Example](#rea)
# * Resolved-Rate Motion Control

# %%
# We will do the imports required for this notebook here

# numpy provides import array and linear algebra utilities
import numpy as np

# the robotics toolbox provides robotics specific functionality
import roboticstoolbox as rtb

# spatial math provides objects for representing transformations
import spatialmath as sm

# swift is a lightweight browser-based simulator which comes eith the toolbox
from swift import Swift

# the Python math library
import math

# spatialgeometry is a utility package for dealing with geometric objects
import spatialgeometry as sg

# typing utilities
from typing import Tuple

# %% [markdown]
# <br>
# 
# <a id='utility'></a>
# 
# ### 3.1 Robotics Toolbox for Python - Robot Class Introduction 
# ---

# %% [markdown]
# The `roboticstoolbox` package provides many ways of interfacing with a robot and performing dynamic or kinematic operations. In Notebooks 1 and 2 we used the `ET` and `ETS` classes. These provide a low level kinematic-only representation of a robot. 
# 
# Robots contain more information than just kinematic data (which is all that the ETS stores). For example, they may contain dynamic data (mass, inertia, centre of gravity etc.) and geometric data. The `Link` class is used to hold this data for a single link within a manipulator. A `Link` contains an `ETS` which is used to represent the kinematics of that link.
# 
# We use the `Robot` class as a high level way of representing a whole robot along with all data. A `Robot` class instance contains a tree structure to represent how links are connected. Links can only have up to one parent but unlimited children. With this structure, we can represent serial-link manipulators (such as the Panda), branched mechanisms (such as the YuMi two armed manipulator), but not mechanisms which form closed loops in their structure.  The `Robot` instance may also hold additional data such as known joint configurations and the robot's pose relative to the world-frame.
# 
# A robot will typically contain an end-effector or gripper. We represent these using the `Gripper` class. A `Robot` contains a list of grippers under the `Robot.grippers` attribute. Each `Gripper` contains its own tree structure of `Link` objects (Think of a `Gripper` as a mini `Robot`).
# 
# We can convert a section or a sequence of a `Robot` to an `ETS` using the `Robot.ets(start, end)` method. The `start` argument defaults to the base link of the robot. The `end` link will default to the first gripper if present, otherwise the first leaf of the link tree. Otherwise `start` and `end` may be a `str` representing the name of the desired link or a reference to the desired link. Note that the referenced link must be within the robot or its grippers. There are no other limits on `start` and `end`, for example `start` could be after `end` in the chain or even from one branch to another (think of going from the link from the left gripper to the right gripper on the YuMi). 
# 
# Within the `roboticstoolbox`, we have many defined robots. These can be accessed from the `.models` namespace.
# 

# %%
# Make a panda robot
panda = rtb.models.Panda()

print(panda)

# %% [markdown]
# From the output of the above cell, we can see that the Panda has several links, and the ETS of each link. We can also see the named configurations `qr` and `qz` which we can access through `panda.qr` and `panda.qz` respectively.
# 
# We can perform kinematic operations with a `Robot` object in a similar manner to an `ETS` object. The only difference is that we can pass arguments defining the `start` and `end` links. The `start` and `end` links define which component of the `Robot` the kinematic operation is performed on and works in the same way described above for the `Robot.ets(start, end)` method.
# 
# In the following cell, will perform forward kinematics.

# %%
# Use the default start and end links
print(panda.fkine(panda.qr))

# Use string start and end links
print(panda.fkine(panda.qr, start="panda_link0", end="panda_link5"))

# Use reference start and end links
print(panda.fkine(panda.qr, start=panda.links[3], end=panda.grippers[0]))

# %% [markdown]
# <br>
# 
# <a id='swift'></a>
# 
# ### 3.2 Swift Introduction 
# ---

# %% [markdown]
# Swift is a browser based robot simulator. We must first make the environment and then use the `launch` method to open the simulator.
# 
# Each time `launch()` is called, a new browser tab is opened with a clean Swift environment.
# 
# By passing the `realtime=True` keyword argument to the `launch` method, the simulator will try to run as close as possible to realtime, instead of as fast as possible.
# 
# We can also add geometric shapes (and custom meshes but we won't show that here). Lets add two axes, to show the current end-effector pose and another to show some other pose such as the desired end-effector pose. There are other primitive shapes such as cuboids, arrows, cylinders and spheres.

# %%
# Make the environment
env = Swift()

# Launch the simulator, will open a browser tab in your default
# browser (chrome is recommended)
# The realtime flag will ask the simulator to simulate as close as
# possible to realtime as apposed to as fast as possible
# We can also choose to see Swift within the notebook using the
# browser="notebook" flag
env.launch(realtime=True, browser="notebook")

# We can then add our robot to the simulator envionment
env.add(panda)

# end-effector axes
ee_axes = sg.Axes(0.1)

# goal axes
goal_axes = sg.Axes(0.1)

# Add the axes to the environment
env.add(ee_axes)
env.add(goal_axes) 

# %% [markdown]
# We can change the pose of the shapes using the `.T` attribute

# %%
# We can set the pose of the shape using the `.T` attribute.
# This pose can be either a spatialmat SE3 or a 4x4 ndarray
ee_axes.T = panda.fkine(panda.q)

# Set the goal axes to something along the x axis
goal_axes.T = sm.SE3.Trans(0.5, 0.0, 0.5)

# step the environment to view the changes
env.step(0)

# %% [markdown]
# The joint configuration which `Swift` displays the robot is stored in `Robot.q`. Therefore, by modifying this attribute, we can modify the robot in `Swift`. Note that we have to call `Swift.step(dt)` to step the simulator and visualise any changes and `dt` corresponds to the length of time to simulate for.

# %%
# Set the joint coordinates to qr
panda.q = panda.qr

# Step the environment with a time step of 0 seconds
env.step(0)

# %% [markdown]
# `Swift` uses the vector stored in `Robot.qd` to simulate joint velocities. Therefore, by modifying this attribute, we can make the robot in `Swift` move.

# %%
# Set all joint velocities to 0.1 rad/s
panda.qd = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# Step the environment 100 times with a time step of 50 milliseconds
for _ in range(100):
    env.step(0.05)

# %% [markdown]
# <br>
# 
# <a id='rrmc'></a>
# 
# ### 3.3 Resolved-Rate Motion Control
# ---

# %% [markdown]
# 
# 
# Resolved-rate motion control (RRMC) is a simple and elegant method to generate straight-line motion of the end effector. RRMC is a direct application of the first-order differential equation
# 
# \begin{equation*}
#    {^0\bf{\nu}} = {^0\bf{J}}(\bf{q}) \ \bf{\dot{q}}
# \end{equation*}
# 
# where ${^0(\cdot)}$ denotes the base-frame. RRMC is simply the inverse of the above
# 
# \begin{equation*}
#     \bf{\dot{q}} = {^0\bf{J}}(\bf{q})^{-1} \ {^0\bf{\nu}}
# \end{equation*}
# 
# which can only be solved when $\bf{J}(\bf{q})$ is square (and non-singular), which is when the robot has 6 degrees-of-freedom.
# 
# For redundant robots there is no unique solution for the above equation. Consequently, the most common approach is to use the Moore-Penrose pseudoinverse
# 
# \begin{equation*}
#     \bf{\dot{q}} = {^0\bf{J}}(\bf{q})^{+} \ {^0\bf{\nu}}.
# \end{equation*}
# 
# The pseudoinverse will return joint velocities with the minimum velocity norm of the possible solutions.
# 
# Since the Panda robot has 7 degrees-of-freedom, we will use the psuedoinverse method

# %%
# Our desired end-effector velocity
# A constant 0.1 m/s along the x-axis
ev = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]

# The base-frame manipulator Jacobian in the qr configuration
J = panda.jacob0(panda.qr)

# Calculate the required joint velocities to achieve the desired end-effector velocity ev
dq = np.linalg.pinv(J) @ ev

# Visualise the results
print(np.round(dq, 4))

# %% [markdown]
# Lets check this out working in the `Swift` simulator

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# Specify our desired end-effector velocity
ev = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]

# Specify our timestep
dt = 0.05

# Run the simulation for 5 seconds
for _ in range(50):

    # Work out the manipulator Jacobian using the current robot configuration
    J = panda.jacob0(panda.q)

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Step the simulator by dt seconds
    env.step(dt)

# %% [markdown]
# In the next example we use the end-effector frame Jacobian and rotate around the z-axis

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# Specify our desired end-effector velocity
ev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

# Specify our timestep
dt = 0.05

# Run the simulation for 5 seconds
for _ in range(50):

    # Work out the manipulator Jacobian using the current robot configuration
    J = panda.jacobe(panda.q)

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Step the simulator by dt seconds
    env.step(dt)

# %% [markdown]
# <br>
# 
# <a id='pbs'></a>
# 
# ### 3.4 Position based Servoing
# ---

# %% [markdown]
# A more useful application of RRMC is to employ it in a closed-loop pose controller which we denote position-based servoing (PBS). Using this method we can get the end-effector to travel in a straight line, in the robot's task space, towards some desired end-effector pose. The PBS scheme relies on an error vector which represents the translation and rotation from the end-effector's current pose to the desired pose
# 
# \begin{equation*}
#     \bf{e} =
#     \begin{pmatrix}
#         \tau\left({^0\bf{T}}_{e^*}\right) - \tau\left({^0\bf{T}}_{e}\right) \\
#         \alpha
#         \left(
#             \rho\left({^0\bf{T}}_{e^*}\right)
#             \rho\left({^0\bf{T}}_{e}\right)^\top
#         \right)
#     \end{pmatrix} \in \mathbb{R}^6
# \end{equation*}
# 
# where the top three rows correspond to the translational error in the world frame, the bottom three rows correspond to the rotational error in the world frame, ${^0\bf{T}}_{e}$ is the forward kinematics of the robot which represents the end-effector pose in the base frame of the robot, ${^0\bf{T}}_{e^*}$ is the desired end-effector pose in the base frame of the robot ($\cdot^*$ denotes desired not actual), and $\alpha(\cdot) : \bf{SO}(3) \mapsto \mathbb{R}^3$ transforms a rotation matrix to its Euler vector equivalent. 

# %% [markdown]
# To calculate the Euler vector (also known as angle-axis representation) we first calculate
# 
# \begin{align*}
#     \bf{l} =
#     \begin{pmatrix}
#         r_{32} - r_{23} \\
#         r_{13} - r_{31} \\
#         r_{21} - r_{12} \\
#     \end{pmatrix}.
# \end{align*}
# 
# If $\bf{R}$ is not a diagonal matrix then the angle-axis equivalent of $\bf{R}$ is calculated as
# 
# \begin{align*}
#     \alpha
#     \left(
#         \bf{R}
#     \right) &=
#     \dfrac
#     {\text{atan2}
#     \left(
#         \lVert\bf{l}\rVert, r_{11} + r_{22} + r_{33} - 1
#     \right)}
#     {\lVert\bf{l}\rVert}\bf{l}.
# \end{align*}
# 
# If $\bf{R}$ is a diagonal matrix then we use different formulas. For the case where $\begin{pmatrix}r_{11} & r_{22} & r_{33}\end{pmatrix} = \begin{pmatrix}1&1&1\end{pmatrix}$ then $\alpha(\bf{R}) = \begin{pmatrix}0 & 0 & 0\end{pmatrix}^\top$ otherwise
# 
# \begin{align*}
#     \alpha
#     \left(
#         \bf{R}
#     \right) &=
#     \frac{\pi}{2}
#     \begin{pmatrix}
#         r_{11} + 1 \\
#         r_{22} + 1 \\
#         r_{33} + 1 \\
#     \end{pmatrix}.
# \end{align*}
# 
# In the following cell, we create a method `angle_axis` which calculates the vector $\bf{e}$ from two poses `T` and `Td`.

# %%
def angle_axis(T: np.ndarray, Td: np.ndarray) -> np.ndarray:
    """
    Returns the error vector between T and Td in angle-axis form.

    :param T: The current pose
    :param Tep: The desired pose

    :returns e: the error vector between T and Td
    """

    e = np.empty(6)

    # The position error
    e[:3] = Td[:3, -1] - T[:3, -1]

    R = Td[:3, :3] @ T[:3, :3].T

    li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if np.linalg.norm(li) < 1e-6:
        # If li is a zero vector (or very close to it)

        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = np.linalg.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    e[3:] = a

    return e

# %% [markdown]
# To construct the PBS scheme we take the error term $\bf{e}$ to set $\bf{\nu}$ in RRMC equation at each time step
# 
# \begin{align*}
#     \bf{\nu} = k \bf{e}
# \end{align*}
# 
# where $k$ is a proportional gain term which controls the rate of convergence to the goal. 
# 
# It is important to remember that the vector $\bf{e}$ is non-homogeneous since its elements have different units (metres and radians).
# 
# Therefore, $k$ can also be a diagonal matrix to set gains for each task-space DoF
# 
# \begin{align*}
#     \bf{k} =
#     \text{diag}
#     (
#         k_t, \ k_t, \ k_t, \ k_r, \ k_r, \ k_r
#     )
# \end{align*}
# 
# where $k_t$ is the translational motion gain and $k_r$ is the rotational motion gain. Note that since the error $\bf{e}$ is in the base-frame of the robot we must use the base-frame manipulator Jacobian $^0\bf{J}(\bf{q})$.

# %%
def p_servo(
    Te: np.ndarray, Tep: np.ndarray, gain: np.ndarray, threshold: float = 0.1
) -> Tuple[np.ndarray, bool]:
    """
    Position-based servoing.

    Returns the end-effector velocity which will cause the robot to approach
    the desired pose.

    :param Te: The current pose of the end-effecor in the base frame.
    :type wTe: ndarray
    :param Tep: The desired pose of the end-effecor in the base frame.
    :type wTep: ndarray
    :param gain: The gain for the controller. A vector corresponding to each
        Cartesian axis.
    :type gain: array-like
    :param threshold: The threshold or tolerance of the final error between
        the robot's pose and desired pose
    :type threshold: float

    :returns v: The velocity of the end-effector which will casue the robot
        to approach Tep
    :rtype v: ndarray(6)
    :returns arrived: True if the robot is within the threshold of the final
        pose
    :rtype arrived: bool
    """

    # Calculate the pose error vector
    e = angle_axis(Te, Tep)

    # Construct our gain diagonal matrix
    k = np.diag(gain)

    # Calculate our desired end0effector velocity
    v = k @ e

    # Check if we have arrived
    arrived = True if np.sum(np.abs(e)) < threshold else False

    return v, arrived


# %% [markdown]
# Lets try it out in `Swift`

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)
env.add(ee_axes)
env.add(goal_axes)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# A variable to specify when to break the loop
arrived = False

# Specify the gain for the p_servo method
gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# Specify our timestep
dt = 0.05

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.3, 0.3, 0.25) * sm.SE3.RPY(0.0, 0.5, 0.5)
Tep = Tep.A

# Set the goal axes to Tep
goal_axes.T = Tep

# Run the simulation until the robot arrives at the goal
while not arrived:

    # Work out the base frame manipulator Jacobian using the current robot configuration
    J = panda.jacob0(panda.q)

    # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
    Te = panda.fkine(panda.q).A

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required end-effector velocity and whether the robot has arrived
    ev, arrived = p_servo(Te, Tep, gain=gain, threshold=0.001)

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Update the ee axes
    ee_axes.T = Te

    # Step the simulator by dt seconds
    env.step(dt)

# %% [markdown]
# <br>
# 
# <a id='err'></a>
# 
# ### 3.5 Velocity Profile
# ---

# %% [markdown]
# The control scheme we have just described will cause the error to asymptotically decrease to zero. 
# For real applications this is slow and impractical.
# 
# We can improve this by increasing $k_t$ and $k_r$ and capping the $\bf{\nu}$ norm at some value $\nu_{m}$, before stopping when the error norm drops below some value ${\lVert \bf{e} \rVert}_m$
# 
# \begin{align*}
#     \bf{\nu}
#     &=
#     \left\{ 
#     \begin{matrix*}[l]
#         \bf{k}\bf{e} \dfrac{\nu_{m}}{\lVert \bf{k}\bf{e} \rVert} & \text{if} \ \ \lVert \bf{k}\bf{e} \rVert > \nu_{m} \\
#         \ \bf{k}\bf{e} & \text{if} \ \ \nu_{m} \geq \lVert \bf{k}\bf{e} \rVert > {\lVert \bf{e} \rVert}_m \\
#         \bf{0} & \text{otherwise.}
#     \end{matrix*}
#     \right.
# \end{align*}
# 
# This will cause the error to reduce at a consistent rate until the end of the motion where the velocity asymptotically decreases to safely stop the robot. The effect of this is displayed in Figure \ref{fig:rrmc2}. Alternatively, the above could be modified to create different velocity profiles, such as a linearly decreasing velocity norm, if desired. Additionally, the above can be modified to adjust the translational and angular velocity pofiles separately.

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)
env.add(ee_axes)
env.add(goal_axes)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# A variable to specify when to break the loop
arrived = False

# Specify the gain for the p_servo method
# Note this is higher than the previous cell
gain = 3 * np.array([1.0, 1.0, 1.0, 1.3, 1.3, 1.3])

# Specify our timestep
dt = 0.05

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.3, 0.3, 0.25) * sm.SE3.RPY(0.0, 0.5, 0.5)
Tep = Tep.A

# Set the goal axes to Tep
goal_axes.T = Tep

# Set the maximum ee velocity
ev_max = 1.0

# Run the simulation until the robot arrives at the goal
while not arrived:

    # Work out the base frame manipulator Jacobian using the current robot configuration
    J = panda.jacob0(panda.q)

    # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
    Te = panda.fkine(panda.q).A

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required end-effector velocity and whether the robot has arrived
    ev, arrived = p_servo(Te, Tep, gain=gain, threshold=0.001)

    # Calculate the magnitude of the ee velocity
    ev_norm = np.linalg.norm(ev)

    # if ee vel is greater than the max
    if ev_norm > ev_max:
        ev = (ev_max / ev_norm) * ev

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Update the ee axes
    ee_axes.T = Te

    # Step the simulator by dt seconds
    env.step(dt)

# %% [markdown]
# <br>
# 
# <a id='rtb'></a>
# 
# ### 3.6 Using the Robotics Toolbox
# ---

# %% [markdown]
# We defined our own `angle_axis` and `p_servo` methods in the Notebook for demonstration. However, the Toolbox has it own implementations of these methods which we demonstrate below.

# %% [markdown]
# The toolbox `p_servo` can use two different error vectors to calculate the required end-effector velocity. The default method is `rpy` in the end-effector frame. Check the method docstring for more details about this method.

# %% [markdown]
# Example using `rpy` option in the `p_servo` method. Note that we must use `jacobe` with this method since the error is in the end-effector frame.

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)
env.add(ee_axes)
env.add(goal_axes)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# A variable to specify when to break the loop
arrived = False

# Specify the gain for the p_servo method
gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# Specify our timestep
dt = 0.05

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.3, 0.3, 0.25) * sm.SE3.RPY(0.0, 0.5, 0.5)
Tep = Tep.A

# Set the goal axes to Tep
goal_axes.T = Tep

# Run the simulation until the robot arrives at the goal
while not arrived:

    # Work out the base frame manipulator Jacobian using the current robot configuration
    J = panda.jacobe(panda.q)

    # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
    Te = panda.fkine(panda.q).A

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required end-effector velocity and whether the robot has arrived
    ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.001, method='rpy')

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Update the ee axes
    ee_axes.T = Te

    # Step the simulator by dt seconds
    env.step(dt)

# %% [markdown]
# Example using `angle-axis` option in the `p_servo` method. Note that we must use `jacob0` with this method since the error is in the base frame.

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)
env.add(ee_axes)
env.add(goal_axes)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# A variable to specify when to break the loop
arrived = False

# Specify the gain for the p_servo method
gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# Specify our timestep
dt = 0.05

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.3, 0.3, 0.25) * sm.SE3.RPY(0.0, 0.5, 0.5)
Tep = Tep.A

# Set the goal axes to Tep
goal_axes.T = Tep

# Run the simulation until the robot arrives at the goal
while not arrived:

    # Work out the base frame manipulator Jacobian using the current robot configuration
    J = panda.jacob0(panda.q)

    # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
    Te = panda.fkine(panda.q).A

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required end-effector velocity and whether the robot has arrived
    ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.001, method='angle-axis')

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Update the ee axes
    ee_axes.T = Te

    # Step the simulator by dt seconds
    env.step(dt)

# %% [markdown]
# <br>
# 
# <a id='rea'></a>
# 
# ### 3.7 Dynamic Target Example
# ---

# %% [markdown]
# For a _reactive_ task, where a dynamic environment is causing the goal pose to change, we simply need to update the desired end-effector pose ${^0\mathbf{T}}_{e^*}$ at each time step. In this example, we have full environemnt knowledge so we know where the target is at all times. In practice, this may not be possible. In such a scenario, a technique such as position-based visual servoing could be used to achieve this.

# %%
# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True, browser="notebook")
env.add(panda)
env.add(ee_axes)
env.add(goal_axes)

# Change the robot configuration to the ready position
panda.q = panda.qr

# Step the sim to view the robot in this configuration
env.step(0)

# A variable to specify when to break the loop
arrived = False

# Specify the gain for the p_servo method
gain = 2.0 * np.ones(6)

# Specify our timestep
dt = 0.05

# Set the goal axes to Tep
goal_axes.T = sm.SE3.Trans(0.4, -0.2, 0.05) * sm.SE3.Rx(np.pi)

# Make the target move, set its velocity
goal_axes.v = np.array([0.0, 0.02, 0.02, 0.0, 0.0, 0.0])

# Run the simulation for 300 steps
# As the target never stops moving, the robot will never arrive at the goal,
# but it will continuously track towards it
for _ in range(300):

    # Work out the base frame manipulator Jacobian using the current robot configuration
    J = panda.jacob0(panda.q)

    # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
    Te = panda.fkine(panda.q).A

    # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
    # use the pseudoinverse (the pinv method)
    J_pinv = np.linalg.pinv(J)

    # Calculate the required end-effector velocity
    # Here we feed in the goal_axes.T, which is an SE3, as our goal pose
    ev, _ = rtb.p_servo(Te, goal_axes.T, gain=gain, threshold=0.001, method='angle-axis')

    # Calculate the required joint velocities and apply to the robot
    panda.qd = J_pinv @ ev

    # Update the ee axes
    ee_axes.T = Te

    # Step the simulator by dt seconds
    env.step(dt)


