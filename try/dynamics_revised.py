# %%
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

# %matplotlib notebook
# %config NotebookBackend.figure_format = 'retina'

# %% [markdown]
# We will instantiate a model of the Puma 560 robot which has well known inertial parameters

# %%
p560 = rtb.models.DH.Puma560()

# %% [markdown]
# and show its configuration in a typical desktop working pose

# %%
p560.plot(p560.qn, block=False)

# %% [markdown]
# The rigid-body equations of motion for a robot are a set of coupled differential equations
# $$
# \mathbf{M}(\mathit{\ddot{q}}) \mathit{\ddot{q}} + \mathbf{C}(\mathit{q}, \mathit{\dot{q}}) \mathit{\dot{q}} + \mathbf{g}(\mathit{q}) = \mathit{\tau}
# $$
# which relate the motion of the robot $(\mathit{q}, \mathit{\dot{q}}, \mathit{\ddot{q}})$ and the applied torque $\mathit{\tau}$.  The coefficients in this equation are:
# - the inertia or mass matrix $\mathbf{M}(\mathit{\ddot{q}})$ which is a function of joint configuration
# - the centripetal and Coriolis or velocity term which is a function of joint configuration and rate
# - the gravity load which is a function of joint configuration
# 
# If the robot is not moving, that is $\mathit{q} = \mathit{\dot{q}} = 0$ then the equation becomes
# $$
# \mathbf{g}(\mathit{q}) = \mathit{\tau}
# $$
# where $\mathit{\tau}$ is the torque required for this condition $\mathit{q} = \mathit{\dot{q}} = 0$ to be true, that is, the torque required to stop the robot falling under its own weight.  The toolbox can compute this

# %%
p560.gravload(p560.qn)

# %% [markdown]
# and it shows, as expected, that the shoulder is exerting significant torque to hold the arm up and stationary.
# 
# The inertia matrix relates torque to joint acceleration and is the mass in a multi-dimensional version of Newton's second law $F = m a$.  In this configuration the inertia matrix is

# %%
p560.inertia(p560.qn)

# %% [markdown]
# The diagonal elements $M_{jj}$ indicate the inertia experienced by the joint $j$, ie. Newton's second law for this joint is $\tau_j  = M_{jj} \ddot{q}_j$.
# 
# The matrix is symmetric and the off-diagonal terms $M_{ij} = M_{ji}$ couple acceleration of one joint into a disturbance torque on another joint, ie.  $\tau_j = M_{ij} \ddot{q}_i$.
# 
# The inertia matrix is a function of joint configuration, that is, the elements of the inertia matrix change as we vary the angles of joints 1 and 2, ie. $q_2$ and $q_3$.  It is this configuration varying inertia and coupling between joints that is a fundamental challenge for high-quality joint control.

# %%
N = 100
(Q2, Q3) = np.meshgrid(np.linspace(-pi, pi, N), np.linspace(-pi, pi, N))
M11 = np.zeros((N,N))
M12 = np.zeros((N,N))
for i in range(N):
    for j in range(N):
        M = p560.inertia(np.r_[0, Q2[i,j], Q3[i,j], 0, 0, 0])
        M11[i,j] = M[0,0]
        M12[i,j] = M[0,1]

# %% [markdown]
# The inertia "seen" by joint 1 varies as a function of $q_2$ and $q_3$ as shown below

# %%
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(Q2, Q3, M11, cmap=cm.coolwarm, linewidth=0, antialiased=False)
fig.colorbar(surf, shrink=0.9, aspect=10, pad=0.12)
ax.set_xlabel('$q_2$ (rad)')
ax.set_ylabel('$q_3$ (rad)')
ax.set_zlabel('$M_{11}$ ($kg.m^2$)')
plt.show()

# %% [markdown]
# The ratio of maximum to minimum values is

# %%
M11.max() / M11.min()

# %% [markdown]
# The coupling inertia between joints 1 and 2 also varies with configuration and we can plot that as well

# %%
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(Q2, Q3, M12, cmap=cm.coolwarm, linewidth=0, antialiased=False)
fig.colorbar(surf, shrink=0.9, aspect=10, pad=0.12)
ax.set_xlabel('$q_2$ (rad)')
ax.set_ylabel('$q_3$ (rad)')
ax.set_zlabel('$M_{12}$ ($kg.m^2$)')
plt.show()

# %% [markdown]
# The velocity terms are a bit harder to comprehend but they mean that rotation of one joint (and its link) can exert a torque on other joints.  Consider that the should joint is rotating at 1 rad/sec, then the torque will be 

# %%
qd = np.r_[0, 1, 0, 0, 0, 0]
p560.coriolis(p560.qn, qd) @ qd

# %% [markdown]
# and we see that it exerts a torque on the waist and elbow joints.
# 
# The algorithms to compute the various terms in the rigid-body equations of motion are based on the recursive Newton-Euler algorithm

# %%
p560.rne(p560.qn, np.zeros((6,)), np.zeros((6,)))

# %% [markdown]
# which computes $\tau = \mbox{rne}(\mathit{q}, \mathit{\dot{q}}, \mathit{\dot{q}})$ and can accept additional arguments such as gravity or a wrench applied to the end-effector.  In the Toolbox this algorithm is implemented in C for maximum performance.
# 
# The example above computes the gravity load, cross check it with the result computed earlier.
# 

# %% [markdown]
# We can rearrange the equations of motion as
# $$
# \mathit{\ddot{q}} = \mathbf{M}(\mathit{\ddot{q}})^{-1} \left\{ \mathit{\tau} - \mathbf{C}(\mathit{q}, \mathit{\dot{q}}) \mathit{\dot{q}} - \mathbf{g}(\mathit{q}) \right\} 
# $$
# which is the joint acceleration for a robot in the state $(\mathit{q}, \mathit{\dot{q}})$ with an applied torque of $\mathit{\tau}$.
# 
# If we integrate this we can compute the trajectory of the joint coordinates (and velocities) as a function of time

# %%
p560nf = p560.nofriction()
tg = p560nf.fdyn(5, p560.qn, dt=0.05)

# %% [markdown]
# The first line needs some explanation.  The Toolbox can model two types of joint friction:
# - viscous friction which is linearly related to joint velocity
# - Coulomb friction which is **non-linearly** related to joint velocity
# 
# Coulomb friction is a _harsh_ non-linearity and it causes the numerical integrator to take very small times steps, so the result will take many minutes to compute.  To speed things up, at the expense of some modeling fidelity, we set the Coulomb friction to zero, but retain the viscous friction.  The `nofriction()` method returns a clone of the robot with its friction parameters modified.
# 
# The computed joint configuration trajectory is

# %%
tg.q

# %% [markdown]
# which we can plot using a Toolbox convenience function

# %%
# rtb.tools.trajectory.qplot(tg.t, tg.q)
rtb.tools.plot.xplot(tg.t, tg.q)


# %% [markdown]
# or we can animate it, showing the robot collapsing under gravity

# %%
p560.plot(tg.q);

# %% [markdown]
# _Note: this animation is not working in Jupyter notebook..._
# 
# The motion of the robot quickly dies out and it hangs downward, this loss of energy is due to the viscous friction in the robot's joints.


