# amm-jenga-play
Jenga play with Franka Reasearch 3 [SKKU 2023 URP Team2's Project]

## Common Error

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