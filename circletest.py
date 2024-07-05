import asyncio
import math
import moteus
import numpy as np
import time
import matplotlib.pyplot as plt
from manipulator import Manipulator as man

arm = man()

arm.calc_trajectory_cartesian_lin(np.array([[0.150], [0.300]]), duration=3)
arm.calc_trajectory_cartesian_circle2(np.array([[0.150], [0.450]]),duration=8)
arm.calc_trajectory_cartesian_circle2(np.array([[0.150], [0.450]]),duration=6)
#arm.calc_trajectory_cartesian_circle2(np.array([[0.150], [0.450]]),duration=4)
#arm.calc_trajectory_cartesian_circle2(np.array([[0.150], [0.450]]),duration=3)
# arm.calc_trajectory_cartesian_circle(np.array([[0.150], [0.450]]),duration=1)
arm.calc_trajectory_cartesian_lin(np.array([[0.0], [0.120]]), duration=5)


#asyncio.run(arm.moteus_play_trajectory())
arm.plot_robot()
