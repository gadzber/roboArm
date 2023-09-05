# Import Libraries
import dynamics
import numpy as np
from manipulator import Manipulator as man
import asyncio
import matplotlib.pyplot as plt

# Initialize robot and dynamics
arm = man()
asyncio.run(arm.moteus_connect())
chain = dynamics.Chain()

# First link
chain.appendLink(prevLink=-1,
                 prevLinkJointPosition=[0, 0, 0],
                 prevLinkJointAxis=[0, 0, 1],
                 prevLinkJointType='revolute',
                 linkMass=0.727,
                 linkCoM=[-0.208, 0.028, 0],
                 linkInertiaZZ=1.467E-02)
# Second link
chain.appendLink(prevLink=0,
                 prevLinkJointPosition=[-0.400, 0.060, 0],
                 prevLinkJointAxis=[0, 0, 1],
                 prevLinkJointType='revolute',
                 linkMass=0.603,
                 linkCoM=[-0.213 + 0.400, 0.09 - 0.06, 0],
                 linkInertiaZZ=1.243E-02)

chain.appendLinkPoint(link=0, pointPositionLink=chain.Links[0].linkCoM, DoF=2)
chain.appendLinkPoint(link=1, pointPositionLink=chain.Links[1].linkCoM, DoF=2)
chain.appendLinkPoint(link=1, pointPositionLink=[0.400, 0.060, 0], DoF=2)

# PLanning Cartesian trajectory
arm.calc_trajectory_cartesian_lin(np.array([[0.0], [0.120]]), duration=0.1)
arm.calc_trajectory_cartesian_lin(np.array([[0.150], [0.300]]), duration=2)
arm.calc_trajectory_cartesian_circle(np.array([[0.150], [0.450]]), duration=4)
arm.calc_trajectory_cartesian_lin(np.array([[0], [0.120]]), duration=2)

# Calculate Joint torques
Q = arm.desJointTrajectoryPos
desJointTorque, desMotorTorque = chain.calcInverseDynamics(Q)
arm.desJointTrajectoryTorque = desJointTorque
arm.desMotorTrajectoryTorque = desMotorTorque

# Postprocessing
# arm.plot_robot()
# plt.plot(np.transpose(desJointTorque))
# plt.show()
asyncio.run(arm.moteus_play_trajectory())

plt.plot(np.rad2deg(np.transpose(Q) - arm.currentJointPos.T))
# plt.plot(arm.desMotorTrajectoryTorque.T)
plt.show()
