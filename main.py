import dynamics
import numpy as np
from manipulator import Manipulator as man
import matplotlib.pyplot as plt

arm = man()

chain=dynamics.Chain()

# First link
chain.appendLink(prevLink=-1,
                 prevLinkJointPosition=[0, 0, 0],
                 prevLinkJointAxis=[0, 0, 1],
                 prevLinkJointType='revolute',
                 linkMass=2,
                 linkCoM=[-200, 30, 0],
                 linkInertiaXX=0,
                 linkInertiaYY=0,
                 linkInertiaZZ=0,
                 linkInertiaXY=0,
                 linkInertiaYZ=0,
                 linkInertiaZX=0)
# Second link
chain.appendLink(prevLink=0,
                 prevLinkJointPosition=[-400, 60, 0],
                 prevLinkJointAxis=[0, 0, 1],
                 prevLinkJointType='revolute',
                 linkMass=2,
                 linkCoM=[200, 30, 0],
                 linkInertiaXX=0,
                 linkInertiaYY=0,
                 linkInertiaZZ=0,
                 linkInertiaXY=0,
                 linkInertiaYZ=0,
                 linkInertiaZX=0)

chain.appendLinkPoint(link=0,pointPositionLink=chain.Links[0].linkCoM,DoF=2)
chain.appendLinkPoint(link=1,pointPositionLink=chain.Links[1].linkCoM,DoF=2)
chain.appendLinkPoint(link=1, pointPositionLink=[400, 60, 0], DoF=2)

t = np.linspace(0,np.pi,1000)
Q = np.array([np.sin(t),np.sin(2*t)])
pos1 = arm.calc_FK(jointTrajectory=Q)
pos2 = pos1*0

for i in range(np.size(Q,1)):
    q = Q[:,i]
    chain.calcCurrentPositions(q)
    pos = chain.Points[0].pointPositionWorld
    pos2[:,i] = pos[0:2,0]

delta = np.abs(pos1-pos2)
print(np.max(delta))

desJointTorque = chain.calcInverseDynamics(Q)
desJointTorque[:,[-1]] = np.zeros((2,1))
desJointTorque[:,[-2]] = np.zeros((2,1))

plt.plot(np.transpose(desJointTorque))
plt.show()

# plt.plot(np.transpose(delta))
# plt.show()

# chain.calcCurrentPositions(np.zeros((2,1)))
# print(chain.Points[0].jacobianMatrix)


