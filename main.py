import dynamics
import numpy as np

chain=dynamics.Chain()

# First link
chain.appendLink(previousLink=0,
                 previousLinkJointPosition=[0, 0, 0],
                 previousLinkJointAxis=[0, 0, 1],
                 previousLinkJointType='revolute',
                 linkMass=2,
                 linkCoM=[0, 0, 0],
                 linkInertiaXX=0,
                 linkInertiaYY=0,
                 linkInertiaZZ=0,
                 linkInertiaXY=0,
                 linkInertiaYZ=0,
                 linkInertiaZX=0)
# Second link
chain.appendLink(previousLink=1,
                 previousLinkJointPosition=[0, 0, 0],
                 previousLinkJointAxis=[0, 0, 1],
                 previousLinkJointType='revolute',
                 linkMass=2,
                 linkCoM=[0, 0, 0],
                 linkInertiaXX=0,
                 linkInertiaYY=0,
                 linkInertiaZZ=0,
                 linkInertiaXY=0,
                 linkInertiaYZ=0,
                 linkInertiaZX=0)
axis = np.array([[0],[0],[1]])

rot = dynamics.axis2rot(axis = axis, angle= np.pi/2)
print(np.round(rot,2))
