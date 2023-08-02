import numpy as np


class Link:
    def __init__(self,
                 previousLink=0,
                 previousLinkJointPosition=[0, 0, 0],
                 previousLinkJointAxis=[0, 0, 0],
                 previousLinkJointType='revolute',
                 linkMass=0,
                 linkCoM=[0, 0, 0],
                 linkInertiaXX=0,
                 linkInertiaYY=0,
                 linkInertiaZZ=0,
                 linkInertiaXY=0,
                 linkInertiaYZ=0,
                 linkInertiaZX=0, ):
        self.previousLink = previousLink
        self.previousLinkJointPosition = np.array(previousLinkJointPosition).reshape((3, 1))
        self.previousLinkJointAxis = np.array(previousLinkJointAxis).reshape((3, 1))
        self.previousLinkJointType = previousLinkJointType

        self.linkMass = linkMass
        self.linkCoM = np.array(linkCoM).reshape((3, 1))
        # self.linkInertiaXX =linkInertiaXX
        # self.linkInertiaYY =linkInertiaYY
        # self.linkInertiaZZ =linkInertiaZZ
        # self.linkInertiaXY =linkInertiaXY
        # self.linkInertiaYZ =linkInertiaYZ
        # self.linkInertiaZX =linkInertiaZX
        self.linkInertiaMatrix = np.array([[linkInertiaXX, linkInertiaXY, linkInertiaZX],
                                           [linkInertiaXY, linkInertiaYY, linkInertiaYZ],
                                           [linkInertiaZX, linkInertiaYZ, linkInertiaZZ]])

        self.jointValue = 0
        self.linkPositionWorld = np.zeros((3, 1))
        self.linkOrientationWorld = np.eye(3)


class LinkPoint:
    def __init__(self, link=0, pointPositionLink=[0, 0, 0]):
        self.link = link
        self.pointPositionLink = np.array(pointPositionLink).reshape((3, 1))
        self.pointPositionWorld = np.zeros([3, 1])
        self.pointOrientationWorld = np.zeros([3, 3])
        self.jacobianColumns = []

    def calcCurrentPosition(self, links):
        linkPosition = links[self.link].linkPositionWorld
        linkOrientation = links[self.link].linkOrientationWorld

        self.pointOrientationWorld = linkOrientation
        self.pointPositionWorld = linkPosition + linkOrientation @ self.pointPositionLink
