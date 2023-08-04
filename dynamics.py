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
                 linkInertiaZX=0):
        self.previousLink = previousLink
        self.previousLinkJointPosition = np.array(previousLinkJointPosition).reshape((3, 1))
        self.previousLinkJointAxis = np.array(previousLinkJointAxis).reshape((3, 1))
        self.previousLinkJointType = previousLinkJointType

        self.linkMass = linkMass
        self.linkCoM = np.array(linkCoM).reshape((3, 1))
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


class Chain:
    def __init__(self):
        self.Links = []
        self.LinkPoints = []
        self.degreeOfFreedom = 0

    def appendLink(self, **kwargs):
        self.Links.append(Link(**kwargs))
        self.degreeOfFreedom += 1

    def appendLinkPoint(self, **kwargs):
        self.LinkPoints.append(LinkPoint(**kwargs))



    def calcCurrentPositions(self):
        # First calculate absolute positions of links
        for i in range(self.degreeOfFreedom):
            previousLink = self.Links[i].previousLink
            if previousLink == 0:
                self.Links[i].linkPositionWorld = self.Links[i].previousLinkJointPosition

            linkPosition = links[self.link].linkPositionWorld
            linkOrientation = links[self.link].linkOrientationWorld

        linkPosition = links[self.link].linkPositionWorld
        linkOrientation = links[self.link].linkOrientationWorld

        self.pointOrientationWorld = linkOrientation
        self.pointPositionWorld = linkPosition + linkOrientation @ self.pointPositionLink

def skewMatrix(vector):
    vector = np.reshape(vector, 3)
    matrix = np.array([[0, -vector[2], vector[1]],
                       [vector[2], 0, -vector[0]],
                       [-vector[1], vector[0], 0]])
    return matrix

def axis2rot(axis, angle):
    axis = np.reshape(axis, (3, 1))

    rot = axis @ np.transpose(axis) * (1-np.cos(angle)) + np.eye(3) * np.cos(angle) + skewMatrix(axis) * np.sin(angle)
    return rot
