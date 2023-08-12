import numpy as np


class Link:
    def __init__(self,
                 prevLink=0,
                 prevLinkJointPosition=[0, 0, 0],
                 prevLinkJointAxis=[0, 0, 0],
                 prevLinkJointType='revolute',
                 linkMass=0,
                 linkCoM=[0, 0, 0],
                 linkInertiaXX=0,
                 linkInertiaYY=0,
                 linkInertiaZZ=0,
                 linkInertiaXY=0,
                 linkInertiaYZ=0,
                 linkInertiaZX=0):
        self.prevLink = prevLink
        self.prevLinkJointPosition = np.array(prevLinkJointPosition).reshape((3, 1))
        self.prevLinkJointAxis = np.array(prevLinkJointAxis).reshape((3, 1))
        self.prevLinkJointType = prevLinkJointType

        self.linkMass = linkMass
        self.linkCoM = np.array(linkCoM).reshape((3, 1))
        self.linkInertiaMatrix = np.array([[linkInertiaXX, linkInertiaXY, linkInertiaZX],
                                           [linkInertiaXY, linkInertiaYY, linkInertiaYZ],
                                           [linkInertiaZX, linkInertiaYZ, linkInertiaZZ]])

        self.jointAngle = 0
        self.linkPositionWorld = np.zeros((3, 1))
        self.linkOrientationWorld = np.eye(3)


class Point:
    def __init__(self, link=0, pointPositionLink=[0, 0, 0], DoF=1):
        self.link = link
        self.pointPositionLink = np.array(pointPositionLink).reshape((3, 1))
        self.pointPositionWorld = np.zeros([3, 1])
        # self.pointOrientationWorld = np.zeros([3, 3])
        self.jacobianMatrix = np.zeros([6, DoF])


class Chain:
    def __init__(self):
        self.Links = []
        self.Points = []
        self.DoF = 0

    def appendLink(self, **kwargs):
        self.Links.append(Link(**kwargs))
        self.DoF += 1

    def appendLinkPoint(self, **kwargs):
        self.Points.append(Point(**kwargs))

    def calcCurrentPositions(self, jointPos):
        # First:
        for i in range(self.DoF):
            self.Links[i].jointAngle = jointPos[i]

        # Second: calculate absolute positions of links
        for i in range(self.DoF):
            prev = self.Links[i].prevLink
            if prev == -1:
                self.Links[i].linkPositionWorld = self.Links[i].prevLinkJointPosition
                self.Links[i].linkOrientationWorld = axis2rot(self.Links[i].prevLinkJointAxis,
                                                              self.Links[i].jointAngle)
            else:
                self.Links[i].linkPositionWorld = self.Links[prev].linkPositionWorld + self.Links[
                    prev].linkOrientationWorld @ self.Links[i].prevLinkJointPosition
                axis = self.Links[prev].linkOrientationWorld @ self.Links[i].prevLinkJointAxis
                self.Links[i].linkOrientationWorld = self.Links[prev].linkOrientationWorld @ axis2rot(axis, self.Links[
                    i].jointAngle)

        # Third: calculate absolute positions of link points
        for i in range(len(self.Points)):
            link = self.Points[i].link
            self.Points[i].pointPositionWorld = self.Links[link].linkPositionWorld + self.Links[
                link].linkOrientationWorld @ self.Points[i].pointPositionLink

    def calcCurrentJacobians(self):
        # Calculate jacobians
        for point in range(len(self.Points)):
            for joint in range(self.DoF):

                link = self.Points[point].link
                if joint <= link:
                    d = self.Points[point].pointPositionWorld - self.Links[joint].linkPositionWorld
                    u = self.Links[link].linkOrientationWorld @ self.Links[link].prevLinkJointAxis

                    jacobianColumnPos = skewMatrix(u) @ d
                    jacobianColumn = np.block([[jacobianColumnPos], [u]])
                    jacobianColumn.reshape((6, 1))
                    self.Points[point].jacobianMatrix[:, [joint]] = jacobianColumn

    def calcInverseDynamics(self, desJointTrajectoryPos):
        N = np.size(desJointTrajectoryPos, 1)
        numOfPoints = len(self.Points)

        desCartesianTrajectoryPos = np.zeros((3 * numOfPoints, N))
        desCartesianTrajectoryVel = np.zeros((6 * numOfPoints, N))
        desCartesianTrajectoryAcc = np.zeros((6 * numOfPoints, N))

        desJointTorque = np.zeros((self.DoF, N))

        # First: calculate first order differential of joint positions
        desJointTrajectoryVel = np.diff(desJointTrajectoryPos, n=1, axis=1, append=np.zeros((self.DoF, 2)))

        # Second: calculate Forward Kinematics, jacobian and cartesian velocities
        for i in range(N):
            jointPos = desJointTrajectoryPos[:, [i]]
            jointVel = desJointTrajectoryVel[:, [i]]
            self.calcCurrentPositions(jointPos)
            self.calcCurrentJacobians()
            jacobian = np.zeros((6 * numOfPoints, self.DoF))

            for j in range(numOfPoints):
                desCartesianTrajectoryPos[3 * j:3 * j + 3, [i]] = self.Points[j].pointPositionWorld
                jacobian[6 * j:6 * j + 6, :] = self.Points[j].jacobianMatrix

            desCartesianTrajectoryVel[:, [i]] = jacobian @ jointVel


        # Third: numerically different velocities to get  and accelerations
        desCartesianTrajectoryAcc = np.diff(desCartesianTrajectoryVel, n=1, axis=1, append=np.zeros((6*numOfPoints, 1)))

        # Fourth: calculate joint torques
        grav = np.array([[0], [0], [-9.81]])
        for i in range(N):
            cartesianForces = np.zeros((6 * numOfPoints, 1))
            jacobian =np.zeros((6 * numOfPoints, self.DoF))

            # calculate current inertial and gravity forces
            for j in range(numOfPoints - 1):
                link = self.Points[j].link
                mass = self.Links[link].linkMass
                inertiaMatrix = self.Links[link].linkInertiaMatrix

                accLin = desCartesianTrajectoryAcc[6 * j:6 * j + 3, [i]]
                accAng = desCartesianTrajectoryAcc[6 * j + 3:6 * j + 6, [i]]
                velAng = desCartesianTrajectoryVel[6 * j + 3:6 * j + 6, [i]]

                cartesianForces[6 * j:6 * j + 3, [0]] = mass * (accLin + grav)
                cartesianForces[(6 * j + 3):(6 * j + 6), [0]] = inertiaMatrix @ accAng + skewMatrix(velAng) @ inertiaMatrix @ velAng

            # todo: add external forces

            # calculate superJacobian(TM) xD
            jointPos = desJointTrajectoryPos[:, [i]]
            self.calcCurrentPositions(jointPos)
            self.calcCurrentJacobians()

            for j in range(numOfPoints):
                jacobian[6 * j:6 * j + 6, :] = self.Points[j].jacobianMatrix

            # calculate transposed jacobian and forces product
            desJointTorque[:,[i]] = jacobian.T @ cartesianForces

        return desJointTorque



def skewMatrix(vector):
    vector = np.reshape(vector, 3)
    matrix = np.array([[0, -vector[2], vector[1]],
                       [vector[2], 0, -vector[0]],
                       [-vector[1], vector[0], 0]])
    return matrix


def axis2rot(axis, angle):
    axis = np.reshape(axis, (3, 1))

    rot = axis @ np.transpose(axis) * (1 - np.cos(angle)) + np.eye(3) * np.cos(angle) + skewMatrix(axis) * np.sin(angle)
    return rot
