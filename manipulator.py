import numpy as np
import moteus
import asyncio
import math
import time
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy.optimize import fsolve


class Manipulator:

    def __init__(self):
        self.c2 = moteus.Controller(id=2)
        self.c1 = moteus.Controller(id=1)
        self.startMotorPos = np.zeros([2, 1])
        self.currentMotorPos = np.zeros([2, 1])
        self.currentJointPos = np.zeros([2, 1])

        self.jacobianJointToMotor = np.array([[1, 0], [0.2, -1]]) / (2 * np.pi) * 20
        self.jacobianMotorToJoint = inv(self.jacobianJointToMotor)

        # Trajectory queue
        self.trajectoryTimeInterval = 0.01

        self.trajectoryMotorPosReal = np.zeros([2, 1])

        self.desCartesianTrajectory = self.calc_FK(self.currentJointPos)
        self.desJointTrajectoryPos = self.currentJointPos
        self.desJointTrajectoryVel = np.zeros([2, 1])
        self.desJointTrajectoryTorque = np.zeros([2, 1])

        # asyncio.run(self.connect())

    async def moteus_connect(self):

        await self.c1.set_stop()
        await self.c2.set_stop()

        state1 = await self.c1.set_position(position=math.nan, query=True)
        state2 = await self.c2.set_position(position=math.nan, query=True)

        self.startMotorPos[0] = state1.values[moteus.Register.POSITION]
        self.startMotorPos[1] = state2.values[moteus.Register.POSITION]

        print("Moteus connected")

    async def moteus_get_pos(self):
        state1 = await self.c1.set_position(position=math.nan, query=True)
        state2 = await self.c2.set_position(position=math.nan, query=True)

        self.currentMotorPos = np.array(
            [state1.values[moteus.Register.POSITION], state2.values[moteus.Register.POSITION]]).T
        self.currentJointPos = self.jacobianMotorToJoint @ self.currentMotorPos

    async def moteus_set_pos(self, desJointPos):

        desMotorPos = self.jacobianJointToMotor @ desJointPos + self.startMotorPos

        state1 = await self.c1.set_position(position=desMotorPos[0], query=True)
        state2 = await self.c2.set_position(position=desMotorPos[1], query=True)

        self.currentMotorPos = np.array(
            [state1.values[moteus.Register.POSITION], state2.values[moteus.Register.POSITION]]).T
        self.currentJointPos = self.jacobianMotorToJoint @ (self.currentMotorPos - self.startMotorPos)

    async def moteus_set_pos_feedforward(self, desJointPos, desJointVel, desJointTorque):
        desMotorPos = self.jacobianJointToMotor @ desJointPos + self.startMotorPos
        desMotorVel = self.jacobianJointToMotor @ desJointVel

        state1 = await self.c1.set_position(position=desMotorPos[0],
                                            velocity=desMotorVel[0],
                                            feedforward_torque=desJointTorque[0],
                                            query=True)
        state2 = await self.c1.set_position(position=desMotorPos[1],
                                            velocity=desJointVel[1],
                                            feedforward_torque=desJointTorque[1],
                                            query=True)

        self.currentMotorPos = np.array(
            [state1.values[moteus.Register.POSITION], state2.values[moteus.Register.POSITION]]).T
        self.currentJointPos = self.jacobianMotorToJoint @ (self.currentMotorPos - self.startMotorPos)

    async def moteus_play_trajectory(self):
        startTime = time.perf_counter_ns()
        timeInterval_ns = int(self.trajectoryTimeInterval * 1e9)
        n = np.size(self.desJointTrajectoryPos, 1)

        expected_time = range(startTime, startTime + n * timeInterval_ns, timeInterval_ns)

        for i in range(n):
            # Send desired joint positions to Moteus
            desJointTrajectoryPos = np.array([[self.desJointTrajectoryPos[0, i]], [self.desJointTrajectoryPos[1, i]]])
            await self.moteus_set_pos(desJointTrajectoryPos)

            # Wait
            expected_time_current = expected_time[i]
            while (time.perf_counter_ns() < expected_time_current):
                pass

        await self.c1.set_stop()
        await self.c2.set_stop()

    async def moteus_goto_joint(self, desJointPos, duration=2, dt=0.01):
        # Update current motor positions
        await self.moteus_get_pos()

        # Calculate PTP trajectory
        n = round(duration / dt)
        profile = self.calc_profile(n)

        desiredMotorPos = self.jacobianJointToMotor @ desJointPos
        self.trajectoryMotorPos = self.currentMotorPos + (desiredMotorPos - self.currentMotorPos) @ profile
        # self.trajectoryMotorVel = np.zeros(np.shape(self.trajectoryMotorPos))
        # self.trajectoryMotorTorque = np.zeros(np.shape(self.trajectoryMotorPos))

        await self.moteus_play_trajectory()

    def calc_trajectory_cartesian_lin(self, desCartesianPos, duration):
        # get the latest desired Cartesian position
        currentCartesianPos = self.desCartesianTrajectory[:, [-1]]

        # calculate smooth profile
        n = int(duration / self.trajectoryTimeInterval)
        profile = self.calc_profile(n)

        # calculate and append desired cartesian trajectory
        desCartesianTrajectory = (desCartesianPos - currentCartesianPos) @ profile + currentCartesianPos
        self.desCartesianTrajectory = np.append(self.desCartesianTrajectory, desCartesianTrajectory, 1)

        # calculate and append desired joint trajectory
        desJointTrajectoryPos = self.calc_IK(desCartesianTrajectory)
        self.desJointTrajectoryPos = np.append(self.desJointTrajectoryPos, desJointTrajectoryPos, 1)

    def calc_trajectory_cartesian_circle(self, centerPointPos, duration):
        # get the latest desired Cartesian position
        currentCartesianPos = self.desCartesianTrajectory[:, [-1]]
        dx = currentCartesianPos[0, 0] - centerPointPos[0, 0]
        dy = currentCartesianPos[1, 0] - centerPointPos[1, 0]
        r = np.array([[dx], [dy]])
        startAngle = np.arctan2(dy, dx)

        # calculate smooth profile
        n = int(duration / self.trajectoryTimeInterval)
        profile = 2 * np.pi * self.calc_profile(n)
        profile = profile[0]

        # calculate and append desired cartesian trajectory
        desCartesianTrajectory = np.zeros([2, n])
        for i in range(n):
            fi = profile[i]
            rot = np.array([[np.cos(fi), -np.sin(fi)], [np.sin(fi), np.cos(fi)]])
            desCartesianTrajectory[:, [i]] = rot @ r + centerPointPos

        self.desCartesianTrajectory = np.append(self.desCartesianTrajectory, desCartesianTrajectory, 1)

        # calculate and append desired joint trajectory
        desJointTrajectoryPos = self.calc_IK(desCartesianTrajectory)
        self.desJointTrajectoryPos = np.append(self.desJointTrajectoryPos, desJointTrajectoryPos, 1)

    def calc_trajectory_cartesian_circle2(self, centerPointPos, duration):
        # get the latest desired Cartesian position
        currentCartesianPos = self.desCartesianTrajectory[:, [-1]]
        dx = currentCartesianPos[0, 0] - centerPointPos[0, 0]
        dy = currentCartesianPos[1, 0] - centerPointPos[1, 0]
        r = np.array([[dx], [dy]])
        startAngle = np.arctan2(dy, dx)

        # calculate smooth profile
        n = int(duration / self.trajectoryTimeInterval)
        profile = 2 * 2 * np.pi * self.calc_profile(n)
        profile = profile[0]

        # calculate and append desired cartesian trajectory
        desCartesianTrajectory = np.zeros([2, n])
        for i in range(n):
            fi = profile[i]
            rot = np.array([[np.cos(fi), -np.sin(fi)], [np.sin(fi), np.cos(fi)]])
            desCartesianTrajectory[:, [i]] = rot @ r + centerPointPos

        self.desCartesianTrajectory = np.append(self.desCartesianTrajectory, desCartesianTrajectory, 1)

        # calculate and append desired joint trajectory
        desJointTrajectoryPos = self.calc_IK(desCartesianTrajectory)
        self.desJointTrajectoryPos = np.append(self.desJointTrajectoryPos, desJointTrajectoryPos, 1)

    def calc_FK(self, jointTrajectory):

        link1 = np.array([[-400], [60]])
        link2 = np.array([[400], [60]])
        jointTrajectory = np.reshape(jointTrajectory, (2, -1))
        endPos = np.zeros(np.shape(jointTrajectory))

        for i in range(np.size(jointTrajectory, 1)):
            q1 = jointTrajectory[0, i]
            q2 = jointTrajectory[1, i]

            rot1 = np.array([[np.cos(q1), -np.sin(q1)], [np.sin(q1), np.cos(q1)]])
            rot2 = np.array([[np.cos(q1 + q2), -np.sin(q1 + q2)], [np.sin(q1 + q2), np.cos(q1 + q2)]])

            endPos[:, [i]] = rot1 @ link1 + rot2 @ link2

        return endPos

    def calc_IK(self, cartesianTrajectory):

        a1 = np.sqrt(400 ** 2 + 60 ** 2)
        a2 = np.sqrt(400 ** 2 + 60 ** 2)
        th1 = np.arctan2(60, -400)
        th2 = np.arctan2(60, 400)

        angles = np.zeros(np.shape(cartesianTrajectory))

        for i in range(np.size(cartesianTrajectory, 1)):
            px = cartesianTrajectory[0, i]
            py = cartesianTrajectory[1, i]

            # Circle intersections
            # https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
            d = np.sqrt(px ** 2 + py ** 2)

            a = (a1 ** 2 - a2 ** 2 + d ** 2) / (2 * d)
            h = np.sqrt(a1 ** 2 - a ** 2)
            x2 = a * px / d
            y2 = a * py / d
            x3 = x2 + h * py / d
            y3 = y2 - h * px / d

            x4 = x2 - h * py / d
            y4 = y2 + h * px / d

            wekt = -py * x3 + px * y3

            if wekt > 0:
                cx = x3
                cy = y3
            else:
                cx = x4
                cy = y4

            fi1 = np.arctan2(cy, cx) - th1
            fi2 = np.arctan2(py - cy, px - cx) - fi1 - th2

            angles[0, i] = fi1
            angles[1, i] = fi2

        return angles


    def calc_profile(self, n):
        profile = np.zeros([1, n])
        x = np.linspace(0, 1, n)

        profile[0, :] = -2 * x[:] ** 3 + 3 * x[:] ** 2
        return profile

    def plot_robot(self):
        fig, ax = plt.subplots()

        ax.axis([-600, 600, -200, 1000])
        plt.ion()
        plt.show()

        for i in range(np.size(self.desJointTrajectoryPos, 1)):
            q1 = self.desJointTrajectoryPos[0, i]
            q2 = self.desJointTrajectoryPos[1, i]
            rot1 = np.array([[np.cos(q1), -np.sin(q1)], [np.sin(q1), np.cos(q1)]])
            rot2 = np.array([[np.cos(q1 + q2), -np.sin(q1 + q2)], [np.sin(q1 + q2), np.cos(q1 + q2)]])

            link1 = np.array([[0, -400, -400], [0, 0, 60]])
            link2 = np.array([[0, 400, 400], [0, 0, 60]])

            link1new = rot1 @ link1
            link2new = rot2 @ link2 + link1new[:, [-1]]

            plt.cla()
            ax.plot(link1new[0, :], link1new[1, :])
            ax.plot(link2new[0, :], link2new[1, :])
            ax.plot(0, 0, 'ro')
            ax.plot(link1new[0, -1], link1new[1, -1], 'ro')
            ax.plot(link2new[0, -1], link2new[1, -1], 'ro')

            ax.axis([-600, 600, -200, 1000])
            # ax.axis('equal')
            ax.grid('on')

            plt.draw()
            plt.pause(0.0001)
