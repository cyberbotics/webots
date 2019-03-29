"""Joint state publisher."""

import actionlib
import copy
import math
import rospy
import time

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def reorder_traj_joints(traj, joint_names):
    """Reorder the JointTrajectory traj according to the order in joint_names."""
    order = [traj.joint_names.index(j) for j in joint_names]
    new_points = []
    for p in traj.points:
        new_points.append(JointTrajectoryPoint(
            positions=[p.positions[i] for i in order],
            velocities=[p.velocities[i] for i in order] if p.velocities else [],
            accelerations=[p.accelerations[i] for i in order] if p.accelerations else [],
            time_from_start=p.time_from_start))
    traj.joint_names = joint_names
    traj.points = new_points


def within_tolerance(a_vec, b_vec, tol_vec):
    """Check if two vectors are equals with a given tolerance."""
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True


def interp_cubic(p0, p1, t_abs):
    """Perform a cubic interpolation between two trajectory points."""
    T = (p1.time_from_start - p0.time_from_start).to_sec()
    t = t_abs - p0.time_from_start.to_sec()
    q = [0] * 6
    qdot = [0] * 6
    qddot = [0] * 6
    for i in range(len(p0.positions)):
        a = p0.positions[i]
        b = p0.velocities[i]
        c = (-3 * p0.positions[i] + 3 * p1.positions[i] - 2 * T * p0.velocities[i] - T * p1.velocities[i]) / T**2
        d = (2 * p0.positions[i] - 2 * p1.positions[i] + T * p0.velocities[i] + T * p1.velocities[i]) / T**3

        q[i] = a + b * t + c * t**2 + d * t**3
        qdot[i] = b + 2 * c * t + 3 * d * t**2
        qddot[i] = 2 * c + 6 * d * t
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))


def sample_traj(traj, t):
    """Return (q, qdot, qddot) for sampling the JointTrajectory at time t, the time t is the time since the trajectory was started."""
    # First point
    if t <= 0.0:
        return copy.deepcopy(traj.points[0])
    # Last point
    if t >= traj.points[-1].time_from_start.to_sec():
        return copy.deepcopy(traj.points[-1])

    # Finds the (middle) segment containing t
    i = 0
    while traj.points[i + 1].time_from_start.to_sec() < t:
        i += 1
    return interp_cubic(traj.points[i], traj.points[i + 1], t)


class TrajectoryFollower(object):
    """Create and handle the action 'follow_joint_trajectory' server."""

    jointNames = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    def __init__(self, robot, jointStatePublisher, goal_time_tolerance=None):
        self.robot = robot
        self.jointStatePublisher = jointStatePublisher
        self.timestep = int(robot.getBasicTimeStep())
        self.motors = []
        self.sensors = []
        for name in TrajectoryFollower.jointNames:
            self.motors.append(robot.getMotor(name))
            self.sensors.append(robot.getPositionSensor(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
        self.goal_handle = None
        self.last_point_sent = True
        self.traj = None
        self.joint_goal_tolerances = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.server = actionlib.ActionServer("follow_joint_trajectory",
                                             FollowJointTrajectoryAction,
                                             self.on_goal, self.on_cancel, auto_start=False)

    def init_traj(self):
        """Initialize a new target trajectory."""
        state = self.jointStatePublisher.last_joint_states
        self.traj_t0 = time.time()  #TODO: Webots time
        self.traj = JointTrajectory()
        self.traj.joint_names = TrajectoryFollower.jointNames
        self.traj.points = [JointTrajectoryPoint(
            positions=state.position if state else [0] * 6,
            velocities=[0] * 6,
            accelerations=[0] * 6,
            time_from_start=rospy.Duration(0.0))]

    def start(self):
        """Initialize and start the action server."""
        self.init_traj()
        self.server.start()
        print("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        print("on_goal")

        # Checks if the joints are just incorrect
        if set(goal_handle.get_goal().trajectory.joint_names) != set(TrajectoryFollower.jointNames):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" %
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        # if not traj_is_finite(goal_handle.get_goal().trajectory):
        #     rospy.logerr("Received a goal with infinites or NaNs")
        #     goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
        #     return
        #
        # # Checks that the trajectory has velocities
        # if not has_velocities(goal_handle.get_goal().trajectory):
        #     rospy.logerr("Received a goal without velocities")
        #     goal_handle.set_rejected(text="Received a goal without velocities")
        #     return
        #
        # # Checks that the velocities are withing the specified limits
        # if not has_limited_velocities(goal_handle.get_goal().trajectory):
        #     message = "Received a goal with velocities that are higher than %f" % max_velocity
        #     rospy.logerr(message)
        #     goal_handle.set_rejected(text=message)
        #     return

        # Orders the joints of the trajectory according to joint_names
        reorder_traj_joints(goal_handle.get_goal().trajectory, TrajectoryFollower.jointNames)

        #TODO Inserts the current setpoint at the head of the trajectory
        self.traj_t0 = time.time()
        self.goal_handle = goal_handle
        self.traj = goal_handle.get_goal().trajectory
        goal_handle.set_accepted()

    def on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        print("on_cancel")

        if goal_handle == self.goal_handle:
            # stop the motors
            for i in range(len(TrajectoryFollower.jointNames)):
                self.motors[i].setPosition(self.sensors[i].getValue())
            self.goal_handle.set_canceled()
            self.goal_handle = None
        else:
            goal_handle.set_canceled()

    def update(self):
        if self.robot and self.traj:
            now = time.time()
            if (now - self.traj_t0) <= self.traj.points[-1].time_from_start.to_sec():
                self.last_point_sent = False  # Sending intermediate points
                setpoint = sample_traj(self.traj, now - self.traj_t0)
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
            elif not self.last_point_sent:
                # All intermediate points sent, sending last point to make sure we reach the goal.
                last_point = self.traj.points[-1]
                state = self.jointStatePublisher.last_joint_states
                position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                # Performing this check to try and catch our error condition.  We will always
                # send the last point just in case.
                if not position_in_tol:
                    rospy.logwarn("Trajectory time exceeded and current robot state not at goal, last point required")
                    rospy.logwarn("Current trajectory time: %s, last point time: %s" % (now - self.traj_t0, self.traj.points[-1].time_from_start.to_sec()))
                    rospy.logwarn("Desired: %s\nactual: %s\nvelocity: %s" % (last_point.positions, state.position, state.velocity))
                setpoint = sample_traj(self.traj, self.traj.points[-1].time_from_start.to_sec())
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
            else:  # Off the end
                if self.goal_handle:
                    last_point = self.traj.points[-1]
                    state = self.jointStatePublisher.last_joint_states
                    position_in_tol = within_tolerance(state.position, last_point.positions, [0.1] * 6)
                    velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05] * 6)
                    if position_in_tol and velocity_in_tol:
                        # The arm reached the goal (and isn't moving).  Succeeding
                        self.goal_handle.set_succeeded()
                        self.goal_handle = None
