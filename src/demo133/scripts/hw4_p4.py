#!/usr/bin/env python3
#
#   trajectory_generator.py
#
#   Create a continuous stream of joint positions, to be animated.
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import math
import sys
import numpy as np

sys.path.append('~/133ws/src/demo133/scripts')

from sensor_msgs.msg   import JointState
import kinematics_skeleton as kinematics
from kinematics_skeleton import Kinematics
from urdf_parser_py.urdf import Robot

class CubicSpline:
    # Initialize.
    def __init__(self, p0, v0, pf, vf, T):
        # Precompute the spline parameters.
        self.T = T
        self.a = p0
        self.b = v0
        self.c =  3*(pf-p0)/T**2 - vf/T    - 2*v0/T
        self.d = -2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2

    # Report the segment's duration (time length).
    def duration(self):
        return(self.T)

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t +   self.c * t**2 +   self.d * t**3
        v =          self.b     + 2*self.c * t    + 3*self.d * t**2
        return (p,v)

class Goto(CubicSpline):
    # Use zero initial/final velocities (of same size as positions).
    def __init__(self, p0, pf, T):
        CubicSpline.__init__(self, p0, 0*p0, pf, 0*pf, T)

class Hold(Goto):
    # Use the same initial and final positions.
    def __init__(self, p, T):
        Goto.__init__(self, p, p, T)

class Stay(Hold):
    # Use an infinite time (stay forever).
    def __init__(self, p):
        Hold.__init__(self, p, math.inf)

#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        robot = Robot.from_parameter_server()
        self.pub = rospy.Publisher("/joint_states", JointState)
        self.count = 0
        self.kin = Kinematics(robot, 'world', 'tip')
        rospy.Subscriber("/joint_states", JointState, self.jointstatemsg)
        rospy.sleep(0.25)
        self.theta = [0, np.pi / 2, -np.pi / 2];  # initial guess
        x_goal = np.array([0.5, -1.0, 0.5]).reshape(3,1)  # goal
        theta_array = []
        self.segments = []
        self.index = 0
        self.t0 = 0
        
        # Newton-Ramphson
        theta = self.theta
        theta_array.append(np.array(theta))
        for i in range(7):
            (T,J) = self.kin.fkin(theta)
            theta = theta + (np.linalg.inv(J[0:3]) @ (x_goal - kinematics.p_from_T(T))).reshape(1,3)[0]
            theta_array.append(np.array(theta))
            
        for i in range(7):
            self.segments.append(Goto(theta_array[i], theta_array[i + 1], 1.0))
        
        
        
          
    def jointstatemsg(self, msg):       
        self.theta = msg.position


    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        msg = JointState()

        # Report.
        # np.set_printoptions(precision=6, suppress=True)
        # print("T:\n", T)
        # print("J:\n", J[0:3])
        # print("p/q: ", kinematics.p_from_T(T).T, kinematics.q_from_T(T))
        
        msg.name = ['theta1', 'theta2', 'theta3']

        if (t - self.t0 >= self.segments[self.index].duration()):
            self.t0    = self.t0 + self.segments[self.index].duration()
            self.index = (self.index+1) % len(self.segments)
        (msg.position, msg.velocity) = \
            self.segments[self.index].evaluate(t-self.t0)     
        
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('generator')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Update the controller.
        generator.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()
