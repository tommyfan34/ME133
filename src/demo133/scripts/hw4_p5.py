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
        self.kin = Kinematics(robot, 'world', 'tip')
        rospy.Subscriber("/joint_states", JointState, self.jointstatemsg)
        rospy.sleep(0.25)
        self.theta = [0, np.pi / 2, -np.pi / 2];  # initial guess
        
        pA = np.array([0.7, 0.7, 0.0])
        pB = np.array([-0.7, 0.7, 1.5])
        thetaA = self.kin.ikin(np.array(pA).reshape(3, 1), self.theta)
        thetaB = self.kin.ikin(np.array(pB).reshape(3, 1), self.theta)
        
        self.segments = [Goto(pA, pB, 2),
                         Goto(thetaB, thetaA, 2)]
        self.index = 0
        self.t0 = 0
          
    def jointstatemsg(self, msg):       
        self.theta = msg.position


    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        msg = JointState()

        (T,J) = self.kin.fkin(self.theta)

        # Report.
        # np.set_printoptions(precision=6, suppress=True)
        # print("T:\n", T)
        # print("J:\n", J[0:3])
        # print("p/q: ", kinematics.p_from_T(T).T, kinematics.q_from_T(T))
        
        msg.name = ['theta1', 'theta2', 'theta3']
        
        if (t - self.t0 >= self.segments[self.index].duration()):
            self.t0    = self.t0 + self.segments[self.index].duration()
            self.index = (self.index+1) % len(self.segments)
            
        # upward leg    
        if self.index == 0:    
            (x_position, x_velocity) = \
                self.segments[self.index].evaluate(t-self.t0) 
        
            theta_position = self.kin.ikin(np.array(x_position).reshape(3, 1), self.theta)
            theta_velocity = (np.linalg.inv(J[0:3]) @ (x_velocity)).reshape(1, 3)[0]
        
            msg.position = theta_position
            msg.velocity = theta_velocity
            
        # downward leg
        elif self.index == 1:
            (theta_position, theta_velocity) = \
                self.segments[self.index].evaluate(t-self.t0)
                
            msg.position = theta_position
            msg.velocity = theta_velocity
            
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
