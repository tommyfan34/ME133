#!/usr/bin/env python3
#
#   better_trajectory_generator.py
#
#   Create a continuous stream of joint positions, to be animated.
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import math
import numpy as np

from sensor_msgs.msg   import JointState


#
#  Trajectory Segment Objects
#
#  All these segments are set up to start at time 0, running to some
#  specified end time.  They each store whichever internal parameters
#  they need.  And provide both an evaluate(t) and duration() method.
#  Using numpy, they work with arrays of joints.
#
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
        self.pub = rospy.Publisher("/joint_states", JointState)
        rospy.sleep(0.25)

        # Prepare a list of trajectory segments.
        ##### CHANGE THESE #####
        pA = np.array([ 0.7854, -1.0472, 0.7854, 0.5236 ])

        

        self.segments = (Hold(pA, 10),
        		  Stay(pA))

        # Initialize the current segment index and starting time t0.
        self.index = 0
        self.t0    = 0.0
        self.splines = 4

    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()

        # Use the same names as in the URDF!!!
        ##### CHANGE THESE #####
        cmdmsg.name = ['theta1', 'theta2', 'theta3', 'theta4']

        # If the current segment is done, shift to the next.
        if (t - self.t0 >= self.segments[self.index].duration()):
            self.t0    = self.t0 + self.segments[self.index].duration()
            self.index = (self.index+1)
            # If the list were cyclic, you could go back to the start with
            self.index = (self.index+1) % self.splines

        # Set the message positions/velocities as a function of time.
        (cmdmsg.position, cmdmsg.velocity) = \
            self.segments[self.index].evaluate(t-self.t0)

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)


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

