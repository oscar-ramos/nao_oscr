# from oscr.sim_ros_robot import SimRosRobot
# from oscr.sim_ros_utils import JointStatePub
import numpy as np
from copy import copy

import rospy
from sensor_msgs.msg import JointState
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

from oscr.ros_robot import RosRobot


class NaoRosRobot(RosRobot):
    """
    Helper class for the NAO robot in ROS (and RViz)

    """

    def __init__(self, node_name, freq, fbase, backend):
        """
        Constructor. It sets the reduced NAO model (26 dofs) and adds specific
        NAO properties to the joint state publisher

        """
        pkg = 'nao_oscr'
        # Reduced NAO model with 26 dofs
        robot_model = '/urdf/naoV40red.urdf'
        RosRobot.__init__(self, node_name, pkg, robot_model,
                          freq, fbase, backend)
        # Add joint names for the fingers
        finger_names = ('LFinger11','LFinger12','LFinger13','LFinger21',
                        'LFinger22','LFinger23','LThumb1','LThumb2',
                        'RFinger11','RFinger12','RFinger13','RFinger21',
                        'RFinger22','RFinger23','RThumb1','RThumb2')
        self.joint_pub.appendJointNames(finger_names)
        # Set finger values to zero
        self.qextras = np.array([[0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0]])
        self.qextras = self.qextras.T



# class NaoRosRobot(SimRosRobot):
#     """
#     Helper class for interfacing with the NAO robot through naoqi

#     """
#     def __init__(self, node_name, freq, fbase, backend):
#         """
#         Constructor. It sets the reduced NAO model (26 dofs) and instanciates
#         a robot_interface which will read the sensors and send the commands
#         through naoqi

#         """
#         pkg = 'nao_oscr'
#         # Reduced NAO model with 26 dofs
#         robot_model = '/urdf/naoV40red.urdf'
#         SimRosRobot.__init__(self, node_name, pkg, robot_model,
#                              freq, fbase, backend)
#         self.robot_interface = RobotInterface(self.robot.ndofActuated,
#                                               self.robot.jointNames)

#     def publishJointState(self, q):
#         """
#         Avoid publishing to joint_states (since naoqi publishes there)
#         """
#         pass

#     def getMeasuredJointPositions(self):
#         """
#         Get the current measured joint positions from the robot (through naoqi)
#         """
#         return np.array([self.robot_interface.getMeasuredJointPositions()]).T

#     def setJointPositionsCmd(self, q):
#         """
#         Send position commands to the robot through naoqi. The robot is
#         position controlled

#         """
#         self.robot_interface.setJointPositionsCmd(q)
#         # Show current position and/or orientation
#         for i in self.task:
#             current = self.task[i].getCurrentValue()
#             self.task[i].marker[0].setPose(current)
#             self.task[i].marker[1].publish()
#         self.rate.sleep()



# class RobotInterface(object):
#     """
#     Helper class that acts as a bridge between naoqi and oscr interface

#     """
#     def __init__(self, ndof_actuated, jnames):
#         """
#         Constructor. Set the publisher and subscriber

#         """
#         self.ndof = ndof_actuated
#         # Publisher to send the joint commands
#         self.pub = rospy.Publisher("joint_angles", JointAnglesWithSpeed,
#                                    queue_size=1000)
#         # Subscriber to read the measured joint values
#         self.sub = rospy.Subscriber("joint_states", JointState,
#                                     self.callbackMeas)
#         # Message for the joint command
#         self.jcmd_msg = JointAnglesWithSpeed()
#         self.jcmd_msg.speed = 0.2
#         self.jcmd_msg.joint_names = jnames
#         self.jsensed_msg = JointState()

#     def callbackMeas(self, msg):
#         """
#         Callback for the measured value from the robot joint sensors
#         """
#         self.jsensed_msg = msg

#     def getMeasuredJointPositions(self):
#         """
#         Read the joint sensors of the robot through naoqi (read from the
#         joint_sensors topic)

#         """
#         iter_rate = rospy.Rate(1000)
#         ndof_sensed = len(self.jsensed_msg.position)
#         niter=0
#         max_iter = 1e3
#         print ndof_sensed
#         print self.ndof
#         while (ndof_sensed != self.ndof):
#             if (niter == max_iter):
#                 print "Sensed joint configuration does not have ", ndof, \
#                     " degrees of freedom, stopping ..."
#                 return
#             ndof_sensed = len(self.jsensed_msg.position)
#             niter = niter+1
#             iter_rate.sleep()
#         qsensed = list(copy(self.jsensed_msg.position))
#         naoqi2oscr = (8, 9, 10, 11, 12, 13, 2, 3, 4, 5, 6, 7)
#         for i in xrange(12):
#             qsensed[naoqi2oscr[i]] = self.jsensed_msg.position[i+2]
#         return qsensed


#     def setJointPositionsCmd(self, q):
#         """
#         Send the position commands to the robot through naoqi (publish to the
#         joint_angles topic)

#         """
#         self.jcmd_msg.joint_angles = q
#         self.jcmd_msg.header.stamp = rospy.Time.now()
#         self.pub.publish(self.jcmd_msg)
