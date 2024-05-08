#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class SM2PX4(object):
    def __init__(self) -> None:
        rospy.init_node('state_machine_to_px4', anonymous=True)
        rospy.on_shutdown(self.shutdownCb)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCb)
        rospy.Subscriber("mavros/state", State, Cb=self.mavrosStateCb)
        rospy.Subscriber("/sm/pose", PoseStamped, Cb=self.poseCb)
        rospy.Service("land", Empty, self.landCb)
        self.pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.state = State()
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1

    def mavrosStateCb(self, msg):
        self.state = msg

    def poseCb(self, msg):
        self.pose = msg

    def timerCb(self, event=None):
        self.pos_pub.publish(self.pose)

    def landCb(self, req):
        set_mode = SetModeRequest()
        set_mode.custom_mode = 'AUTO.LAND'
        if self.set_mode_client.call(set_mode).mode_sent == True:
            rospy.logwarn("Drone auto land.")
        self.timer.shutdown()

    def shutdownCb(self):
        rospy.loginfo("SM2PX4 shutting down...")
        # set_mode = SetModeRequest()
        # set_mode.custom_mode = 'POSCTL'
        # if self.set_mode_client.call(set_mode).mode_sent == True:
        #     rospy.logwarn("Drone have set to position control.")
        # arm_cmd = CommandBoolRequest()
        # arm_cmd.value = False
        # if self.arming_client.call(arm_cmd).success == True:
        #     rospy.logwarn("Drone disarmed.")

if __name__ == '__main__':
    sm2px4 = SM2PX4()
    rospy.loginfo("State machine to px4 start.")
    rospy.sleep(1.0)

    set_mode = SetModeRequest()
    set_mode.custom_mode = 'POSCTL'
    if sm2px4.set_mode_client.call(set_mode).mode_sent == True:
        rospy.logwarn("Drone have set to position control.")
    set_mode.custom_mode = 'OFFBOARD'
    if sm2px4.set_mode_client.call(set_mode).mode_sent == True:
        rospy.logwarn("Drone OFFBOARD enable.")
    else:
        rospy.signal_shutdown("Fail to set OFFBOARD mode.")

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    if sm2px4.arming_client.call(arm_cmd).success == True:
        rospy.logwarn("Drone Armed.")

    rospy.spin()