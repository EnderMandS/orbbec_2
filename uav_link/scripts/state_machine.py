#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from math import pi

from quadrotor_msgs.msg import ExecStatus

POSITION_ABS:float = 0.15  # m
YAW_ABS:float = 10/180*pi  # rad
TIMEOUT:int = 30        # second
ODOM_TIMEOUT:int = 10   # second
YAW_SPEED:float = pi/8    # rad/s

def ENU2NED(x:float, y:float, z:float, yaw:float):
    return y,x,-z,-yaw
def NED2ENU(x:float, y:float, z:float, yaw:float):
    return y,x,-z,-yaw
def checkExit():
    if rospy.is_shutdown():
        rospy.logwarn("SM user exit.")
        rospy.signal_shutdown("User exit.")
        exit()

class StateMachine(object):
    def __init__(self) -> None:
        rospy.init_node('state_machine')
        rospy.on_shutdown(self.shutdownCb)

        rospy.Subscriber("odom", Odometry, self.odomCb)
        rospy.Subscriber("planning/exec_state", ExecStatus, self.egoStateCb)
        rospy.Subscriber('planning/pos_cmd_geo', PoseStamped, self.egoPoseCb)
        rospy.Subscriber("mavros/state", State, callback=self.mavrosStateCb)
        self.ego_goal_pub = rospy.Publisher("goal", PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("sm/pose", PoseStamped, queue_size=5)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        self.ego_state = ExecStatus.EXEC_STATUS_INIT
        self.ego_sent = False
        self.odom = Odometry()
        self.odom_update = False
        self.state = State()

    def odomCb(self, msg):
        self.odom = msg
        self.odom_update = True

    def waitOdomUpdate(self):
        rospy.loginfo("SM Waiting for odom update.")
        self.odom_update = False
        start_time = rospy.Time.now().to_sec()
        while self.odom_update==False:
            if (rospy.Time.now().to_sec() - start_time) > ODOM_TIMEOUT:
                rospy.logerr("Waiting for odom timeout.")
                return False
            rospy.sleep(0.5)
            if rospy.is_shutdown():
                rospy.signal_shutdown("User exit.")
        return True

    def egoStateCb(self, msg):
        self.ego_state = msg.exec_flag

    def egoPoseCb(self, msg):
        if self.ego_sent == True:
            self.pose_pub.publish(msg)

    def mavrosStateCb(self, msg):
        self.state = msg

    def gotoTarget(self, x:float, y:float, z:float, yaw:float):
        if self.checkArrive(x, y, z, yaw) == True:
            rospy.logwarn("Already at (%.2f, %.2f, %.2f, %.2f)." % (x, y, z, yaw))
            return
        rospy.logwarn("Going to (%.2f, %.2f, %.2f, %.2f)." % (x, y, z, yaw))

        self.ego_sent = False
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.pose_pub.publish(pose)
        
        rospy.sleep(1.0)
        start_time = rospy.Time.now().to_sec()
        while self.checkArrive(x, y, z, yaw) == False:
            if rospy.Time.now().to_sec()-start_time > TIMEOUT:
                rospy.logerr("Going to (%.2f, %.2f, %.2f, %.2f) time out." % (x, y, z, yaw))
                self.arriveFail()
                break
            checkExit()
            rospy.sleep(1.0)
        rospy.logwarn("Going arrive (%.2f, %.2f, %.2f, %.2f)." % (x, y, z, yaw))
        checkExit()

    def turn360(self):
        rospy.logwarn("Turn 360.")
        start_position = self.odom.pose.pose.position
        r, p, yaw_angle = euler_from_quaternion([ \
                self.odom.pose.pose.orientation.x, \
                self.odom.pose.pose.orientation.y, \
                self.odom.pose.pose.orientation.z, \
                self.odom.pose.pose.orientation.w])
        total_time:float = 2*pi/YAW_SPEED # second
        self.ego_sent = False
        for i in range(1,(int)(total_time*100)):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            yaw_expect = yaw_angle+(i/100.0*YAW_SPEED)
            if yaw_expect>pi:
                yaw_expect -= 2*pi
            q = quaternion_from_euler(0, 0, yaw_expect)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            pose.pose.position.x = start_position.x
            pose.pose.position.y = start_position.y
            pose.pose.position.z = start_position.z
            self.pose_pub.publish(pose)
            rospy.sleep(0.01)
            checkExit()

    def turn180(self):
        rospy.logwarn("Turn 180.")
        start_position = self.odom.pose.pose.position
        r, p, yaw_angle = euler_from_quaternion([ \
                self.odom.pose.pose.orientation.x, \
                self.odom.pose.pose.orientation.y, \
                self.odom.pose.pose.orientation.z, \
                self.odom.pose.pose.orientation.w])
        total_time:float = pi/YAW_SPEED # second
        self.ego_sent = False
        for i in range(1,(int)(total_time*100)):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            yaw_expect = yaw_angle+(i/100.0*YAW_SPEED)
            if yaw_expect>pi:
                yaw_expect -= 2*pi
            q = quaternion_from_euler(0, 0, yaw_expect)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            pose.pose.position.x = start_position.x
            pose.pose.position.y = start_position.y
            pose.pose.position.z = start_position.z
            self.pose_pub.publish(pose)
            rospy.sleep(0.01)
            checkExit()

    def plannertoTarget(self, x:float, y:float, z:float, yaw:float, delay:float=1.5):
        if self.checkArrive(x, y, z, yaw) == True:
            rospy.loginfo("Already in (%.2f, %.2f, %.2f, %.2f)" % (x, y, z, yaw))
            return
        rospy.logwarn("Planner to (%.2f, %.2f, %.2f, %.2f)." % (x, y, z, yaw))

        pose = PoseStamped()
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.ego_goal_pub.publish(pose)
        rospy.sleep(delay+0.1)
        self.ego_sent = True

        start_time = rospy.Time.now().to_sec()
        rospy.sleep(0.5)
        while self.ego_state != ExecStatus.EXEC_STATUS_WAIT_TARGET:
            if rospy.Time.now().to_sec()-start_time > TIMEOUT:
                self.ego_sent = False
                rospy.logerr("Planning to (%.2f, %.2f, %.2f, %.2f) time out." % (x, y, z, yaw))
                self.arriveFail()
                break
            checkExit()
            rospy.sleep(1.0)
        rospy.logwarn("Planner arrive (%.2f, %.2f, %.2f, %.2f)." % (x, y, z, yaw))
        self.ego_sent = False
        checkExit()

    def takeoff(self):
        set_mode = SetModeRequest()
        set_mode.custom_mode = 'AUTO.TAKEOFF'
        if self.set_mode_client.call(set_mode).mode_sent == True:
            rospy.loginfo("Drone auto takeoff.")
    
    def land(self):
        rospy.loginfo("SM drone going to land.")
        set_mode = SetModeRequest()
        set_mode.custom_mode = 'AUTO.LAND'
        if self.set_mode_client.call(set_mode).mode_sent == True:
            rospy.loginfo("Drone auto land.")
    
    def arm(self):
        rospy.loginfo("SM drone going to arm.")
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if self.arming_client.call(arm_cmd).success == True:
            rospy.logwarn("Drone armed.")
    
    def disArm(self):
        rospy.loginfo("SM drone going to disArm.")
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = False
        if self.arming_client.call(arm_cmd).success == True:
            rospy.logwarn("Drone disarmed.")

    def checkArrive(self, x:float, y:float, z:float, yaw:float):
        r, p, yaw_angle = euler_from_quaternion([ \
                self.odom.pose.pose.orientation.x, \
                self.odom.pose.pose.orientation.y, \
                self.odom.pose.pose.orientation.z, \
                self.odom.pose.pose.orientation.w])
        if  abs(self.odom.pose.pose.position.x-x) < POSITION_ABS and \
            abs(self.odom.pose.pose.position.y-y) < POSITION_ABS and \
            abs(self.odom.pose.pose.position.z-z) < POSITION_ABS and \
            abs(yaw_angle-yaw) < YAW_ABS:
            return True
        return False

    def arriveFail(self):
        rospy.signal_shutdown("Go to position fail.")

    def shutdownCb(self):
        rospy.loginfo("State machine shut down.")

if __name__ == '__main__':
    sm = StateMachine()
    rospy.loginfo("State machine start.")
    rospy.sleep(0.1)

    # Wait for px4 connect
    rospy.loginfo("SM Waiting for sm2px4 connect.")
    rospy.wait_for_service("mavros/set_mode")
    rospy.wait_for_service("mavros/cmd/arming")
    while sm.pose_pub.get_num_connections() < 1:
        checkExit()
        rospy.sleep(0.1)

    # Wait for ego planner
    rospy.loginfo("SM Waiting for ego planner.")
    while sm.ego_state != ExecStatus.EXEC_STATUS_WAIT_TARGET:
        rospy.sleep(0.1)
        checkExit()

    # Wait for odom
    rospy.loginfo("SM Waiting for odometry.")
    while not sm.waitOdomUpdate():
        checkExit()

    # Wait for px4 OFFBOARD mode
    rospy.loginfo("SM Waiting for OFFBOARD.")
    while sm.state.mode != "OFFBOARD":
        checkExit()
        rospy.sleep(0.1)

    rospy.logwarn("Ready to fly.")
    checkExit()

    # Take off
    sm.gotoTarget(0, 0, 0.3, 0.0)
    rospy.sleep(1.0)
    checkExit()

    sm.plannertoTarget(3.0, 0, 0.3, 0.0, 0.0)
    rospy.sleep(1.0)
    checkExit()

    sm.turn180()
    rospy.sleep(1.0)
    checkExit()

    sm.plannertoTarget(0, 0, 0.3, pi/2, 1.5)
    rospy.sleep(1.0)
    checkExit()

    sm.turn180()
    rospy.sleep(1.0)
    checkExit()

    sm.gotoTarget(0, 0, 0.3, 0.0)
    rospy.sleep(1.0)
    checkExit()

    # sm.gotoTarget(0.5, 0.0, 0.3, 0.0)
    # rospy.sleep(1.0)
    # checkExit()

    # sm.turn180()
    # rospy.sleep(3.0)
    # checkExit()

    # sm.gotoTarget(0, 0, 0.3, 0.0)
    # rospy.sleep(1.0)
    # checkExit()

    # rospy.logwarn("Land by hand.")
    # rospy.sleep(10.0)

    # Look around
    # sm.gotoTarget(0, 0, 0.25, pi/2)
    # sm.gotoTarget(0, 0, 0.25, pi)
    # sm.gotoTarget(0, 0, 0.25, -pi/2)
    # sm.gotoTarget(0, 0, 0.25, 0)

    # Planner to 
    # sm.plannertoTarget(0.5, 0, 0.35, 0)

    # sm.gotoTarget(0, 0, 0.25, 0)

    # Land
    sm.land()
    rospy.sleep(3.0)
    sm.disArm()

    rospy.loginfo("All waypoint done exit.")
    rospy.signal_shutdown("All waypoint done exit.")
    exit()
