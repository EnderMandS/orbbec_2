#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Empty
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import pi

from uav_link.msg import ExecStatus

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
        rospy.Subscriber('planning/pos_cmd', PoseStamped, self.egoPoseCb)
        rospy.Subscriber("mavros/state", State, callback=self.mavrosStateCb)
        self.ego_goal_pub = rospy.Publisher("goal", PoseStamped, queue_size=1)
        self.land_client = rospy.ServiceProxy("land", Empty)
        self.pose_pub = rospy.Publisher("sm/pose", PoseStamped, queue_size=5)

        self.ego_state = ExecStatus.EXEC_STATUS_INIT
        self.ego_sent = False
        self.odom = Odometry()
        self.odom_update = False
        self.state = State()

    def odomCb(self, msg):
        self.odom = msg
        self.odom_update = True

    def waitOdomUpdate(self):
        rospy.loginfo("Waiting for odom update.")
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
        checkExit()
        rospy.sleep(1.0)

    def turn360(self):
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

    def plannertoTarget(self, x:float, y:float, z:float, yaw:float):
        if self.checkArrive(x, y, z, yaw) == True:
            rospy.loginfo("Already in (%.2f, %.2f, %.2f, %.2f)" % (x, y, z, yaw))
            return
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
        rospy.sleep(0.1)
        self.ego_sent = True

        start_time = rospy.Time.now().to_sec()
        rospy.sleep(1.0)
        while self.ego_state != ExecStatus.EXEC_STATUS_WAIT_TARGET:
            if rospy.Time.now().to_sec()-start_time > TIMEOUT:
                self.ego_sent = False
                rospy.logerr("Planning to (%.2f, %.2f, %.2f, %.2f) time out." % (x, y, z, yaw))
                self.arriveFail()
                break
            checkExit()
            rospy.sleep(1.0)
        self.ego_sent = False
        checkExit()
        rospy.sleep(1.0)

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
    rospy.sleep(1.0)

    # Wait for px4 connect
    rospy.wait_for_service("land")
    while sm.pose_pub.get_num_connections() < 1:
        checkExit()
        rospy.loginfo("Waiting for sm2px4 connect.")
        rospy.sleep(1.5)

    # Wait for ego planner
    # while sm.ego_state != ExecStatus.EXEC_STATUS_WAIT_TARGET:
    #     rospy.loginfo("Waiting for ego planner.")
    #     rospy.sleep(2.0)
    # rospy.sleep(3.0)

    # Wait for odom
    while not sm.waitOdomUpdate():
        checkExit()
        pass

    # Wait for px4 OFFBOARD mode
    while sm.state.mode != "OFFBOARD":
        checkExit()
        rospy.loginfo("SM Waiting for OFFBOARD.")
        rospy.sleep(1.0)

    rospy.logwarn("Ready to fly.")
    rospy.logwarn("Ready to fly.")
    rospy.logwarn("Ready to fly.")
    rospy.sleep(1.5)
    checkExit()

    # Take off
    sm.gotoTarget(0, 0, 0.2, 0.0)
    rospy.sleep(1.0)
    checkExit()

    # sm.turn360()
    # rospy.sleep(3.0)
    # checkExit()

    sm.gotoTarget(0.5, 0.0, 0.2, 0.0)
    rospy.sleep(1.0)
    checkExit()

    # sm.turn180()
    # rospy.sleep(3.0)
    # checkExit()

    sm.gotoTarget(0, 0, 0.2, 0.0)
    rospy.sleep(3.0)
    checkExit()

    rospy.logwarn("Land by hand.")
    rospy.logwarn("Land by hand.")
    rospy.logwarn("Land by hand.")
    rospy.sleep(20.0)

    # Look around
    # sm.gotoTarget(0, 0, 0.25, pi/2)
    # sm.gotoTarget(0, 0, 0.25, pi)
    # sm.gotoTarget(0, 0, 0.25, -pi/2)
    # sm.gotoTarget(0, 0, 0.25, 0)

    # Planner to 
    # sm.plannertoTarget(0.5, 0, 0.35, 0)

    # sm.gotoTarget(0, 0, 0.25, 0)

    # Land
    # sm.land_client.call(Empty())

    rospy.loginfo("All waypoint done exit.")
    rospy.signal_shutdown("All waypoint done exit.")
    exit()
