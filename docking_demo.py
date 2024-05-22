#!/usr/bin/python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, pi

MAX_LIN_VEL = 0.063
MAX_ANG_VEL = 0.64

GOAL_TOLERANCE = 0.08
ANGLE_TOLERANCE = 0.08
GOAL_DIST_FROM_MARKER = 0.58

last_heartbeat = 0
detected_markers = set()
marker_ids = []
state = 1

def callback(data):
    global last_heartbeat, detected_markers, state
    detected_markers.clear()
    marker_positions = []
    marker_orientations = []
    if state == 1:
        for marker in data.markers:
            # Check if the marker id is in the target list
            if marker.id not in marker_ids:
                continue

            detected_markers.add(marker.id)
            rospy.loginfo(rospy.get_caller_id() + " I heard %s", marker)
            marker_pos = np.array([
                marker.pose.pose.position.x,
                marker.pose.pose.position.y,
                marker.pose.pose.position.z,
                1
            ])
            marker_ori = (
                marker.pose.pose.orientation.x,
                marker.pose.pose.orientation.y,
                marker.pose.pose.orientation.z,
                marker.pose.pose.orientation.w
            )

            marker_positions.append(marker_pos)
            marker_orientations.append(marker_ori)
        
        if len(marker_positions) != 2:
            rospy.logwarn("Did not detect both markers")
            return
    
    
        # Compute the midpoint between the two markers
        midpoint = (marker_positions[0] + marker_positions[1]) / 2.0
    
        # Find the direction from the robot to the midpoint
        goal_dir = midpoint[:3]  # We only need the x, y, z components
        dist_to_goal = np.linalg.norm(goal_dir[:2])
        angle_to_goal = atan2(goal_dir[1], goal_dir[0])
    
        twist_msg = Twist()
        twist_msg.linear.x = min(max(goal_dir[0], -MAX_LIN_VEL), MAX_LIN_VEL)
        twist_msg.linear.y = min(max(goal_dir[1], -MAX_LIN_VEL), MAX_LIN_VEL)
        twist_msg.angular.z = 4 * min(max(angle_to_goal, -MAX_ANG_VEL), MAX_ANG_VEL)

        if dist_to_goal <= GOAL_DIST_FROM_MARKER:
            # Robot is within the desired distance from the midpoint
            twist_msg = Twist()
            cmd_vel_pub.publish(twist_msg)
            state = 2
            rospy.loginfo("Robot has reached the desired distance from the markers. Switching to state 2.")
        else:
            rospy.loginfo("Updating cmd vel %s", twist_msg)
            last_heartbeat = rospy.get_time()
            cmd_vel_pub.publish(twist_msg)
    if state == 2:
        twist_msg = Twist()
        twist_msg.angular.z = MAX_ANG_VEL
    
        rospy.loginfo("Starting 180 degree turn")
    
        # Start turning
        start_time = rospy.get_time()
        turn_duration = 1.3*(pi / MAX_ANG_VEL)  # Time required to turn 180 degrees
    
        #rate = rospy.Rate(10)  # 10 Hz
        while rospy.get_time() - start_time < turn_duration:
            cmd_vel_pub.publish(twist_msg)
            #rate.sleep()
    
        # Stop turning
        twist_msg.angular.z = 0
        cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("Completed 180 degree turn")
        state = 3

def timer_callback(event):
    global last_heartbeat, detected_markers, state
    if state == 3:
        # If the robot is in state 3, do nothing (or implement state 3 logic)
        cmd_vel_pub.publish(Twist())
        return

    if len(detected_markers) < 2 and state == 1:
        # Spin the robot in place
        twist_msg = Twist()
        twist_msg.angular.z = MAX_ANG_VEL  # Rotate in place
        cmd_vel_pub.publish(twist_msg)
    elif (rospy.get_time() - last_heartbeat) >= 0.5:
        cmd_vel_pub.publish(Twist())

def docking_demo():
    global marker_ids, cmd_vel_pub
    # Initialize this ROS node
    rospy.init_node('docking_demo', anonymous=True)
    # get target marker ids
    marker_ids_str = rospy.get_param('~marker_ids', "[10, 11]")  # Default value "[10, 11]" if not provided
    marker_ids = [int(id.strip()) for id in marker_ids_str.strip('[]').split(',') if id.strip()]
    rospy.loginfo("Tracking marker IDs: %s", marker_ids)

    #initialize heartbeat
    global last_heartbeat
    last_heartbeat = rospy.get_time()

    # Create publisher for command velocity
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Register heartbeat timer
    t = rospy.Timer(rospy.Duration(0.1), timer_callback)

    # Set up subscriber for /ar_pose_marker
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

    rospy.spin()

if __name__ == '__main__':
    docking_demo()

