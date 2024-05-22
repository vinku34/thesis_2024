#!/usr/bin/python3

import rospy

import tf

import numpy as np

from geometry_msgs.msg import Twist

from ar_track_alvar_msgs.msg import AlvarMarkers

from math import atan2, pi



MAX_LIN_VEL = 0.06
MAX_ANG_VEL = 0.6



REVERSE_VEL = 0.5 # m/s
REVERSE_DURATION = 1.5  # seconds



# For making a 180 degree turn
TURN_DURATION = 1.3*(pi / MAX_ANG_VEL)  # seconds
TURN_VEL = 0.5 # rad/s



# Tolerance for the goal distance and angle
GOAL_TOLERANCE = 0.08
ANGLE_TOLERANCE = 0.08
GOAL_DIST_FROM_MARKER = 0.58



last_heartbeat = 0
detected_markers = set()
last_midpoint = None
last_midpoint_time = 0
marker_ids = []



# state machine definition

# state 1: move towards the midpoint of the two markers

# state 2: rotate 180 degrees

# state 3: back up into the docking station

# state 4: stop

state = 1
state_start_time = 0



def callback(data):
    global detected_markers, state, last_midpoint, last_midpoint_time
    detected_markers.clear()
    marker_positions = []
    marker_orientations = []



    # Only process markers in state 1
    if state != 1:
        return

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

    # Verify that both markers were detected

    if len(marker_positions) != 2:
        rospy.logwarn("Did not detect both markers")
        return

    # Compute the midpoint between the two markers and update the heartbeat
    last_midpoint = (marker_positions[0] + marker_positions[1]) / 2.0
    last_midpoint_time = rospy.get_time()

def timer_callback(event):
    global last_heartbeat, state, state_start_time, last_midpoint, cmd_vel_pub

    # Prepare the twist message with 0 velocities
    twist_msg = Twist()
    rospy.loginfo("State: %d", state)

    # State 1: Move towards the midpoint of the two markers
    if state == 1:
        # check if the last midpoint is recent
        if rospy.get_time() - last_midpoint_time > 0.5:
            rospy.logwarn("No markers detected within 0.5 sec, rotating in place...")
            twist_msg.angular.z = MAX_ANG_VEL
        else:
          # our data is fresh, let's move the robot towards the midpoint
          # Find the direction from the robot to the midpoint
          goal_dir = last_midpoint[:3]  # We only need the x, y, z components
          dist_to_goal = np.linalg.norm(goal_dir[:2])
          angle_to_goal = atan2(goal_dir[1], goal_dir[0])

          if dist_to_goal <= GOAL_DIST_FROM_MARKER:
              # Robot is within the desired distance from the midpoint
              state = 2
              state_start_time = rospy.get_time()
              rospy.loginfo("Robot has reached the desired distance from the markers.")
              rospy.loginfo("Starting 180 degree turn")
          else:
              twist_msg.linear.x = min(max(goal_dir[0], -MAX_LIN_VEL), MAX_LIN_VEL)
              twist_msg.linear.y = min(max(goal_dir[1], -MAX_LIN_VEL), MAX_LIN_VEL)
              twist_msg.angular.z = 4 * min(max(angle_to_goal, -MAX_ANG_VEL), MAX_ANG_VEL)

    # State 2: Rotate 180 degrees
    if state == 2:
        twist_msg.angular.z = TURN_VEL

        if rospy.get_time() - state_start_time > TURN_DURATION:
            rospy.loginfo("Completed 180 degree turn")
            rospy.loginfo("Starting backing up into the docking station")
            state = 3
            state_start_time = rospy.get_time()

    # State 3: Back up into the docking station
    if state == 3:
        twist_msg.linear.x = -REVERSE_VEL

        if rospy.get_time() - state_start_time > REVERSE_DURATION:
            rospy.loginfo("Completed backing up into the docking station")
            rospy.loginfo("Docking complete. Stopping the robot.")
            state = 4

    # State 4: Stop the robot
    if state == 4:
        pass

    

    # Publish the twist message to the robot
    cmd_vel_pub.publish(twist_msg)

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
    t = rospy.Timer(rospy.Duration(0.05), timer_callback)

    # Set up subscriber for /ar_pose_marker
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()

if __name__ == '__main__':
    docking_demo()
    
