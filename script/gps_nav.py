#!/usr/bin/env python
import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False
flight_altitude = 3   # Check altitude value before experiments

def waiter(condition):
	while True:
		if condition:
			return
		else:
			rospy.sleep(2)

def waypoint_callback(data):
	# print("\n----------waypoint_callback----------")
	global last_waypoint
	# rospy.loginfo("Got waypoint: %s", data)
	if len(data.waypoints) != 0:							# If waypoint list is not empty
		rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
		last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	# Checks status of "is_current" for last waypoint

def globalPosition_callback(data):
	# print("\n----------globalPosition_callback----------")
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude
	
def clear_pull():
	print("\n----------clear_pull----------")
	# Clearing waypoints
	rospy.wait_for_service("/mavros/mission/clear")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	rospy.sleep(5)
	# Call waypoints_pull
	rospy.wait_for_service("/mavros/mission/pull")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	rospy.sleep(5)
	return

def finishWaypoints():
	print("\n----------finishwaypoints----------")
	while True:						# Waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		# Waiting for last_waypoint to be true
		if last_waypoint == True:			# If last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				# Waiting for last_waypoint to be false
				if last_waypoint == True:	# If last_waypoint has been visited (due to previous constraint)
					break
			break
	return

def armingCall():
	print("\n----------armingCall----------")
	rospy.wait_for_service("/mavros/cmd/arming")
	asv_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = asv_arm(True)
	rospy.sleep(2)
	
def pushingWaypoints(poi):
	print("\n----------pushingWaypoints----------")
	rospy.wait_for_service("/mavros/mission/push")
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(0, poi)
	rospy.sleep(5)
	return

def switch_modes(): # current_mode: int, next_mode: str (http://docs.ros.org/jade/api/mavros_msgs/html/srv/SetMode.html)
	print("\n----------switch_modes----------")
	rospy.wait_for_service("/mavros/set_mode")
	modes = rospy.ServiceProxy("/mavros/set_mode", SetMode)
	resp = modes(custom_mode ='AUTO')
	rospy.sleep(5)
	return

def disarmingCall():
	print("\n----------disarmingCall----------")
	rospy.wait_for_service("/mavros/cmd/arming")
	asv_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = asv_arm(False)
	rospy.sleep(2)

def main():
    rospy.init_node('gps_navigation_node', anonymous=True)
    rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	#readyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10) # Flag topic
     
    clear_pull()
    
    armingCall()	

    waypoints = [
        Waypoint(),
        Waypoint(),
        Waypoint()
    ]

    # Configure the waypoints
    waypoints[0].x_lat = 38.99044  # Latitude of the first waypoint
    waypoints[0].y_long = -76.93776  # Longitude of the first waypoint
    waypoints[0].z_alt = 0  # Altitude of the first waypoint

    waypoints[1].x_lat = 38.99066  # Latitude of the second waypoint
    waypoints[1].y_long = -76.93738  # Longitude of the second waypoint
    waypoints[1].z_alt = 0  # Altitude of the second waypoint

    waypoints[2].x_lat = 38.99113  # Latitude of the third waypoint
    waypoints[2].y_long =-76.93747  # Longitude of the third waypoint
    waypoints[2].z_alt = 0  # Altitude of the third waypoint

    pushingWaypoints(waypoints)

    switch_modes() #changing from manual mode to auto mode

    finishWaypoints()

    rospy.sleep(10)

    disarmingCall()

    rospy.spin()


if __name__ == '__main__':
	main()
