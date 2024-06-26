import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float32MultiArray
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Constants
FOV = 100  # FOV of the camera in degrees
NUM_SECTIONS = 72  # Original number of sections
REDUCED_SECTIONS = NUM_SECTIONS // 2  # Reduced number of sections after averaging

# Thresholds
OBSTACLE_THRESHOLD = 40
TARGET_REACHED_THRESHOLD = 0.05
ANGLE_DIFF_THRESHOLD = 45.0 

# Target position
target_lat = 28.75375436390428
target_lon = 77.11608874115085

# Connect to the vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=False)

def arm_and_start():
    """
    Arms vehicle and starts movement.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

def calculate_bearing(current_position, destination_position):
    dx = destination_position.lon - current_position.lon
    dy = destination_position.lat - current_position.lat
    bearing = math.degrees(math.atan2(dy, dx))
    return bearing

def send_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)     # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def move_towards_target(current_position):
    global target_lat, target_lon
    dNorth = target_lat - current_position.lat
    dEast = target_lon - current_position.lon
    dist = math.sqrt(dNorth**2 + dEast**2)
    bearing = calculate_bearing(current_position, LocationGlobalRelative(target_lat, target_lon, 0))
    vx = dist * math.cos(math.radians(bearing))
    vy = dist * math.sin(math.radians(bearing))
    max_speed = 2.0  
    vx = max(min(vx, max_speed), -max_speed)
    vy = max(min(vy, max_speed), -max_speed)

    send_ned_velocity(vx, vy, 0)

def average_sections(data):
    averaged_data = []
    for i in range(0, len(data), 2):
        section_avg = np.mean(data[i:i+2])
        averaged_data.append(section_avg)
        #print(averaged_data)

    return averaged_data

def distances_callback(msg):
    global distances
    distances = average_sections(msg.data)

def find_safe_direction(distances, current_heading, num_sections=REDUCED_SECTIONS):
    """
    Finds the safest direction to move towards, considering negative (left) and positive (right) bearings.
    """
    max_distance_left = -1
    max_distance_right = -1
    best_bearing = None
    bearing_increment = FOV / num_sections

   
    current_index = int(round((current_heading + FOV/2) / bearing_increment)) % num_sections

    # Check negative (left) bearings
    for i in range(1, num_sections // 2 + 1):
        left_index = (current_index - i) % num_sections
        if distances[left_index] > max_distance_left:
            max_distance_left = distances[left_index]

    # Check positive (right) bearings
    for i in range(1, num_sections // 2 + 1): 
        right_index = (current_index + i) % num_sections
        if distances[right_index] > max_distance_right:
            max_distance_right = distances[right_index]

    # Determine the safest direction based on distances
    if max_distance_left > max_distance_right:
        best_bearing = current_heading - (FOV / 2) / 2  # Move towards negative bearing
    else:
        best_bearing = current_heading + (FOV / 2) / 2  # Move towards positive bearing

    return best_bearing



    return best_bearing

def dynamic_speed(distance):
    if distance < OBSTACLE_THRESHOLD:
        return max(0.5, 2.0 * (distance / OBSTACLE_THRESHOLD))
    return 2.0
def main():
    global current_position, distances

    rospy.init_node('bendy_ruler_navigation')
    rospy.Subscriber('filtered_data', Float32MultiArray, distances_callback)

    arm_and_start()

    vehicle.mode = VehicleMode("GUIDED")

    distances = []  

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_position = vehicle.location.global_relative_frame
        distance_to_destination = np.linalg.norm([
            current_position.lat - target_lat,
            current_position.lon - target_lon
        ])
        if distance_to_destination <= TARGET_REACHED_THRESHOLD:
            rospy.loginfo("Destination reached")
            break
        if distances and min(distances) < OBSTACLE_THRESHOLD:
            
            rospy.loginfo("Obstacleeeeeeeeeeeeeeee detected! Changing direction.")
            current_heading = calculate_bearing(current_position, LocationGlobalRelative(target_lat, target_lon, 0))
            best_bearing = find_safe_direction(distances, current_heading)
            
            if best_bearing is not None:
                speed = dynamic_speed(min(distances))
                vx = speed * math.cos(math.radians(best_bearing))
                vy = speed * math.sin(math.radians(best_bearing))
                send_ned_velocity(vx, vy, 0)
            else:
                rospy.loginfo("No safe direction found within threshold.")

        else:
            move_towards_target(current_position)

        rospy.loginfo(f"Current position: {current_position.lat}, {current_position.lon}")

        rate.sleep()

    vehicle.mode = VehicleMode("HOLD")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
