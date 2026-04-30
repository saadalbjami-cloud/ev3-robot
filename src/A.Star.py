#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
import random
from collections import deque

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Define motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
servo_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Define sensors
obstacle_sensor = UltrasonicSensor(Port.S4)
gyro_sensor = GyroSensor(Port.S1)
colorSensor = ColorSensor(Port.S2)

# Node Class
class Node:
    def __init__(self, nid, nprev, nnext, disttop, x, y):
        self.nID = nid
        self.nPrev = nprev
        self.distToPrev = disttop
        self.nNext = nnext
        self.x = x
        self.y = y

# Priority Queue implementation for A*
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        self.elements.append((priority, item))
        self.elements.sort(key=lambda x: x[0])  # Simple sort for small queues
    
    def get(self):
        return self.elements.pop(0)[1]

# List to store nodes
listNodes = []

# Initialize nodes from static map
def initialize_static_map():
    for node_id in range(6):  # 6 nodes (0 to 5)
        node_data = static_map[node_id]
        new_node = Node(node_id, None, None, 0, node_data['x'], node_data['y'])
        listNodes.append(new_node)
    
    # Now set the connections properly
    for node_id in range(6):
        if static_map[node_id]['connections']:
            listNodes[node_id].nNext = static_map[node_id]['connections']  # Store all connections

# Reset gyro
def reset_gyro():
    gyro_sensor.reset_angle(0)
    wait(300)

# Get direction
def getDirection():
    return gyro_sensor.angle()

# Turn to specific angle
def turnRobot(reqAngle):
    current_angle = getDirection()
    angle_diff = reqAngle - current_angle
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    tolerance = 3
    if angle_diff < -tolerance:
        while getDirection() > reqAngle + tolerance:
            robot.drive(0, -30)
            wait(10)
    elif angle_diff > tolerance:
        while getDirection() < reqAngle - tolerance:
            robot.drive(0, 30)
            wait(10)
    robot.stop()
    wait(300)

# Read color sensor
def readColor():
    color_val = colorSensor.reflection()
    if color_val > 70:
        ev3.screen.draw_text(0, 60, "White detected")
        robot.stop()
        wait(5000)
        return True
    return False

# Calculate Euclidean distance between two nodes
def calculate_distance(node1, node2):
    dx = node1.x - node2.x
    dy = node1.y - node2.y
    return math.sqrt(dx**2 + dy**2)

# Heuristic function for A* (Euclidean distance)
def heuristic(a, b):
    return calculate_distance(a, b)

# A* pathfinding algorithm
def a_star_search(start_node, goal_node):
    frontier = PriorityQueue()
    frontier.put(start_node.nID, 0)
    
    came_from = {}
    cost_so_far = {}
    
    came_from[start_node.nID] = None
    cost_so_far[start_node.nID] = 0
    
    while not frontier.empty():
        current_id = frontier.get()
        current_node = listNodes[current_id]
        
        if current_id == goal_node.nID:
            break
        
        for next_id in current_node.nNext:
            next_node = listNodes[next_id]
            new_cost = cost_so_far[current_id] + calculate_distance(current_node, next_node)
            
            if next_id not in cost_so_far or new_cost < cost_so_far[next_id]:
                cost_so_far[next_id] = new_cost
                priority = new_cost + heuristic(next_node, goal_node)
                frontier.put(next_id, priority)
                came_from[next_id] = current_id
    
    # Reconstruct path
    path = []
    current_id = goal_node.nID
    while current_id != start_node.nID:
        path.append(current_id)
        current_id = came_from[current_id]
    path.append(start_node.nID)
    path.reverse()
    
    return path

# Follow the path calculated by A*
def follow_path(path):
    for i in range(len(path)-1):
        current_node = listNodes[path[i]]
        next_node = listNodes[path[i+1]]
        
        # Calculate direction and distance to next node
        dx = next_node.x - current_node.x
        dy = next_node.y - current_node.y
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.degrees(math.atan2(dy, dx))
        
        # Turn to face next node
        turnRobot(angle)
        
        # Move to next node
        robot.straight(distance)
        
        # Check for white line (stop condition)
        if readColor():
            break
        
        # Small delay between movements
        wait(500)

# Main function
def main():
    reset_gyro()
    
    # Initialize static map with 6 nodes
    initialize_static_map()
    
    # Beep 6 times to indicate 6 nodes are ready
    for i in range(6):
        ev3.speaker.beep()
        wait(300)
    
    # Set start and goal nodes (can be changed)
    start_node = listNodes[0]  # Node 0
    goal_node = listNodes[2]   # Node 2
    
    # Run A* algorithm to find path
    path = a_star_search(start_node, goal_node)
    
    # Display the path found
    ev3.screen.clear()
    ev3.screen.draw_text(0, 0, "Path found:")
    ev3.screen.draw_text(0, 20, str(path))
    wait(3000)
    
    # Follow the path
    follow_path(path)
    
    # Display completion message
    ev3.screen.clear()
    ev3.screen.draw_text(0, 0, "Path completed!")
    wait(5000)

# Run
if __name__ == "__main__":
    main()
