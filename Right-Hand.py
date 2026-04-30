#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor, TouchSensor, ColorSensor,
    InfraredSensor, UltrasonicSensor,
    GyroSensor
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import random
import math

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
color_sensor = ColorSensor(Port.S2)

# Obstacle detection thresholds
FRONT_OBSTACLE_DISTANCE = 60
SIDE_OBSTACLE_DISTANCE = 200

# List to store nodes
listNodes = []
visited_nodes = set()
target_nodes = []

is_paused = False
last_target_time = 0
TARGET_COOLDOWN = 5000


class Node:
    def __init__(self, tid, tprev, tnext, disttop, pos_x=0, pos_y=0, is_target=False):
        self.tID = tid
        self.tPrev = tprev
        self.distToPrev = disttop
        self.tNext = tnext
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.visited = False
        self.is_target = is_target


def reset_gyro():
    gyro_sensor.reset_angle(0)
    wait(300)


def getDirection():
    return gyro_sensor.angle()


def readDirection():
    return gyro_sensor.angle()


def turnRobot(reqAngle):
    current_angle = readDirection()
    angle_diff = reqAngle - current_angle

    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360

    tolerance = 3

    if angle_diff < -tolerance:
        while readDirection() > reqAngle + tolerance:
            robot.drive(0, -30)
            wait(10)

    elif angle_diff > tolerance:
        while readDirection() < reqAngle - tolerance:
            robot.drive(0, 30)
            wait(10)

    robot.stop()
    wait(300)


def turnRight():
    reset_gyro()
    target_angle = 90

    while abs(gyro_sensor.angle() - target_angle) > 3:
        robot.drive(0, 30)
        wait(10)

    robot.stop()
    wait(300)


def turnLeft():
    reset_gyro()
    target_angle = -90

    while abs(gyro_sensor.angle() - target_angle) > 3:
        robot.drive(0, -30)
        wait(10)

    robot.stop()
    wait(300)


def turnOpp():
    reset_gyro()
    target_angle = 180

    while abs(gyro_sensor.angle() - target_angle) > 3:
        robot.drive(0, 30)
        wait(10)

    robot.stop()
    wait(300)


def getDistToObst():
    distOb = obstacle_sensor.distance()
    ev3.screen.clear()
    ev3.screen.draw_text(20, 0, "Dist: " + str(distOb))
    return distOb


def checkRightObs():
    servo_motor.run_angle(50, 90)
    wait(500)

    rightObst = getDistToObst()
    wait(500)

    servo_motor.run_angle(50, -90)
    wait(500)

    return rightObst


def checkLeftObs():
    servo_motor.run_angle(50, -90)
    wait(500)

    leftObst = getDistToObst()
    wait(500)

    servo_motor.run_angle(50, 90)
    wait(500)

    return leftObst


def readColor():
    color = color_sensor.color()
    reflection = color_sensor.reflection()

    if color == Color.WHITE and reflection > 30:
        ev3.screen.clear()
        ev3.screen.draw_text(0, 90, "TARGET DETECTED!")
        ev3.speaker.beep(frequency=800, duration=300)
        return "WHITE"

    return "OTHER"


def all_nodes_visited():
    for node in listNodes:
        if not node.visited:
            return False
    return True


def display_visited_nodes():
    ev3.screen.clear()
    y_position = 20

    ev3.screen.draw_text(0, 0, "Visited Nodes:")

    displayed_nodes = 0
    max_nodes_per_page = 4

    for node in listNodes:
        prefix = "T" if node.is_target else "N"

        node_info = "{} {}: ({:.0f},{:.0f}) {}".format(
            prefix,
            node.tID,
            node.pos_x,
            node.pos_y,
            "★" if node.is_target else ""
        )

        ev3.screen.draw_text(
            0,
            y_position,
            node_info,
            text_color=Color.BLACK,
            background_color=Color.WHITE if node.is_target else None
        )

        y_position += 20
        displayed_nodes += 1

        if displayed_nodes >= max_nodes_per_page and displayed_nodes < len(listNodes):
            ev3.screen.draw_text(0, 120, "Press any button")

            while not any(ev3.buttons.pressed()):
                wait(10)

            while any(ev3.buttons.pressed()):
                wait(10)

            ev3.screen.clear()
            y_position = 20
            ev3.screen.draw_text(0, 0, "More Nodes:")
            displayed_nodes = 0


def display_targets():
    ev3.screen.clear()

    if not target_nodes:
        ev3.screen.draw_text(0, 50, "No targets found")
        return

    y_position = 20
    ev3.screen.draw_text(0, 0, "Target Nodes:")

    for i, node_id in enumerate(target_nodes):
        node = listNodes[node_id]

        node_info = "T{}: ({:.0f},{:.0f}) ★".format(
            node.tID,
            node.pos_x,
            node.pos_y
        )

        ev3.screen.draw_text(
            0,
            y_position,
            node_info,
            text_color=Color.BLACK,
            background_color=Color.WHITE
        )

        y_position += 20

        if (i + 1) % 4 == 0 and (i + 1) < len(target_nodes):
            ev3.screen.draw_text(0, 120, "Press any button")

            while not any(ev3.buttons.pressed()):
                wait(10)

            while any(ev3.buttons.pressed()):
                wait(10)

            ev3.screen.clear()
            y_position = 20
            ev3.screen.draw_text(0, 0, "More Targets:")


def decide_direction(right_obstacle, left_obstacle, front_obstacle):
    if right_obstacle > SIDE_OBSTACLE_DISTANCE:
        return "right"
    elif front_obstacle > FRONT_OBSTACLE_DISTANCE:
        return "forward"
    elif left_obstacle > SIDE_OBSTACLE_DISTANCE:
        return "left"
    else:
        return "back"


def is_position_unique(new_x, new_y, tolerance=50):
    for node in listNodes:
        if abs(node.pos_x - new_x) < tolerance and abs(node.pos_y - new_y) < tolerance:
            return False
    return True


def main():
    global target_nodes, is_paused, last_target_time

    reset_gyro()

    # Initialize first node
    tCount = 0
    t0 = Node(tCount, None, None, 0)
    listNodes.append(t0)
    tCount += 1

    # Start beep
    for _ in range(3):
        ev3.speaker.beep()
        wait(300)

    while True:

        if is_paused:
            if Button.UP in ev3.buttons.pressed():
                ev3.speaker.beep(frequency=500, duration=200)
                display_visited_nodes()

                ev3.screen.draw_text(0, 120, "Press DOWN for targets")

                while not any(ev3.buttons.pressed()):
                    wait(10)

                if Button.DOWN in ev3.buttons.pressed():
                    display_targets()

                while any(ev3.buttons.pressed()):
                    wait(10)

                continue

            elif Button.CENTER in ev3.buttons.pressed():
                is_paused = False
                ev3.screen.clear()
                robot.reset()

                while any(ev3.buttons.pressed()):
                    wait(10)

                continue

        robot.reset()
        robot.drive(100, 0)

        should_create_node = False

        while True:
            try:
                front_dist = obstacle_sensor.distance()

                if front_dist <= FRONT_OBSTACLE_DISTANCE:
                    robot.stop()
                    should_create_node = True
                    break
            except:
                pass

            if Button.UP in ev3.buttons.pressed():
                robot.stop()
                robot.reset()
                is_paused = True
                should_create_node = False

                ev3.speaker.beep(frequency=500, duration=200)

                display_visited_nodes()
                ev3.screen.draw_text(0, 120, "Press DOWN for targets")

                while not any(ev3.buttons.pressed()):
                    wait(10)

                if Button.DOWN in ev3.buttons.pressed():
                    display_targets()

                while any(ev3.buttons.pressed()):
                    wait(10)

                break

            color_detected = readColor()

            if color_detected == "WHITE" and not is_paused:
                current_time = StopWatch().time()
                dist_to_prev = robot.distance()

                if dist_to_prev > 50 and current_time - last_target_time >= TARGET_COOLDOWN:
                    angle = readDirection()
                    last_node = listNodes[-1]

                    new_x = last_node.pos_x + dist_to_prev * math.cos(math.radians(angle))
                    new_y = last_node.pos_y + dist_to_prev * math.sin(math.radians(angle))

                    if is_position_unique(new_x, new_y):
                        newNode = Node(
                            tCount,
                            listNodes[tCount - 1].tID,
                            None,
                            dist_to_prev,
                            new_x,
                            new_y,
                            is_target=True
                        )

                        listNodes.append(newNode)
                        listNodes[tCount - 1].tNext = tCount
                        target_nodes.append(tCount)

                        ev3.screen.clear()
                        ev3.screen.draw_text(20, 0, "TARGET T" + str(tCount))

                        tCount += 1
                        last_target_time = current_time

                        wait(2000)
                        ev3.screen.clear()

            wait(100)

        if is_paused:
            continue

        if tCount > 0:
            listNodes[-1].visited = True

        right_obstacle = checkRightObs()
        left_obstacle = checkLeftObs()
        front_obstacle = obstacle_sensor.distance()

        direction = decide_direction(right_obstacle, left_obstacle, front_obstacle)

        if direction == "right":
            turnRight()
        elif direction == "left":
            turnLeft()
        elif direction == "back":
            turnOpp()
        else:
            wait(1000)

        robot.stop()
        reset_gyro()

        if should_create_node and not is_paused:
            dist_to_prev = robot.distance()

            if dist_to_prev > 50 and tCount > 0:
                angle = readDirection()
                last_node = listNodes[-1]

                new_x = last_node.pos_x + dist_to_prev * math.cos(math.radians(angle))
                new_y = last_node.pos_y + dist_to_prev * math.sin(math.radians(angle))

                if is_position_unique(new_x, new_y):
                    newNode = Node(
                        tCount,
                        listNodes[tCount - 1].tID,
                        None,
                        dist_to_prev,
                        new_x,
                        new_y
                    )

                    listNodes.append(newNode)
                    listNodes[tCount - 1].tNext = tCount

                    ev3.screen.clear()
                    ev3.screen.draw_text(20, 0, "Node N" + str(tCount))

                    tCount += 1

                    wait(1000)
                    ev3.screen.clear()


if __name__ == "__main__":
    main()