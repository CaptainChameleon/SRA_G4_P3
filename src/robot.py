import abc
import csv
import json
import logging
import math
import os
import time
from configparser import ConfigParser
from typing import Tuple

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_3

from utils import Vector


class Robot:

    def __init__(self, log, wheel_diameter: float, wheel_base: float, base_left_speed: float, base_right_speed: float,
                 initial_theta: float):
        self.log = log

        # Robot components
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        self.ultrasonic_sensor = UltrasonicSensor()
        self.color_sensor = ColorSensor(INPUT_3)
        self.color_sensor.mode = 'COL-COLOR'
        # self.gyroscope = GyroSensor()
        self.sound = Sound()

        # Physical parameters
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.base_speed = 10
        self.base_left_speed = base_left_speed
        self.base_right_speed = base_right_speed

        # Odometry parameters
        self.left_motor_dis = 0.0
        self.right_motor_dis = 0.0
        self.pos = Vector(0, 0)
        self.look_at = Vector(0, 1)
        self.theta = math.radians(initial_theta)

        # Positional parameters
        self.stored_theta = None
        self.stored_position = None

    # ****************************************************************
    # Positional System
    # ****************************************************************

    def tachos_to_distance(self, tachos: int) -> float:
        return (tachos / 360) * math.pi * self.wheel_diameter

    def distance_to_tachos(self, distance: float) -> float:
        return (distance / (math.pi * self.wheel_diameter)) * 360

    def rotation_to_wheel_degrees(self, degrees: float) -> float:
        wheel_travel_distance = (degrees / 360) * math.pi * self.wheel_base
        return self.distance_to_tachos(wheel_travel_distance)

    def reset_motors(self):
        self.log.debug("Motor L pos after movement: {}".format(self.left_motor.position))
        self.log.debug("Motor R pos after movement: {}".format(self.right_motor.position))
        self.left_motor.reset()
        self.right_motor.reset()
        self.left_motor_dis = 0
        self.right_motor_dis = 0
        self.log.debug("Motor L pos after reset: {}".format(self.left_motor.position))
        self.log.debug("Motor R pos after reset: {}".format(self.right_motor.position))

    def update_odometry(self):
        self.log.debug("Updating odometry")

        # Update robot's orientation (theta)
        current_left_motor_dis = self.tachos_to_distance(self.left_motor.position)
        current_right_motor_dis = self.tachos_to_distance(self.right_motor.position)
        d_left = current_left_motor_dis - self.left_motor_dis
        d_right = current_right_motor_dis - self.right_motor_dis
        self.left_motor_dis = current_left_motor_dis
        self.right_motor_dis = current_right_motor_dis

        d_center = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / self.wheel_base
        self.theta += delta_theta
        self.theta %= (2 * math.pi)  # Keep theta within [0, 2*pi]

        # Update position
        self.pos.x += d_center * math.cos(self.theta)
        self.pos.y += d_center * math.sin(self.theta)

        # Update look_at vector
        self.look_at.x = math.sin(self.theta)
        self.look_at.y = math.cos(self.theta)

        # Log updated values
        self.log.debug("Updated Position: {}".format(self.pos))
        self.log.debug("Updated Orientation: {:.4f} radians".format(self.theta))
        self.log.debug("Look-at Vector: {}".format(self.look_at))

    @property
    def speed(self):
        return self.base_speed

    @speed.setter
    def speed(self, new_speed):
        self.base_speed = new_speed
        self.base_left_speed = new_speed * 1.0023746690616273
        self.base_right_speed = new_speed * 0.9976309566323637

    def move_straight(self, distance: float):
        self.log.info("\n||> MOVING {} {:.4f} cm".format("FORWARD" if distance > 0 else "BACKWARD", distance))
        degrees_to_turn = self.distance_to_tachos(distance)
        self.log.info("Target Tachos: {:.4f}".format(degrees_to_turn))
        self.left_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.base_left_speed), brake=True,
                                       block=False)
        self.right_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.base_right_speed), brake=True,
                                        block=True)
        self.update_odometry()
        self.reset_motors()

    def turn_degrees(self, degrees: float):
        self.log.info("\n||> ROTATING {:.4f} deg...".format(degrees))
        wheel_degrees = self.rotation_to_wheel_degrees(degrees)
        self.log.debug("Target Tachos: {:.4f}".format(wheel_degrees))
        self.left_motor.on_for_degrees(degrees=-wheel_degrees, speed=SpeedPercent(self.base_left_speed), brake=True,
                                       block=False)
        self.right_motor.on_for_degrees(degrees=wheel_degrees, speed=SpeedPercent(self.base_right_speed), brake=True,
                                        block=True)
        self.update_odometry()
        self.reset_motors()

    def run_forever(self):
        self.log.info("\n||> MOVING IN A STRAIGHT LINE...")
        self.left_motor.speed_sp = int(self.base_left_speed / 100 * self.left_motor.max_speed)
        self.right_motor.speed_sp = int(self.base_right_speed / 100 * self.right_motor.max_speed)
        self.left_motor.run_forever()
        self.right_motor.run_forever()

    def turn_forever(self, center_of_rotation: Vector = None, clockwise: bool = False):
        self.log.info(
            "\n||> ROTATING {}...".format("AROUND {}".format(center_of_rotation) if center_of_rotation else "IN PLACE")
        )
        if center_of_rotation:
            distance_to_center = (center_of_rotation - self.pos).length
            robot_cor_angle = math.degrees(
                center_of_rotation.dot(self.look_at) / (center_of_rotation.length * self.look_at.length)
            )
            if robot_cor_angle != 90:
                self.turn_degrees(90 - robot_cor_angle)
            left_radius = 2*distance_to_center - (self.wheel_base / 2)
            right_radius = 2*distance_to_center + (self.wheel_base / 2)
            speed_ratio = right_radius / left_radius
            self.log.debug("Distance to CoR: {}".format(distance_to_center))
            self.log.debug("Speed ratio: {}".format(speed_ratio))
            self.left_motor.speed_sp = int((self.base_left_speed * speed_ratio) / 100 * self.left_motor.max_speed)
            self.right_motor.speed_sp = int(1.1*(self.base_right_speed / speed_ratio) / 100 * self.left_motor.max_speed)
            # self.left_motor.speed_sp = self.left_motor.speed_sp if clockwise else -self.left_motor.speed_sp
            # self.right_motor.speed_sp = -self.right_motor.speed_sp if clockwise else self.right_motor.speed_sp
        else:
            self.left_motor.speed_sp = int(self.base_left_speed / 100 * self.left_motor.max_speed)
            self.right_motor.speed_sp = int(self.base_right_speed / 100 * self.right_motor.max_speed)
            self.left_motor.speed_sp = self.left_motor.speed_sp if clockwise else -self.left_motor.speed_sp
            self.right_motor.speed_sp = -self.right_motor.speed_sp if clockwise else self.right_motor.speed_sp
        self.left_motor.run_forever()
        self.right_motor.run_forever()

    def stop(self):
        self.log.info("||> STOP MOTORS".format(SpeedPercent(self.base_left_speed)))
        self.left_motor.stop()
        self.right_motor.stop()
        self.update_odometry()
        self.reset_motors()

    def look_towards(self, point: Vector):
        self.log.info("||> LOOKING TOWARDS: {}".format(point))
        self.turn_degrees(point.angle_with(self.look_at, in_degrees=True))

    def rotate_to_match(self, new_theta):
        self.log.info("||> ROTATING TO MATCH: {:.4f} rad from {} rad".format(new_theta, self.theta))
        self.turn_degrees(math.degrees(new_theta - self.theta))

    def store_position(self):
        self.stored_theta = self.theta
        self.stored_position = Vector(self.pos.x, self.pos.y)

    def move_to_stored_position(self):
        self.log.info("||> RETURNING TO STORED POSITION: {}".format(self.stored_position))
        return_direction = self.stored_position - self.pos
        angle_with_return_dir = return_direction.angle_with(self.look_at)
        if 0 != angle_with_return_dir != 180:
            self.turn_degrees(angle_with_return_dir)
        self.move_straight(return_direction.length if angle_with_return_dir == 0 else -return_direction.length)
        self.rotate_to_match(self.stored_theta)

    # ****************************************************************
    # Ultrasonic Sensor
    # ****************************************************************

    def is_detecting_obstacle_within(self, max_range) -> bool:
        return self.ultrasonic_sensor.distance_centimeters <= max_range

    def scan_until_not_detected(self, initial_obstacle_distance: float, clockwise: bool, restore: bool = True) -> float:
        initial_speed = self.speed
        self.speed = 4
        initial_theta = self.theta
        current_dis = initial_obstacle_distance

        # Scan until obstacle is no longer detected
        self.turn_forever(clockwise=clockwise)
        while current_dis <= initial_obstacle_distance * 1.1:
            self.update_odometry()
            current_dis = self.ultrasonic_sensor.distance_centimeters
            self.log.info(
                "Scanning obstacle to the {}: {} cm [Initial distance: {}]".format(
                    "right" if clockwise else "left", current_dis, initial_obstacle_distance
                )
            )
        self.stop()
        theta_limit = self.theta
        self.log.info("Stopped detecting obstacle at {}".format(math.degrees(theta_limit)))
        if restore:
            self.log.info("Turning back to initial theta...")
            self.rotate_to_match(initial_theta)
        self.speed = initial_speed

        return theta_limit

    def approach_obstacle(self, security_distance: float = 10):
        self.log.info("||> APPROACHING OBSTACLE...")
        self.run_forever()
        while self.ultrasonic_sensor.distance_centimeters > security_distance:
            self.update_odometry()
        self.stop()

    def rotate_to_avoid_obstacle(self, obstacle_dis, clockwise: bool, safety_theta: float = 10):
        self.log.info("||> ROTATING TO AVOID OBSTACLE...")
        # self.scan_until_not_detected(obstacle_dis, clockwise, restore=False)
        left_wheel_pos = self.look_at.rotate_degrees(90).to_length(self.wheel_base / 2)
        right_wheel_pos = self.look_at.rotate_degrees(-90).to_length(self.wheel_base / 2)
        left_wheel_ray = left_wheel_pos + self.look_at.to_length(self.ultrasonic_sensor.distance_centimeters)
        right_wheel_ray = right_wheel_pos + self.look_at.to_length(self.ultrasonic_sensor.distance_centimeters)
        if clockwise:
            self.turn_degrees(left_wheel_ray.angle_with(right_wheel_ray, in_degrees=True) - safety_theta)
        else:
            self.turn_degrees(right_wheel_ray.angle_with(left_wheel_ray, in_degrees=True) + safety_theta)

    def scan_for_closest_obstacle(self, search_cone_degrees: float = 120, max_range: float = None, restore: bool = True) -> None or Tuple[float, float]:
        self.log.info("\n||> SEARCHING FOR CLOSEST OBSTACLE...")
        self.log.debug("Search cone: {} deg".format(search_cone_degrees))
        if max_range:
            self.log.debug("Search range: {} cm".format(max_range))
        initial_speed = self.speed
        self.speed = 8
        initial_theta = self.theta
        search_cone_half_radians = math.radians(search_cone_degrees / 2)
        search_max_theta = initial_theta + search_cone_half_radians
        search_min_theta = initial_theta - search_cone_half_radians
        self.turn_degrees(search_cone_degrees / 2)
        min_dis_theta = self.theta
        min_dis = max_range if max_range else self.ultrasonic_sensor.distance_centimeters
        self.log.debug("Initial distance: {:.4f} {:.4f}".format(min_dis, min_dis_theta))

        self.turn_forever(clockwise=True)
        while 0.7 * search_min_theta <= self.theta <= 1.3 * search_max_theta:
            self.update_odometry()
            self.log.debug("\nCurrent theta: {:.4f}  Target theta {:.4f}".format(
                math.degrees(self.theta), math.degrees(search_min_theta))
            )
            current_dis = self.ultrasonic_sensor.distance_centimeters
            current_theta = self.theta
            self.log.debug("Current obs dis: {:.2f}\n".format(current_dis))
            if current_dis < min_dis:
                min_dis = current_dis
                min_dis_theta = current_theta
        self.stop()
        if restore:
            self.rotate_to_match(initial_theta)
        else:
            self.rotate_to_match(min_dis_theta)
        self.speed = initial_speed
        self.log.debug("Robot angle: {:.2f} deg".format(math.degrees(self.theta)))

        if max_range and min_dis_theta == initial_theta:
            self.log.info("Detected no obstacle within range")
            return None
        self.log.info(
            "Detected closest obstacle at {:.2f} cm and {:.4f} deg ".format(min_dis, math.degrees(min_dis_theta))
        )
        return min_dis, min_dis_theta

    def get_obstacle_pos(self) -> Vector:
        obstacle_dis = self.ultrasonic_sensor.distance_centimeters
        left_theta_limit = self.scan_until_not_detected(obstacle_dis, clockwise=False)
        right_theta_limit = self.scan_until_not_detected(obstacle_dis, clockwise=True)
        beta = left_theta_limit - (left_theta_limit - right_theta_limit) / 2
        obstacle_pos = self.pos + Vector(obstacle_dis * math.cos(beta), obstacle_dis * math.sin(beta))
        self.log.info("Found obstacle at {}".format(obstacle_pos))
        return obstacle_pos

    def scan_for_second_closest_obstacle(self, clockwise: bool, search_cone_degrees: float = 120) -> Tuple[float, float]:
        self.log.info("\n||> SEARCHING FOR SECOND CLOSEST OBSTACLE...")
        self.log.debug("Search cone: {} deg".format(search_cone_degrees))
        initial_speed = self.speed
        self.speed = 8
        search_cone_radians = math.radians(search_cone_degrees)
        
        min_dis_theta = self.theta
        min_dis = self.ultrasonic_sensor.distance_centimeters
        self.log.info("Initial params: {:.4f} {:.4f}".format(min_dis, min_dis_theta))

        self.turn_forever(clockwise=clockwise)
        while 0.3 <= abs(self.theta - search_cone_radians) :
            self.update_odometry()
            self.log.info("\nCurrent theta: {:.4f}  Target theta {:.4f}".format(self.theta, search_cone_radians))
            current_dis = self.ultrasonic_sensor.distance_centimeters
            current_theta = self.theta
            self.log.info("Current obs dis: {:.2f}\n".format(current_dis))
            if current_dis < min_dis:
                min_dis = current_dis
                min_dis_theta = current_theta
        self.stop()
        self.rotate_to_match(min_dis_theta)
        self.speed = initial_speed
        self.log.debug("Robot angle: {:.2f} deg".format(self.theta))
        self.log.info("Detected closest obstacle at {:.2f} cm and {:.4f} deg ".format(min_dis, min_dis_theta))
        return min_dis, min_dis_theta

    # ****************************************************************
    # Other Sensors & functions
    # ****************************************************************

    def beep(self, n_times=1, delay=0):
        for _ in range(n_times):
            self.sound.beep()
            if delay:
                time.sleep(delay)

    def is_detecting_black(self) -> bool:
        black_threshold = 20
        reflectivity = self.color_sensor.reflected_light_intensity
        self.log.debug("Reflectivity: {}".format(reflectivity))
        return reflectivity < black_threshold


class RobotController(abc.ABC):

    def __init__(self, log_name: str = "sra_grupo4", log_level: int = logging.DEBUG):
        log_path = "/home/robot/SRA_G4_P3/{}.log".format(log_name)
        logging.basicConfig(filename=log_path, filemode='w+', level=log_level)
        self.log = logging.getLogger('ev3dev')
        self.config = ConfigParser()
        self.config.read("/home/robot/SRA_G4_P3/config.ini")
        self.robot = Robot(
            self.log,
            self.config.getfloat("Base", "wheel_diameter"),
            self.config.getfloat("Base", "wheel_base"),
            self.config.getfloat("Base", "base_left_speed"),
            self.config.getfloat("Base", "base_right_speed"),
            self.config.getfloat("Base", "initial_theta")
        )

    @abc.abstractmethod
    def move(self):
        pass

    def run(self):
        # self.robot.beep(2)
        self.move()
        # self.robot.beep(3)
