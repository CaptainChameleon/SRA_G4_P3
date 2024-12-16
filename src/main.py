#!/usr/bin/env python3
import abc
import logging
import math
import time
from time import sleep

from utils import Vector

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sound import Sound


class Robot:

    WHEEL_DIAMETER = 5.6  # centimeters
    WHEEL_BASE = 11.36  # 11.5  # centimeters

    CORRECTED_SPEED_R = 20.02  # 20.47
    CORRECTED_SPEED_L = 19.98  # 19.54

    def __init__(self, log):
        self.log = log
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        self.sound = Sound()

        self.pos = Vector(0, 0)
        self.look_at = Vector(0, 1)
        self.theta = math.pi / 2  # radians

    def distance_to_wheel_rotation(self, distance: float) -> float:
        return (distance / (math.pi * self.WHEEL_DIAMETER)) * 360

    def rotation_to_wheel_degrees(self, degrees: float) -> float:
        wheel_travel_distance = (degrees/360) * math.pi * self.WHEEL_BASE
        return self.distance_to_wheel_rotation(wheel_travel_distance)

    def update_odometry(self):
        self.log.info("Updating odometry")

        # Get current tacho counts
        left_tacho = self.left_motor.position
        right_tacho = self.right_motor.position
        self.log.info("Motor L pos after movement: {}".format(self.left_motor.position))
        self.log.info("Motor R pos after movement: {}".format(self.right_motor.position))

        # Reset motor tachos
        self.left_motor.reset()
        self.right_motor.reset()
        self.log.info("Motor L pos after reset: {}".format(self.left_motor.position))
        self.log.info("Motor R pos after reset: {}".format(self.right_motor.position))

        # Compute wheel distances
        d_left = (left_tacho / 360) * math.pi * self.WHEEL_DIAMETER
        d_right = (right_tacho / 360) * math.pi * self.WHEEL_DIAMETER

        # Compute displacement and orientation change
        d_center = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / self.WHEEL_BASE

        # Update robot's orientation (theta)
        self.theta += delta_theta
        self.theta %= (2 * math.pi)  # Keep theta within [0, 2*pi]

        # Update position
        dx = d_center * math.sin(self.theta)
        dy = d_center * math.cos(self.theta)
        self.pos.x += dx
        self.pos.y += dy

        # Update look_at vector
        self.look_at.x = math.sin(self.theta)
        self.look_at.y = math.cos(self.theta)

        # Log updated values
        self.log.info("Updated Position: {}".format(self.pos))
        self.log.info("Updated Orientation: {:.8f} radians".format(self.theta))
        self.log.info("Look-at Vector: {}".format(self.look_at))

    def move_forward(self, distance: float, speed=20):
        self.log.info("** Moving forward {}cm **".format(distance))
        degrees_to_turn = self.distance_to_wheel_rotation(distance)
        self.log.info("Target Tachos: {}".format(degrees_to_turn))
        self.left_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.CORRECTED_SPEED_L), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.CORRECTED_SPEED_R), brake=True, block=True)
        self.update_odometry()
        self.log.info("\n")

    def turn_degrees(self, degrees: float, speed=20):
        self.log.info("** Rotating {}deg **".format(degrees))
        wheel_degrees = self.rotation_to_wheel_degrees(degrees)
        self.log.info("Target Tachos: {}".format(wheel_degrees))
        self.left_motor.on_for_degrees(degrees=-wheel_degrees, speed=SpeedPercent(self.CORRECTED_SPEED_L), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=wheel_degrees, speed=SpeedPercent(self.CORRECTED_SPEED_R), brake=True, block=True)
        self.update_odometry()
        self.log.info("\n")

    def wait(self, duration: int = 5):
        self.sound.beep()
        self.sound.beep()
        time.sleep(duration)

    def make_square(self, side_len: float = 20, clockwise: bool = True):
        degrees = -90 if clockwise else 90
        for _ in range(4):
            self.move_forward(side_len)
            self.turn_degrees(degrees)

    def bilateral_test(self, square_size=40, wait_time=10):
        self.make_square()
        self.wait(duration=wait_time)
        self.make_square(clockwise=False)


if __name__ == '__main__':
    logging.basicConfig(filename="/home/robot/SRA_G4_P3/sra_grupo4.log",
                        filemode='w+',
                        format='%(asctime)s,%(msecs)d %(message)s',
                        datefmt='%H:%M:%S',
                        level=logging.DEBUG)
    logger = logging.getLogger('ev3dev')
    robot = Robot(logger)

    robot.make_square(40, clockwise=True)

    # # APARTADO A
    # robot.make_square()

    # robot.wait()

    # # APARTADO B
    # for _ in range(10):
    #     robot.make_square()
    #
    # # APARTADO C
    # for _ in range(10):
    #     robot.make_square(clockwise=False)

    # robot.wait()

    # APARTADO D - Forzar la aparición de error sistemático y comprobar si el test aplica las correcciones adecuadas.

    # APARTADO E - Construir una plataforma diferencial con una geometría diferente y comparar y
    # justificar los resultados.

    # APARTADO F - Caracterización del error de otros sensores: ultrasonidos, orientación
