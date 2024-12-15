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
    WHEEL_BASE = 11.5  # centimeters

    def __init__(self, log):
        self.log = log
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        self.sound = Sound()

        self.pos = Vector(0, 0)
        self.look_at = Vector(0, 1)
        self.theta = 0

    def distance_to_wheel_rotation(self, distance: float) -> float:
        return (distance / (math.pi * self.WHEEL_DIAMETER)) * 360

    def rotation_to_wheel_degrees(self, degrees: float) -> float:
        wheel_travel_distance = (degrees/360) * math.pi * self.WHEEL_BASE
        return self.distance_to_wheel_rotation(wheel_travel_distance)

    def update_odometry(self):
        # Get current tacho counts
        left_tacho = self.left_motor.position
        right_tacho = self.right_motor.position

        # Reset motor tachos
        self.left_motor.reset()
        self.right_motor.reset()

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
        dx = d_center * math.cos(self.theta)
        dy = d_center * math.sin(self.theta)
        self.pos.x += dx
        self.pos.y += dy

        # Update look_at vector
        self.look_at.x = math.cos(self.theta)
        self.look_at.y = math.sin(self.theta)

        # Log updated values
        self.log.info("Updated Position: {}".format(self.pos))
        self.log.info("Updated Orientation: {:.8f} radians".format(self.theta))
        self.log.info("Look-at Vector: {}".format(self.look_at))

    def move_forward(self, distance: float, speed=20):
        self.log.info("- Moving forward {}cm -".format(distance))
        self.log.info("Motor L pos: {}".format(self.left_motor.position))
        self.log.info("Motor R pos: {}".format(self.right_motor.position))
        degrees_to_turn = self.distance_to_wheel_rotation(distance)
        self.left_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(speed), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(speed), brake=True, block=True)
        self.log.info("Motor L pos: {}".format(self.left_motor.position))
        self.log.info("Motor R pos: {}".format(self.right_motor.position))
        self.update_odometry()
        self.log.info("\n")

    def turn_degrees(self, degrees: float, speed=20):
        self.log.info("- Rotating {}deg -".format(degrees))
        self.log.info("Motor L pos: {}".format(self.left_motor.position))
        self.log.info("Motor R pos: {}".format(self.right_motor.position))
        wheel_degrees = self.rotation_to_wheel_degrees(degrees)
        self.left_motor.on_for_degrees(degrees=wheel_degrees, speed=SpeedPercent(speed), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=-wheel_degrees, speed=SpeedPercent(speed), brake=True, block=True)
        self.log.info("Motor L pos: {}".format(self.left_motor.position))
        self.log.info("Motor R pos: {}".format(self.right_motor.position))
        self.update_odometry()
        self.log.info("\n")

    def wait(self, duration: int = 5):
        self.sound.beep()
        self.sound.beep()
        time.sleep(duration)

    def make_square(self, side_len: float = 20, clockwise: bool = True):
        degrees = 90 if clockwise else -90
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

    robot.bilateral_test(square_size=20, wait_time=5)
    # robot.make_square()
    # robot.turn_degrees(90)
    # robot.wait()
    # robot.turn_degrees(-90)

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
