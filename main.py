#!/usr/bin/env python3
import logging
import math
import time
from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sound import Sound


class Robot:

    WHEEL_DIAMETER = 5.6  # centimeters
    WHEEL_BASE = 11.5  # centimeters
    DEFAULT_WAIT_TIME = 5

    def __init__(self, log):
        self.log = log
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        self.sound = Sound()

    def distance_to_wheel_rotation(self, distance: float) -> float:
        return (distance / (math.pi * self.WHEEL_DIAMETER)) * 360

    def rotation_to_wheel_degrees(self, degrees: float) -> float:
        wheel_travel_distance = (degrees/360) * math.pi * self.WHEEL_BASE
        return self.distance_to_wheel_rotation(wheel_travel_distance)

    def move_forward(self, distance: float, speed=20):
        degrees_to_turn = self.distance_to_wheel_rotation(distance)
        self.left_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(speed), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(speed), brake=True, block=True)

    def turn_degrees(self, degrees: float, speed=20):
        wheel_degrees = self.rotation_to_wheel_degrees(degrees)
        self.left_motor.on_for_degrees(degrees=wheel_degrees, speed=SpeedPercent(speed), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=-wheel_degrees, speed=SpeedPercent(speed), brake=True, block=True)

    def wait(self):
        self.sound.beep()
        self.sound.beep()
        time.sleep(self.DEFAULT_WAIT_TIME)

    def make_square(self, clockwise: bool = True):
        degrees = 90 if clockwise else -90
        for _ in range(4):
            self.move_forward(40)
            self.turn_degrees(degrees)

    def bilateral_test(self):
        self.make_square()
        self.wait()
        self.make_square(clockwise=False)


if __name__ == '__main__':
    logger = logging.getLogger('ev3dev')
    logger.addHandler(logging.FileHandler('sra_grupo4.log'))
    robot = Robot(logger)

    robot.bilateral_test()

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
