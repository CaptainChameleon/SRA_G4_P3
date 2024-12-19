#!/usr/bin/env python3
import logging
import math
import time
from statistics import mean

from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sound import Sound

from utils import Vector
from wheels import WheelPlatform


class Robot:

    MAX_RANGE_DIS = 20  # centimeters

    def __init__(self, log):
        self.log = log
        self.wheels = WheelPlatform(self.log)
        self.ultrasonic_sensor = UltrasonicSensor()
        self.sound = Sound()
        self.obstacle_pos_1 = None
        self.obstacle_pos_2 = None

    def beep(self, n=1):
        for _ in range(n):
            self.sound.beep()

    def wait(self, duration: int = 5):
        self.sound.beep()
        self.sound.beep()
        time.sleep(duration)

    def make_square(self, side_len: float = 20, clockwise: bool = True):
        degrees = -90 if clockwise else 90
        for _ in range(4):
            self.wheels.move_straight(side_len)
            self.wheels.turn_degrees(degrees)

    def bilateral_test(self, square_size=40, wait_time=10):
        self.make_square()
        self.wait(duration=wait_time)
        self.make_square(clockwise=False)

    def scan_obstacle(self, obstacle_dis):
        initial_theta = self.wheels.theta
        current_dis = obstacle_dis
        self.wheels.turn_forever()
        while current_dis <= obstacle_dis * 1.1:
            current_dis = self.ultrasonic_sensor.distance_centimeters
            self.log.info("Scanning obstacle to the left: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.wheels.stop()
        mediatriz_izq = self.wheels.theta
        self.wheels.turn_degrees(math.degrees(-(mediatriz_izq-initial_theta)))
        self.log.info("Found obstacle, turned {} and turning back to initial theta".format(mediatriz_izq))

        time.sleep(0.5)

        current_dis = obstacle_dis
        self.wheels.turn_forever(clockwise=True)
        while current_dis <= obstacle_dis * 1.1:
            self.log.info("Scanning obstacle to the right: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
            current_dis = self.ultrasonic_sensor.distance_centimeters
            self.log.info("Scanning obstacle to the right: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.wheels.stop()
        mediatriz_der = self.wheels.theta
        self.wheels.turn_degrees(math.degrees(initial_theta-mediatriz_der))
        self.wheels.pos = Vector(0, 0)

        beta = mediatriz_izq - (mediatriz_izq - mediatriz_der) / 2
        self.obstacle_pos_1 = Vector(obstacle_dis * math.cos(beta), obstacle_dis * math.sin(beta))
        self.log.info("Found obstacle at {}".format(self.obstacle_pos_1))

    def run(self):
        self.beep(2)
        self.wheels.run_forever()
        while True:
            dis_to_obstacle = self.ultrasonic_sensor.distance_centimeters
            if dis_to_obstacle <= self.MAX_RANGE_DIS:
                self.wheels.stop()
                self.scan_obstacle(dis_to_obstacle)
                break
            # self.log.info("Mean distance: {} cm\n".format(dis_to_obstacle))
            # if dis_to_obstacle < self.MAX_RANGE_DIS:
            #     self.wheels.move_straight(-(self.MAX_RANGE_DIS - dis_to_obstacle))
        self.beep(3)


if __name__ == '__main__':
    logging.basicConfig(filename="/home/robot/SRA_G4_P3/sra_grupo4.log",
                        filemode='w+',
                        format='%(asctime)s,%(msecs)d %(message)s',
                        datefmt='%H:%M:%S',
                        level=logging.DEBUG)
    logger = logging.getLogger('ev3dev')
    robot = Robot(logger)

    robot.run()

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
