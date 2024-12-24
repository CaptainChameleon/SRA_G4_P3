#!/usr/bin/env python3

import abc
import logging
import math
import time
from configparser import ConfigParser

from robot import Robot
from utils import Vector


class RobotController(abc.ABC):
    
    def __init__(self, log, robot: Robot):
        self.log = log
        self.robot = robot
        self.moving = False

    def start(self):
        self.robot.beep(2)
        self.moving = True
        
    def stop(self):
        self.moving = False
        self.robot.beep(3)
        
    @abc.abstractmethod
    def move(self):
        pass
 
    def run(self):
        self.start()
        while self.moving:
            self.move()
        
        
class SquareTestController(RobotController):
    
    def wait(self, duration: int = 5):
        self.robot.beep(2)
        time.sleep(duration)
    
    def make_square(self, side_len: float = 20, clockwise: bool = True):
        degrees = -90 if clockwise else 90
        for _ in range(4):
            self.robot.move_straight(side_len)
            self.robot.turn_degrees(degrees)

    def bilateral_test(self, square_size=40, wait_time=10):
        self.make_square()
        self.wait(duration=wait_time)
        self.make_square(clockwise=False)

    def apartado_a(self):
        self.make_square()

    def apartado_b(self):
        for _ in range(10):
            self.make_square(50)

        #self.wait(20)
        
        #for _ in range(10):
            #self.make_square(50, clockwise=False)

    def apartado_c(self):
        # Determinar si existe error sistemático.
        pass

    def apartado_d(self):
        # Forzar la aparición de error sistemático y comprobar si el test aplica las correcciones adecuadas.
        pass

    def apartado_e(self):
        # Construir una plataforma diferencial con una geometría diferente y comparar y justificar los resultados.
        pass

    def apartado_f(self):
        # Caracterización del error de otros sensores: ultrasonidos, orientación
        pass

    def move(self):
        #self.apartado_a()
        self.apartado_b()
        #self.apartado_c()
        #self.apartado_d()
        #self.apartado_e()
        #self.apartado_f()
        self.stop()


class ParkingController(RobotController):
    
    def __init__(self, log, robot: Robot):
        super().__init__(log, robot)
        self.obstacle_pos_1 = None
        self.obstacle_pos_2 = None
    
    def search_for_first_obstacle(self):
        initial_theta = self.robot.theta
        search_cone_angle = math.radians(60)
        search_max_theta = initial_theta + search_cone_angle
        search_min_theta = initial_theta - search_cone_angle

        self.log.info("||> SEARCHING FOR FIRST OBSTACLE [SEARCH CONE: {:.4f} deg]".format(
            2 * math.degrees(search_cone_angle))
        )
        self.robot.turn_degrees(math.degrees(search_cone_angle))
        min_dis_angle = self.robot.theta
        min_dis = self.robot.ultrasonic_sensor.distance_centimeters
        self.log.debug("Initial distance: {:.4f} {:.4f}".format(min_dis, min_dis_angle))

        self.robot.turn_forever(clockwise=True)
        while 0.7*search_min_theta <= self.robot.theta <= 1.3*search_max_theta:
            self.robot.update_odometry()
            self.log.debug("\nCurrent theta: {:.4f}  Target theta {:.4f}".format(
                math.degrees(self.robot.theta), math.degrees(search_min_theta))
            )
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            current_theta = self.robot.theta
            self.log.debug("Current obs dis: {:.2f}\n".format(current_dis))
            if current_dis < min_dis:
                min_dis = current_dis
                min_dis_angle = current_theta
        self.robot.stop()

        self.log.info("Detected closest obstacle at {:.2f} cm and {:.4f} deg ".format(
            min_dis, math.degrees(min_dis_angle))
        )
        self.log.debug("Robot angle: {:.2f} deg".format(math.degrees(self.robot.theta)))
        self.robot.turn_degrees(math.degrees(min_dis_angle - self.robot.theta))
        self.scan_obstacle(min_dis)

    def search_for_second_obstacle(self):
        pass

    def scan_obstacle(self, obstacle_dis) -> Vector:
        initial_theta = self.robot.theta
        current_dis = obstacle_dis
        self.robot.turn_forever()
        while current_dis <= obstacle_dis * 1.1:
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            self.log.debug("Scanning obstacle to the left: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.robot.stop()
        mediatriz_izq = self.robot.theta
        self.robot.turn_degrees(math.degrees(-(mediatriz_izq-initial_theta)))
        self.log.debug("Found obstacle, turned {} and turning back to initial theta".format(mediatriz_izq))

        current_dis = obstacle_dis
        self.robot.turn_forever(clockwise=True)
        while current_dis <= obstacle_dis * 1.1:
            self.log.debug("Scanning obstacle to the right: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            self.log.debug("Scanning obstacle to the right: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.robot.stop()
        mediatriz_der = self.robot.theta
        self.robot.turn_degrees(math.degrees(initial_theta-mediatriz_der))
        self.robot.pos = Vector(0, 0)

        beta = mediatriz_izq - (mediatriz_izq - mediatriz_der) / 2
        obstacle_pos = Vector(obstacle_dis * math.cos(beta), obstacle_dis * math.sin(beta))
        self.log.info("Found obstacle at {}".format(obstacle_pos))
        return obstacle_pos

    def move(self):
        self.search_for_first_obstacle()
        self.stop()
        # self.robot.run_forever()
        # while True:
        #     dis_to_obstacle = self.ultrasonic_sensor.distance_centimeters
        #     if dis_to_obstacle <= self.MAX_RANGE_DIS:
        #         self.robot.stop()
        #         self.scan_obstacle(dis_to_obstacle)
        #         break
            # self.log.info("Mean distance: {} cm\n".format(dis_to_obstacle))
            # if dis_to_obstacle < self.MAX_RANGE_DIS:
            #     self.robot.move_straight(-(self.MAX_RANGE_DIS - dis_to_obstacle))


if __name__ == '__main__':
    logging.basicConfig(filename="/home/robot/SRA_G4_P3/sra_grupo4.log",
                        filemode='w+',
                        level=logging.DEBUG)
    logger = logging.getLogger('ev3dev')
    config = ConfigParser()
    config.read("../config.ini")
    robot = Robot(
        logger, 
        config.getfloat("Base", "wheel_diameter"),
        config.getfloat("Base", "wheel_base"),
        config.getfloat("Base", "base_speed"),
        config.getfloat("Base", "speed_correction"),
        config.getfloat("Base", "initial_theta")
    )
    #robot_controller = ParkingController(logger, robot)
    robot_controller = SquareTestController(logger, robot)
    robot_controller.run()
